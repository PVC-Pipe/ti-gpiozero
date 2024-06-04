# vim: set fileencoding=utf-8:
#
# GPIO Zero: a library for controlling the Raspberry Pi's GPIO pins
#
# Copyright (c) 2016-2023 Dave Jones <dave@waveform.org.uk>
# Copyright (c) 2020 Fangchen Li <fangchen.li@outlook.com>
#
# SPDX-License-Identifier: BSD-3-Clause

import re
from threading import RLock
from types import MethodType
from weakref import ref, WeakMethod
import warnings

try:
    from spidev import SpiDev
except ImportError:
    SpiDev = None

from . import Factory, Pin, BoardInfo, HeaderInfo, PinInfo, data
from .data import SPI_HARDWARE_PINS
from ..compat import frozendict
from ..devices import Device
from ..exc import (
    GPIOPinInUse,
    PinInvalidPin,
    PinNoPins,
    PinNonPhysical,
    PinUnknownPi,
    SPIBadArgs,
    SPISoftwareFallback,
)

class TiBoardInfo(BoardInfo):
    __slots__ = () # workaround python issue #24931

    @classmethod
    def from_revision(cls, revision):
        # TODO: BeagleY-AI does not have a revision number
        if revision & 0x800000:
            # New-style revision, parse information from bit-pattern:
            #
            # MSB -----------------------> LSB
            # NOQuuuWuFMMMCCCCPPPPTTTTTTTTRRRR
            #
            # N        - Overvoltage (0=allowed, 1=disallowed)
            # O        - OTP programming (0=allowed, 1=disallowed)
            # Q        - OTP read (0=allowed, 1=disallowed)
            # u        - Unused
            # W        - Warranty bit (0=intact, 1=voided by overclocking)
            # F        - New flag (1=valid new-style revision, 0=old-style)
            # MMM      - Memory size (see memory dict below)
            # CCCC     - Manufacturer (see manufacturer dict below)
            # PPPP     - Processor (see soc dict below)
            # TTTTTTTT - Type (see model dict below)
            # RRRR     - Revision (0, 1, 2, etc.)
            revcode_memory       = (revision & 0x700000) >> 20
            revcode_manufacturer = (revision & 0xf0000)  >> 16
            revcode_processor    = (revision & 0xf000)   >> 12
            revcode_type         = (revision & 0xff0)    >> 4
            revcode_revision     = (revision & 0x0f)
            model = {
                0x0:  'A',
                0x1:  'B',
                0x2:  'A+',
                0x3:  'B+',
                0x4:  '2B',
                0x6:  'CM',
                0x8:  '3B',
                0x9:  'Zero',
                0xa:  'CM3',
                0xc:  'Zero W',
                0xd:  '3B+',
                0xe:  '3A+',
                0x10: 'CM3+',
                0x11: '4B',
                0x12: 'Zero2W',
                0x13: '400',
                0x14: 'CM4',
                0x17: '5B',
                }.get(revcode_type, '???')
            if model in ('A', 'B'):
                pcb_revision = {
                    0: '1.0', # is this right?
                    1: '1.0',
                    2: '2.0',
                    }.get(revcode_revision, 'Unknown')
            else:
                pcb_revision = f'1.{revcode_revision}'
            soc = {
                0: 'BCM2835',
                1: 'BCM2836',
                2: 'BCM2837',
                3: 'BCM2711',
                4: 'BCM2712',
                }.get(revcode_processor, 'Unknown')
            manufacturer = {
                0: 'Sony',
                1: 'Egoman',
                2: 'Embest',
                3: 'Sony Japan',
                4: 'Embest',
                5: 'Stadium',
                }.get(revcode_manufacturer, 'Unknown')
            memory = {
                0: 256,
                1: 512,
                2: 1024,
                3: 2048,
                4: 4096,
                5: 8192,
                6: 16384,
                }.get(revcode_memory, None)
            released = {
                'A':      '2013Q1',
                'B':      '2012Q1' if pcb_revision == '1.0' else '2012Q4',
                'A+':     '2014Q4' if memory == 512 else '2016Q3',
                'B+':     '2014Q3',
                '2B':     '2015Q1' if pcb_revision in ('1.0', '1.1') else '2016Q3',
                'CM':     '2014Q2',
                '3B':     '2016Q1' if manufacturer in ('Sony', 'Embest') else '2016Q4',
                'Zero':   '2015Q4' if pcb_revision == '1.2' else '2016Q2',
                'CM3':    '2017Q1',
                'Zero W': '2017Q1',
                '3B+':    '2018Q1',
                '3A+':    '2018Q4',
                'CM3+':   '2019Q1',
                '4B':     '2020Q2' if memory == 8192 else '2019Q2',
                'CM4':    '2020Q4',
                '400':    '2020Q4',
                'Zero2W': '2021Q4',
                '5B':     '2023Q4',
                }.get(model, 'Unknown')
            storage = {
                'A':    'SD',
                'B':    'SD',
                'CM':   'eMMC',
                'CM3':  'eMMC / off-board',
                'CM3+': 'eMMC / off-board',
                'CM4':  'eMMC / off-board',
                }.get(model, 'MicroSD')
            usb = {
                'A':      1,
                'A+':     1,
                'Zero':   1,
                'Zero W': 1,
                'Zero2W': 1,
                'B':      2,
                'CM':     1,
                'CM3':    1,
                '3A+':    1,
                'CM3+':   1,
                'CM4':    2,
                '400':    3,
                }.get(model, 4)
            usb3 = {
                '4B':     2,
                '400':    2,
                '5B':     2,
                }.get(model, 0)
            ethernet = {
                'A':      0,
                'A+':     0,
                'Zero':   0,
                'Zero W': 0,
                'Zero2W': 0,
                'CM':     0,
                'CM3':    0,
                '3A+':    0,
                'CM3+':   0,
                }.get(model, 1)
            eth_speed = {
                'B':      100,
                'B+':     100,
                '2B':     100,
                '3B':     100,
                '3B+':    300,
                '4B':     1000,
                '400':    1000,
                'CM4':    1000,
                '5B':     1000,
                }.get(model, 0)
            bluetooth = wifi = {
                '3B':     True,
                'Zero W': True,
                'Zero2W': True,
                '3B+':    True,
                '3A+':    True,
                '4B':     True,
                '400':    True,
                'CM4':    True,
                '5B':     True,
                }.get(model, False)
            csi = {
                'Zero':   0 if pcb_revision == '1.2' else 1,
                'CM':     2,
                'CM3':    2,
                'CM3+':   2,
                '400':    0,
                'CM4':    2,
                '5B':     2,
                }.get(model, 1)
            dsi = {
                'Zero':   0,
                'Zero W': 0,
                'Zero2W': 0,
                '5B':     2,
                }.get(model, csi)
            headers = {
                'A':      {'P1': data.REV2_P1, 'P5': data.REV2_P5, 'P6': data.REV2_P6, 'P2': data.PI1_P2, 'P3': data.PI1_P3},
                'B':      {'P1': data.REV1_P1, 'P2': data.PI1_P2, 'P3': data.PI1_P3} if pcb_revision == '1.0' else
                          {'P1': data.REV2_P1, 'P5': data.REV2_P5, 'P6': data.REV2_P6, 'P2': data.PI1_P2, 'P3': data.PI1_P3},
                'B+':     {'J8': data.PLUS_J8, 'RUN': data.ZERO_RUN},
                'CM':     {'SODIMM': data.CM_SODIMM},
                'CM3':    {'SODIMM': data.CM3_SODIMM},
                'CM3+':   {'SODIMM': data.CM3_SODIMM},
                'Zero':   {'J8': data.PLUS_J8, 'RUN': data.ZERO_RUN, 'TV': data.ZERO_TV},
                'Zero W': {'J8': data.PLUS_J8, 'RUN': data.ZERO_RUN, 'TV': data.ZERO_TV},
                '2B':     {'J8': data.PLUS_J8, 'RUN': data.ZERO_RUN},
                '3B':     {'J8': data.PLUS_J8, 'RUN': data.ZERO_RUN},
                '3A+':    {'J8': data.PLUS_J8, 'RUN': data.PLUS_RUN},
                '3B+':    {'J8': data.PLUS_J8, 'RUN': data.PLUS_RUN, 'POE': data.PLUS_POE},
                '4B':     {'J8': data.PI4_J8, 'J2': data.PI4_J2, 'J14': data.PI4_J14},
                '400':    {'J8': data.PI4_J8},
                'CM4':    {'J8': data.PI4_J8, 'J1': data.CM4_J1, 'J2': data.CM4_J2, 'J3': data.CM4_J3, 'J6': data.CM4_J6, 'J9': data.CM4_J9},
                '5B':     {'J8': data.PI4_J8, 'J2': data.PI5_J2, 'J7': data.PI5_J7, 'J14': data.PI4_J14},
                }.get(model, {'J8': data.PLUS_J8})
            board = {
                'A':      data.A_BOARD,
                'B':      data.REV1_BOARD if pcb_revision == '1.0' else data.REV2_BOARD,
                'A+':     data.APLUS_BOARD,
                'CM':     data.CM_BOARD,
                'CM3':    data.CM_BOARD,
                'CM3+':   data.CM3PLUS_BOARD,
                'Zero':   data.ZERO12_BOARD if pcb_revision == '1.2' else data.ZERO13_BOARD,
                'Zero W': data.ZERO13_BOARD,
                'Zero2W': data.ZERO2_BOARD,
                '3A+':    data.A3PLUS_BOARD,
                '3B+':    data.B3PLUS_BOARD,
                '4B':     data.B4_BOARD,
                'CM4':    data.CM4_BOARD,
                '400':    data.P400_BOARD,
                '5B':     data.B5_BOARD,
                }.get(model, data.BPLUS_BOARD)
        else:
            # Old-style revision, use the lookup table
            try:
                (
                    model,
                    pcb_revision,
                    released,
                    soc,
                    manufacturer,
                    memory,
                    storage,
                    usb,
                    ethernet,
                    wifi,
                    bluetooth,
                    csi,
                    dsi,
                    headers,
                    board,
                    ) = data.PI_REVISIONS[revision]
                usb3 = 0
                eth_speed = ethernet * 100
            except KeyError:
                raise PinUnknownPi(f'unknown old-style revision "{revision:x}"')
        headers = frozendict({
            header: HeaderInfo(
                name=header, rows=rows, columns=columns,
                pins=frozendict({
                    number: cls._make_pin(
                        header, number, row + 1, col + 1, functions)
                    for number, functions in header_data.items()
                    for row, col in (divmod(number - 1, 2),)
                })
            )
            for header, (rows, columns, header_data) in headers.items()
        })
        return cls(
            f'{revision:04x}',
            model,
            pcb_revision,
            released,
            soc,
            manufacturer,
            memory,
            storage,
            usb,
            usb3,
            ethernet,
            eth_speed,
            wifi,
            bluetooth,
            csi,
            dsi,
            headers,
            board,
            )

    @staticmethod
    def _make_pin(header, number, row, col, interfaces):
        pull = 'up' if number in (3, 5) and header in ('P1', 'J8') else ''
        phys_name = f'{header}:{number}'
        names = {phys_name}
        if header in ('P1', 'J8', 'SODIMM'):
            names.add(f'BOARD{number}')
        try:
            name = interfaces['gpio']
            gpio = int(name[4:])
            names.add(name)
            names.add(gpio)
            names.add(str(gpio))
            names.add(f'BCM{gpio}')
            try:
                wpi_map = {
                    'J8:3':  8,  'J8:5':  9,  'J8:7':  7,  'J8:8':  15,
                    'J8:10': 16, 'J8:11': 0,  'J8:12': 1,  'J8:13': 2,
                    'J8:15': 3,  'J8:16': 4,  'J8:18': 5,  'J8:19': 12,
                    'J8:21': 13, 'J8:22': 6,  'J8:23': 14, 'J8:24': 10,
                    'J8:26': 11, 'J8:27': 30, 'J8:28': 31, 'J8:29': 21,
                    'J8:31': 22, 'J8:32': 26, 'J8:33': 23, 'J8:35': 24,
                    'J8:36': 27, 'J8:37': 25, 'J8:38': 28, 'J8:40': 29,
                    'P1:3':  8,  'P1:5':  9,  'P1:7':  7,  'P1:8':  15,
                    'P1:10': 16, 'P1:11': 0,  'P1:12': 1,  'P1:13': 2,
                    'P1:15': 3,  'P1:16': 4,  'P1:18': 5,  'P1:19': 12,
                    'P1:21': 13, 'P1:22': 6,  'P1:23': 14, 'P1:24': 10,
                    'P1:26': 11, 'P1:27': 30, 'P1:28': 31, 'P1:29': 21,
                    'P1:31': 22, 'P1:32': 26, 'P1:33': 23, 'P1:35': 24,
                    'P1:36': 27, 'P1:37': 25, 'P1:38': 28, 'P1:40': 29,
                    'P5:3':  17, 'P5:4':  18, 'P5:5':  19, 'P5:6':  20,
                }
                names.add(f'WPI{wpi_map[phys_name]}')
            except KeyError:
                pass
        except KeyError:
            name = interfaces['']
            names.add(name)
        return PinInfo(
            number=number, name=name, names=frozenset(names), pull=pull,
            row=row, col=col, interfaces=frozenset(interfaces))

    @property
    def description(self):
        return f"BeagleY-AI {self.model} rev {self.pcb_revision}"


class TiFactory(Factory):
    """
    Extends :class:`~gpiozero.Factory`. Abstract base class representing
    hardware attached to a TI board. This forms the base of
    :class:`~gpiozero.pins.local.LocalTiFactory`.
    """
    def __init__(self):
        super().__init__()
        self._info = None
        self.pins = {}
        self.pin_class = None

    def close(self):
        for pin in self.pins.values():
            pin.close()
        self.pins.clear()

    def pin(self, name):
        for header, info in self.board_info.find_pin(name):
            try:
                pin = self.pins[info]
            except KeyError:
                pin = self.pin_class(self, info)
                self.pins[info] = pin
            return pin
        raise PinInvalidPin(f'{name} is not a valid pin name')

    def _get_revision(self):
        """
        This method must be overridden by descendents to return the TI board's
        revision code as an :class:`int`. The default is unimplemented.
        """
        raise NotImplementedError

    def _get_board_info(self):
        # TODO: BeagleY-AI does not have a revision number
        if self._info is None:
            self._info = TiBoardInfo.from_revision(self._get_revision())
        return self._info

class TiPin(Pin):
    """
    Extends :class:`~gpiozero.Pin`. Abstract base class representing a
    multi-function GPIO pin attached to a TI board. Descendents *must*
    override the following methods:

    * :meth:`_get_function`
    * :meth:`_set_function`
    * :meth:`_get_state`
    * :meth:`_call_when_changed`
    * :meth:`_enable_event_detect`
    * :meth:`_disable_event_detect`

    Descendents *may* additionally override the following methods, if
    applicable:

    * :meth:`close`
    * :meth:`output_with_state`
    * :meth:`input_with_pull`
    * :meth:`_set_state`
    * :meth:`_get_frequency`
    * :meth:`_set_frequency`
    * :meth:`_get_pull`
    * :meth:`_set_pull`
    * :meth:`_get_bounce`
    * :meth:`_set_bounce`
    * :meth:`_get_edges`
    * :meth:`_set_edges`
    """
    def __init__(self, factory, info):
        super().__init__()
        if 'gpio' not in info.interfaces:
            raise PinInvalidPin(f'{info} is not a GPIO pin')
        self._factory = factory
        self._info = info
        self._number = int(info.name[4:])
        self._when_changed_lock = RLock()
        self._when_changed = None

    @property
    def info(self):
        return self._info

    def __repr__(self):
        return self._info.name

    @property
    def factory(self):
        return self._factory

    def _call_when_changed(self, ticks, state):
        """
        Called to fire the :attr:`when_changed` event handler; override this
        in descendents if additional (currently redundant) parameters need
        to be passed.
        """
        method = self._when_changed()
        if method is None:
            self.when_changed = None
        else:
            method(ticks, state)

    def _get_when_changed(self):
        return None if self._when_changed is None else self._when_changed()

    def _set_when_changed(self, value):
        with self._when_changed_lock:
            if value is None:
                if self._when_changed is not None:
                    self._disable_event_detect()
                self._when_changed = None
            else:
                enabled = self._when_changed is not None
                # Have to take care, if value is either a closure or a bound
                # method, not to keep a strong reference to the containing
                # object
                if isinstance(value, MethodType):
                    self._when_changed = WeakMethod(value)
                else:
                    self._when_changed = ref(value)
                if not enabled:
                    self._enable_event_detect()

    def _enable_event_detect(self):
        """
        Enables event detection. This is called to activate event detection on
        pin :attr:`number`, watching for the specified :attr:`edges`. In
        response, :meth:`_call_when_changed` should be executed.
        """
        raise NotImplementedError

    def _disable_event_detect(self):
        """
        Disables event detection. This is called to deactivate event detection
        on pin :attr:`number`.
        """
        raise NotImplementedError
