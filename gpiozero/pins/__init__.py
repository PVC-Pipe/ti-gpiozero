# vim: set fileencoding=utf-8:
#
# GPIO Zero: a library for controlling the Raspberry Pi's GPIO pins
#
# Copyright (c) 2015-2021 Dave Jones <dave@waveform.org.uk>
# Copyright (c) 2018 Rick Ansell <rick@nbinvincible.org.uk>
# Copyright (c) 2018 Mike Kazantsev <mk.fraggod@gmail.com>
# Copyright (c) 2016 Andrew Scheller <github@loowis.durge.org>
#
# SPDX-License-Identifier: BSD-3-Clause

import warnings
from weakref import ref
from threading import Lock
from textwrap import dedent
from itertools import cycle
from operator import attrgetter
from collections import defaultdict, namedtuple

from .style import Style
from .data import V5, V3_3, V1_8, GND, NC
from ..devices import Device
from ..exc import (
    PinInvalidPin,
    PinInvalidFunction,
    PinSetInput,
    PinFixedPull,
    PinUnsupported,
    PinSPIUnsupported,
    PinPWMUnsupported,
    PinEdgeDetectUnsupported,
    PinMultiplePins,
    PinNoPins,
    SPIFixedClockMode,
    SPIFixedBitOrder,
    SPIFixedSelect,
    SPIFixedWordSize,
    SPIFixedRate,
    GPIOPinInUse,
)


class Factory:
    """
    Generates pins and SPI interfaces for devices. This is an abstract
    base class for pin factories. Descendents *must* override the following
    methods:

    * :meth:`ticks`
    * :meth:`ticks_diff`
    * :meth:`_get_board_info`

    Descendents *may* override the following methods, if applicable:

    * :meth:`close`
    * :meth:`reserve_pins`
    * :meth:`release_pins`
    * :meth:`release_all`
    * :meth:`pin`
    * :meth:`spi`
    """
    def __init__(self):
        self._reservations = defaultdict(list)
        self._res_lock = Lock()

    def reserve_pins(self, requester, *specs):
        """
        Called to indicate that the device reserves the right to use the
        specified pin *specs*. This should be done during device construction.
        If pins are reserved, you must ensure that the reservation is released
        by eventually called :meth:`release_pins`.
        """
        with self._res_lock:
            pins = (
                info
                for spec in specs
                for header, info in self.board_info.find_by_spec(spec)
            )
            for pin in pins:
                for reserver_ref in self._reservations[pin]:
                    reserver = reserver_ref()
                    if reserver is not None and requester._conflicts_with(reserver):
                        raise GPIOPinInUse(
                            'pin {pin.spec} is already in use by '
                            '{reserver!r}'.format(pin=pin, reserver=reserver))
                self._reservations[pin].append(ref(requester))

    def release_pins(self, reserver, *specs):
        """
        Releases the reservation of *reserver* against pin *specs*.  This is
        typically called during :meth:`~gpiozero.Device.close` to clean up
        reservations taken during construction. Releasing a reservation that is
        not currently held will be silently ignored (to permit clean-up after
        failed / partial construction).
        """
        pins = (
            info
            for spec in specs
            for header, info in self.board_info.find_by_spec(spec)
        )
        with self._res_lock:
            for pin in pins:
                self._reservations[pin] = [
                    ref for ref in self._reservations[pin]
                    if ref() not in (reserver, None) # may as well clean up dead refs
                    ]

    def release_all(self, reserver):
        """
        Releases all pin reservations taken out by *reserver*. See
        :meth:`release_pins` for further information).
        """
        # Yes, this would be more efficient if it simply regenerated the
        # reservations list without any references to reserver instead of
        # (in release_pins) looping over each pin individually. However, this
        # then causes a subtle bug in LocalPiFactory which does something
        # horribly naughty (with good reason) and makes its _reservations
        # dictionary equivalent to a class-level one.
        self.release_pins(reserver, *(
            pin.spec for pin in self._reservations))

    def close(self):
        """
        Closes the pin factory. This is expected to clean up all resources
        manipulated by the factory. It it typically called at script
        termination.
        """
        pass

    def pin(self, spec):
        """
        Creates an instance of a :class:`Pin` descendent representing the
        specified pin.

        .. warning::

            Descendents must ensure that pin instances representing the same
            hardware are identical; i.e. two separate invocations of
            :meth:`pin` for the same pin specification must return the same
            object.
        """
        raise PinUnsupported(  # pragma: no cover
            "Individual pins are not supported by this pin factory")

    def spi(self, **spi_args):
        """
        Returns an instance of an :class:`SPI` interface, for the specified SPI
        *port* and *device*, or for the specified pins (*clock_pin*,
        *mosi_pin*, *miso_pin*, and *select_pin*).  Only one of the schemes can
        be used; attempting to mix *port* and *device* with pin numbers will
        raise :exc:`SPIBadArgs`.
        """
        raise PinSPIUnsupported(  # pragma: no cover
            'SPI not supported by this pin factory')

    def ticks(self):
        """
        Return the current ticks, according to the factory. The reference point
        is undefined and thus the result of this method is only meaningful when
        compared to another value returned by this method.

        The format of the time is also arbitrary, as is whether the time wraps
        after a certain duration. Ticks should only be compared using the
        :meth:`ticks_diff` method.
        """
        raise NotImplementedError

    def ticks_diff(self, later, earlier):
        """
        Return the time in seconds between two :meth:`ticks` results. The
        arguments are specified in the same order as they would be in the
        formula *later* - *earlier* but the result is guaranteed to be in
        seconds, and to be positive even if the ticks "wrapped" between calls
        to :meth:`ticks`.
        """
        raise NotImplementedError

    def _get_board_info(self):
        raise NotImplementedError

    board_info = property(
        lambda self: self._get_board_info(),
        doc="""\
        Returns a :class:`BoardInfo` instance (or derivative) representing the
        board that instances generated by this factory will be attached to.
        """)

    def _get_pi_info(self):
        warnings.warn(
            DeprecationWarning(
                "Please use Factory.board_info instead of Factory.pi_info"))
        return self._get_board_info()

    pi_info = property(lambda self: self._get_pi_info())


class Pin:
    """
    Abstract base class representing a pin attached to some form of controller,
    be it GPIO, SPI, ADC, etc.

    Descendents should override property getters and setters to accurately
    represent the capabilities of pins. Descendents *must* override the
    following methods:

    * :meth:`_get_info`
    * :meth:`_get_function`
    * :meth:`_set_function`
    * :meth:`_get_state`

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
    * :meth:`_get_when_changed`
    * :meth:`_set_when_changed`
    """

    def __repr__(self):
        return "<Pin>"  # pragma: no cover

    def close(self):
        """
        Cleans up the resources allocated to the pin. After this method is
        called, this :class:`Pin` instance may no longer be used to query or
        control the pin's state.
        """
        pass

    def output_with_state(self, state):
        """
        Sets the pin's function to "output" and specifies an initial state
        for the pin. By default this is equivalent to performing::

            pin.function = 'output'
            pin.state = state

        However, descendents may override this in order to provide the smallest
        possible delay between configuring the pin for output and specifying an
        initial value (which can be important for avoiding "blips" in
        active-low configurations).
        """
        self.function = 'output'
        self.state = state

    def input_with_pull(self, pull):
        """
        Sets the pin's function to "input" and specifies an initial pull-up
        for the pin. By default this is equivalent to performing::

            pin.function = 'input'
            pin.pull = pull

        However, descendents may override this order to provide the smallest
        possible delay between configuring the pin for input and pulling the
        pin up/down (which can be important for avoiding "blips" in some
        configurations).
        """
        self.function = 'input'
        self.pull = pull

    def _get_info(self):
        raise NotImplementedError

    info = property(
        lambda self: self._get_info(),
        doc="""\
        Returns the :class:`PinInfo` associated with the pin. This can be used
        to determine physical properties of the pin, including its location on
        the header, fixed pulls, and the various specs that can be used to
        identify it.
        """)

    def _get_function(self):
        raise NotImplementedError

    def _set_function(self, value):
        raise NotImplementedError

    function = property(
        lambda self: self._get_function(),
        lambda self, value: self._set_function(value),
        doc="""\
        The function of the pin. This property is a string indicating the
        current function or purpose of the pin. Typically this is the string
        "input" or "output". However, in some circumstances it can be other
        strings indicating non-GPIO related functionality.

        With certain pin types (e.g. GPIO pins), this attribute can be changed
        to configure the function of a pin. If an invalid function is
        specified, for this attribute, :exc:`PinInvalidFunction` will be
        raised.
        """)

    def _get_state(self):
        raise NotImplementedError

    def _set_state(self, value):
        raise PinSetInput(  # pragma: no cover
            "Cannot set the state of pin {self!r}".format(self=self))

    state = property(
        lambda self: self._get_state(),
        lambda self, value: self._set_state(value),
        doc="""\
        The state of the pin. This is 0 for low, and 1 for high. As a low level
        view of the pin, no swapping is performed in the case of pull ups (see
        :attr:`pull` for more information):

        .. code-block:: text

            HIGH - - - - >       ,----------------------
                                 |
                                 |
            LOW  ----------------'

        Descendents which implement analog, or analog-like capabilities can
        return values between 0 and 1. For example, pins implementing PWM
        (where :attr:`frequency` is not :data:`None`) return a value between
        0.0 and 1.0 representing the current PWM duty cycle.

        If a pin is currently configured for input, and an attempt is made to
        set this attribute, :exc:`PinSetInput` will be raised. If an invalid
        value is specified for this attribute, :exc:`PinInvalidState` will be
        raised.
        """)

    def _get_pull(self):
        return 'floating'  # pragma: no cover

    def _set_pull(self, value):
        raise PinFixedPull(  # pragma: no cover
            "Cannot change pull-up on pin {self!r}".format(self=self))

    pull = property(
        lambda self: self._get_pull(),
        lambda self, value: self._set_pull(value),
        doc="""\
        The pull-up state of the pin represented as a string. This is typically
        one of the strings "up", "down", or "floating" but additional values
        may be supported by the underlying hardware.

        If the pin does not support changing pull-up state (for example because
        of a fixed pull-up resistor), attempts to set this property will raise
        :exc:`PinFixedPull`. If the specified value is not supported by the
        underlying hardware, :exc:`PinInvalidPull` is raised.
        """)

    def _get_frequency(self):
        return None  # pragma: no cover

    def _set_frequency(self, value):
        if value is not None:
            raise PinPWMUnsupported(  # pragma: no cover
                "PWM is not supported on pin {self!r}".format(self=self))

    frequency = property(
        lambda self: self._get_frequency(),
        lambda self, value: self._set_frequency(value),
        doc="""\
        The frequency (in Hz) for the pin's PWM implementation, or :data:`None`
        if PWM is not currently in use. This value always defaults to
        :data:`None` and may be changed with certain pin types to activate or
        deactivate PWM.

        If the pin does not support PWM, :exc:`PinPWMUnsupported` will be
        raised when attempting to set this to a value other than :data:`None`.
        """)

    def _get_bounce(self):
        return None  # pragma: no cover

    def _set_bounce(self, value):
        if value is not None:  # pragma: no cover
            raise PinEdgeDetectUnsupported(
                "Edge detection is not supported on pin {self!r}".format(
                    self=self))

    bounce = property(
        lambda self: self._get_bounce(),
        lambda self, value: self._set_bounce(value),
        doc="""\
        The amount of bounce detection (elimination) currently in use by edge
        detection, measured in seconds. If bounce detection is not currently in
        use, this is :data:`None`.

        For example, if :attr:`edges` is currently "rising", :attr:`bounce` is
        currently 5/1000 (5ms), then the waveform below will only fire
        :attr:`when_changed` on two occasions despite there being three rising
        edges:

        .. code-block:: text

            TIME 0...1...2...3...4...5...6...7...8...9...10..11..12 ms

            bounce elimination   |===================| |==============

            HIGH - - - - >       ,--. ,--------------. ,--.
                                 |  | |              | |  |
                                 |  | |              | |  |
            LOW  ----------------'  `-'              `-'  `-----------
                                 :                     :
                                 :                     :
                           when_changed          when_changed
                               fires                 fires

        If the pin does not support edge detection, attempts to set this
        property will raise :exc:`PinEdgeDetectUnsupported`. If the pin
        supports edge detection, the class must implement bounce detection,
        even if only in software.
        """)

    def _get_edges(self):
        return 'none'  # pragma: no cover

    def _set_edges(self, value):
        raise PinEdgeDetectUnsupported(  # pragma: no cover
            "Edge detection is not supported on pin {self!r}".format(self=self))

    edges = property(
        lambda self: self._get_edges(),
        lambda self, value: self._set_edges(value),
        doc="""\
        The edge that will trigger execution of the function or bound method
        assigned to :attr:`when_changed`. This can be one of the strings
        "both" (the default), "rising", "falling", or "none":

        .. code-block:: text

            HIGH - - - - >           ,--------------.
                                     |              |
                                     |              |
            LOW  --------------------'              `--------------
                                     :              :
                                     :              :
            Fires when_changed     "both"         "both"
            when edges is ...     "rising"       "falling"

        If the pin does not support edge detection, attempts to set this
        property will raise :exc:`PinEdgeDetectUnsupported`.
        """)

    def _get_when_changed(self):
        return None  # pragma: no cover

    def _set_when_changed(self, value):
        raise PinEdgeDetectUnsupported(  # pragma: no cover
            "Edge detection is not supported on pin {self!r}".format(self=self))

    when_changed = property(
        lambda self: self._get_when_changed(),
        lambda self, value: self._set_when_changed(value),
        doc="""\
        A function or bound method to be called when the pin's state changes
        (more specifically when the edge specified by :attr:`edges` is detected
        on the pin). The function or bound method must accept two parameters:
        the first will report the ticks (from :meth:`Factory.ticks`) when
        the pin's state changed, and the second will report the pin's current
        state.

        .. warning::

            Depending on hardware support, the state is *not guaranteed to be
            accurate*. For instance, many GPIO implementations will provide
            an interrupt indicating when a pin's state changed but not what it
            changed to. In this case the pin driver simply reads the pin's
            current state to supply this parameter, but the pin's state may
            have changed *since* the interrupt. Exercise appropriate caution
            when relying upon this parameter.

        If the pin does not support edge detection, attempts to set this
        property will raise :exc:`PinEdgeDetectUnsupported`.
        """)


class SPI(Device):
    """
    Abstract interface for `Serial Peripheral Interface`_ (SPI)
    implementations. Descendents *must* override the following methods:

    * :meth:`transfer`
    * :meth:`_get_clock_mode`

    Descendents *may* override the following methods, if applicable:

    * :meth:`read`
    * :meth:`write`
    * :meth:`_set_clock_mode`
    * :meth:`_get_lsb_first`
    * :meth:`_set_lsb_first`
    * :meth:`_get_select_high`
    * :meth:`_set_select_high`
    * :meth:`_get_bits_per_word`
    * :meth:`_set_bits_per_word`

    .. _Serial Peripheral Interface: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
    """

    def read(self, n):
        """
        Read *n* words of data from the SPI interface, returning them as a
        sequence of unsigned ints, each no larger than the configured
        :attr:`bits_per_word` of the interface.

        This method is typically used with read-only devices that feature
        half-duplex communication. See :meth:`transfer` for full duplex
        communication.
        """
        return self.transfer([0] * n)

    def write(self, data):
        """
        Write *data* to the SPI interface. *data* must be a sequence of
        unsigned integer words each of which will fit within the configured
        :attr:`bits_per_word` of the interface. The method returns the number
        of words written to the interface (which may be less than or equal to
        the length of *data*).

        This method is typically used with write-only devices that feature
        half-duplex communication. See :meth:`transfer` for full duplex
        communication.
        """
        return len(self.transfer(data))

    def transfer(self, data):
        """
        Write *data* to the SPI interface. *data* must be a sequence of
        unsigned integer words each of which will fit within the configured
        :attr:`bits_per_word` of the interface. The method returns the sequence
        of words read from the interface while writing occurred (full duplex
        communication).

        The length of the sequence returned dictates the number of words of
        *data* written to the interface. Each word in the returned sequence
        will be an unsigned integer no larger than the configured
        :attr:`bits_per_word` of the interface.
        """
        raise NotImplementedError

    @property
    def clock_polarity(self):
        """
        The polarity of the SPI clock pin. If this is :data:`False` (the
        default), the clock pin will idle low, and pulse high. Setting this to
        :data:`True` will cause the clock pin to idle high, and pulse low. On
        many data sheets this is documented as the CPOL value.

        The following diagram illustrates the waveform when
        :attr:`clock_polarity` is :data:`False` (the default), equivalent to
        CPOL 0:

        .. code-block:: text

                   on      on      on      on      on      on      on
                  ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
            CLK   |   |   |   |   |   |   |   |   |   |   |   |   |   |
                  |   |   |   |   |   |   |   |   |   |   |   |   |   |
            ------'   `---'   `---'   `---'   `---'   `---'   `---'   `------
            idle       off     off     off     off     off     off       idle

        The following diagram illustrates the waveform when
        :attr:`clock_polarity` is :data:`True`, equivalent to CPOL 1:

        .. code-block:: text

            idle       off     off     off     off     off     off       idle
            ------.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,------
                  |   |   |   |   |   |   |   |   |   |   |   |   |   |
            CLK   |   |   |   |   |   |   |   |   |   |   |   |   |   |
                  `---'   `---'   `---'   `---'   `---'   `---'   `---'
                   on      on      on      on      on      on      on
        """
        return bool(self.clock_mode & 2)

    @clock_polarity.setter
    def clock_polarity(self, value):
        self.clock_mode = self.clock_mode & (~2) | (bool(value) << 1)

    @property
    def clock_phase(self):
        """
        The phase of the SPI clock pin. If this is :data:`False` (the default),
        data will be read from the MISO pin when the clock pin activates.
        Setting this to :data:`True` will cause data to be read from the MISO
        pin when the clock pin deactivates. On many data sheets this is
        documented as the CPHA value. Whether the clock edge is rising or
        falling when the clock is considered activated is controlled by the
        :attr:`clock_polarity` attribute (corresponding to CPOL).

        The following diagram indicates when data is read when
        :attr:`clock_polarity` is :data:`False`, and :attr:`clock_phase` is
        :data:`False` (the default), equivalent to CPHA 0:

        .. code-block:: text

                ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `---'   `---'   `---'   `-------
                :       :       :       :       :       :       :
            MISO---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
              /     \\ /     \\ /     \\ /     \\ /     \\ /     \\ /     \\
            -{  Bit  X  Bit  X  Bit  X  Bit  X  Bit  X  Bit  X  Bit  }------
              \\     / \\     / \\     / \\     / \\     / \\     / \\     /
               `---'   `---'   `---'   `---'   `---'   `---'   `---'

        The following diagram indicates when data is read when
        :attr:`clock_polarity` is :data:`False`, but :attr:`clock_phase` is
        :data:`True`, equivalent to CPHA 1:

        .. code-block:: text

                ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `---'   `---'   `---'   `-------
                    :       :       :       :       :       :       :
            MISO   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
                  /     \\ /     \\ /     \\ /     \\ /     \\ /     \\ /     \\
            -----{  Bit  X  Bit  X  Bit  X  Bit  X  Bit  X  Bit  X  Bit  }--
                  \\     / \\     / \\     / \\     / \\     / \\     / \\     /
                   `---'   `---'   `---'   `---'   `---'   `---'   `---'
        """
        return bool(self.clock_mode & 1)

    @clock_phase.setter
    def clock_phase(self, value):
        self.clock_mode = self.clock_mode & (~1) | bool(value)

    def _get_clock_mode(self):
        raise NotImplementedError  # pragma: no cover

    def _set_clock_mode(self, value):
        raise SPIFixedClockMode(  # pragma: no cover
            "clock_mode cannot be changed on {self!r}".format(self=self))

    clock_mode = property(
        lambda self: self._get_clock_mode(),
        lambda self, value: self._set_clock_mode(value),
        doc="""\
        Presents a value representing the :attr:`clock_polarity` and
        :attr:`clock_phase` attributes combined according to the following
        table:

        +------+-----------------+--------------+
        | mode | polarity (CPOL) | phase (CPHA) |
        +======+=================+==============+
        | 0    | False           | False        |
        +------+-----------------+--------------+
        | 1    | False           | True         |
        +------+-----------------+--------------+
        | 2    | True            | False        |
        +------+-----------------+--------------+
        | 3    | True            | True         |
        +------+-----------------+--------------+

        Adjusting this value adjusts both the :attr:`clock_polarity` and
        :attr:`clock_phase` attributes simultaneously.
        """)

    def _get_lsb_first(self):
        return False  # pragma: no cover

    def _set_lsb_first(self, value):
        raise SPIFixedBitOrder(  # pragma: no cover
            "lsb_first cannot be changed on {self!r}".format(self=self))

    lsb_first = property(
        lambda self: self._get_lsb_first(),
        lambda self, value: self._set_lsb_first(value),
        doc="""\
        Controls whether words are read and written LSB in (Least Significant
        Bit first) order. The default is :data:`False` indicating that words
        are read and written in MSB (Most Significant Bit first) order.
        Effectively, this controls the `Bit endianness`_ of the connection.

        The following diagram shows the a word containing the number 5 (binary
        0101) transmitted on MISO with :attr:`bits_per_word` set to 4, and
        :attr:`clock_mode` set to 0, when :attr:`lsb_first` is :data:`False`
        (the default):

        .. code-block:: text

                ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `-----
                :     ,-------. :     ,-------.
            MISO:     | :     | :     | :     |
                :     | :     | :     | :     |
            ----------' :     `-------' :     `----
                :       :       :       :
               MSB                     LSB

        And now with :attr:`lsb_first` set to :data:`True` (and all other
        parameters the same):

        .. code-block:: text

                ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `-----
              ,-------. :     ,-------. :
            MISO:     | :     | :     | :
              | :     | :     | :     | :
            --' :     `-------' :     `-----------
                :       :       :       :
               LSB                     MSB

        .. _Bit endianness: https://en.wikipedia.org/wiki/Endianness#Bit_endianness
        """)

    def _get_select_high(self):
        return False  # pragma: no cover

    def _set_select_high(self, value):
        raise SPIFixedSelect(  # pragma: no cover
            "select_high cannot be changed on {self!r}".format(self=self))

    select_high = property(
        lambda self: self._get_select_high(),
        lambda self, value: self._set_select_high(value),
        doc="""\
        If :data:`False` (the default), the chip select line is considered
        active when it is pulled low. When set to :data:`True`, the chip select
        line is considered active when it is driven high.

        The following diagram shows the waveform of the chip select line, and
        the clock when :attr:`clock_polarity` is :data:`False`, and
        :attr:`select_high` is :data:`False` (the default):

        .. code-block:: text

            ---.                                                     ,------
            __ |                                                     |
            CS |      chip is selected, and will react to clock      |  idle
               `-----------------------------------------------------'

                ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `---'   `---'   `---'   `-------

        And when :attr:`select_high` is :data:`True`:

        .. code-block:: text

               ,-----------------------------------------------------.
            CS |      chip is selected, and will react to clock      |  idle
               |                                                     |
            ---'                                                     `------

                ,---.   ,---.   ,---.   ,---.   ,---.   ,---.   ,---.
            CLK |   |   |   |   |   |   |   |   |   |   |   |   |   |
                |   |   |   |   |   |   |   |   |   |   |   |   |   |
            ----'   `---'   `---'   `---'   `---'   `---'   `---'   `-------
        """)

    def _get_bits_per_word(self):
        return 8  # pragma: no cover

    def _set_bits_per_word(self, value):
        raise SPIFixedWordSize(  # pragma: no cover
            "bits_per_word cannot be changed on {self!r}".format(self=self))

    bits_per_word = property(
        lambda self: self._get_bits_per_word(),
        lambda self, value: self._set_bits_per_word(value),
        doc="""\
        Controls the number of bits that make up a word, and thus where the
        word boundaries appear in the data stream, and the maximum value of a
        word. Defaults to 8 meaning that words are effectively bytes.

        Several implementations do not support non-byte-sized words.
        """)

    def _get_rate(self):
        return 100000  # pragma: no cover

    def _set_rate(self, value):
        raise SPIFixedRate(  # pragma: no cover
            "rate cannot be changed on {self!r}".format(self=self))

    rate = property(
        lambda self: self._get_rate(),
        lambda self, value: self._set_rate(value),
        doc="""\
        Controls the speed of the SPI interface in Hz (or baud).

        Note that most software SPI implementations ignore this property, and
        will raise :exc:`SPIFixedRate` if an attempt is made to set it, as they
        have no rate control (they simply bit-bang as fast as possible because
        typically this isn't very fast anyway, and introducing measures to
        limit the rate would simply slow them down to the point of being
        useless).
        """)


class PinInfo(namedtuple('PinInfo', (
    'number',
    'spec',
    'specs',
    'pull',
    'row',
    'col',
    'is_gpio',
    ))):
    """
    This class is a :func:`~collections.namedtuple` derivative used to
    represent information about a pin present on a GPIO header. The following
    attributes are defined:

    .. attribute:: number

        An integer containing the physical pin number on the header (starting
        from 1 in accordance with convention).

    .. attribute:: spec

        A string describing the function of the pin. Some common examples
        include "GND" (for pins connecting to ground), "3V3" (for pins which
        output 3.3 volts), "GPIO9" (for GPIO9 in the board's numbering scheme),
        etc.

        When :attr:`is_gpio` is :data:`True`, the spec can be used with
        :meth:`Factory.pin` to construct a :class:`Pin` instance representing
        this pin.

    .. attribute:: specs

        A set of all the values (note: not necessarily just strings) that can
        be used to describe this pin (and when :attr:`is_gpio` is :data:`True`,
        the set of all values that can be used to construct a :class:`Pin`
        instance).

        The :attr:`spec` is the "canonical" spec for this pin, and will be one
        of the values in this set.

    .. attribute:: pull

        A string indicating the fixed pull of the pin, if any. This is blank
        if the pin has no fixed pull, but may be "up" in the case of pins
        typically used for I2C such as GPIO2 and GPIO3 on a Raspberry Pi.

    .. attribute:: row

        An integer indicating on which row the pin is physically located in
        the header (1-based)

    .. attribute:: col

        An integer indicating in which column the pin is physically located
        in the header (1-based)

    .. attribute:: is_gpio

        A bool which is :data:`True` if this is a pin which can be controlled
        by calling :meth:`Factory.pin` with its :attr:`spec`.

    .. autoattribute:: function

    .. autoattribute:: pull_up
    """
    __slots__ = () # workaround python issue #24931

    @property
    def function(self):
        """
        A deprecated alias for :attr:`spec`.
        """
        warnings.warn(
            DeprecationWarning(
                "PinInfo.function is deprecated; please use PinInfo.spec"))
        return self.spec

    @property
    def pull_up(self):
        """
        Deprecated variant of :attr:`pull`.
        """
        warnings.warn(
            DeprecationWarning(
                "PinInfo.pull_up is deprecated; please use PinInfo.pull"))
        return self.pull == 'up'


class HeaderInfo(namedtuple('HeaderInfo', (
    'name',
    'rows',
    'columns',
    'pins',
    ))):
    """
    This class is a :func:`~collections.namedtuple` derivative used to
    represent information about a pin header on a board. The object can be used
    in a format string with various custom specifications::

        from gpiozero.pins.native import NativeFactory

        factory = NativeFactory()
        print('{0}'.format(factory.board_info.headers['J8']))
        print('{0:full}'.format(factory.board_info.headers['J8']))
        print('{0:col2}'.format(factory.board_info.headers['P1']))
        print('{0:row1}'.format(factory.board_info.headers['P1']))

    "color" and "mono" can be prefixed to format specifications to force the
    use of `ANSI color codes`_. If neither is specified, ANSI codes will only
    be used if stdout is detected to be a tty::

        # force use of ANSI codes
        print('{0:color row2}'.format(factory.board_info.headers['J8']))
        # force plain ASCII
        print('{0:mono row2}'.format(factory.board_info.headers['P1']))

    The following attributes are defined:

    .. automethod:: pprint

    .. attribute:: name

        The name of the header, typically as it appears silk-screened on the
        board (e.g. "P1" or "J8").

    .. attribute:: rows

        The number of rows on the header.

    .. attribute:: columns

        The number of columns on the header.

    .. attribute:: pins

        A dictionary mapping physical pin numbers to :class:`PinInfo` tuples.

    .. _ANSI color codes: https://en.wikipedia.org/wiki/ANSI_escape_code
    """
    __slots__ = () # workaround python issue #24931

    def _pin_style(self, pin, style):
        if pin.is_gpio:
            return style('bold green')
        elif pin.spec == V5:
            return style('bold red')
        elif pin.spec in (V3_3, V1_8):
            return style('bold cyan')
        elif pin.spec in (GND, NC):
            return style('bold black')
        else:
            return style('yellow')

    def _format_full(self, style):
        Cell = namedtuple('Cell', ('content', 'align', 'style'))

        lines = []
        for row in range(self.rows):
            line = []
            for col in range(self.columns):
                pin = (row * self.columns) + col + 1
                try:
                    pin = self.pins[pin]
                    cells = [
                        Cell(pin.spec, '><'[col % 2], self._pin_style(pin, style)),
                        Cell('({pin.number})'.format(pin=pin), '><'[col % 2], ''),
                        ]
                    if col % 2:
                        cells = reversed(cells)
                    line.extend(cells)
                except KeyError:
                    line.append(Cell('', '<', ''))
            lines.append(line)
        cols = list(zip(*lines))
        col_lens = [max(len(cell.content) for cell in col) for col in cols]
        lines = [
            ' '.join(
                '{cell.style}{cell.content:{cell.align}{width}s}{style:reset}'.format(
                    cell=cell, width=width, style=style)
                for cell, width, align in zip(line, col_lens, cycle('><')))
            for line in lines
            ]
        return '\n'.join(lines)

    def _format_pin(self, pin, style):
        return ''.join((
            style('on black'),
            (
                ' ' if pin is None else
                self._pin_style(pin, style) +
                ('1' if pin.number == 1 else 'o')
                ),
            style('reset')
            ))

    def _format_row(self, row, style):
        if row > self.rows:
            raise ValueError(
                'invalid row {row} for header {self.name}'.format(
                    row=row, self=self))
        start_pin = (row - 1) * self.columns + 1
        return ''.join(
            self._format_pin(pin, style)
            for n in range(start_pin, start_pin + self.columns)
            for pin in (self.pins.get(n),)
            )

    def _format_col(self, col, style):
        if col > self.columns:
            raise ValueError(
                'invalid col {col} for header {self.name}'.format(
                    col=col, self=self))
        return ''.join(
            self._format_pin(pin, style)
            for n in range(col, self.rows * self.columns + 1, self.columns)
            for pin in (self.pins.get(n),)
            )

    def __format__(self, format_spec):
        style, content = Style.from_style_content(format_spec)
        if content == 'full':
            return self._format_full(style)
        elif content.startswith('row') and content[3:].isdigit():
            return self._format_row(int(content[3:]), style)
        elif content.startswith('col') and content[3:].isdigit():
            return self._format_col(int(content[3:]), style)
        else:
            raise ValueError('Invalid format specifier')

    def pprint(self, color=None):
        """
        Pretty-print a diagram of the header pins.

        If *color* is :data:`None` (the default, the diagram will include ANSI
        color codes if stdout is a color-capable terminal). Otherwise *color*
        can be set to :data:`True` or :data:`False` to force color or
        monochrome output.
        """
        print('{0:{style} full}'.format(self, style=Style(color)))


class BoardInfo(namedtuple('BoardInfo', (
    'revision',
    'model',
    'pcb_revision',
    'released',
    'soc',
    'manufacturer',
    'memory',
    'storage',
    'usb',
    'usb3',
    'ethernet',
    'eth_speed',
    'wifi',
    'bluetooth',
    'csi',
    'dsi',
    'headers',
    'board',
    ))):
    """
    This class is a :func:`~collections.namedtuple` derivative used to
    represent information about a particular board. While it is a tuple, it is
    strongly recommended that you use the following named attributes to access
    the data contained within. The object can be used in format strings with
    various custom format specifications::

        from gpiozero.pins.native import NativeFactory

        factory = NativeFactory()
        print('{0}'.format(factory.board_info))
        print('{0:full}'.format(factory.board_info))
        print('{0:board}'.format(factory.board_info))
        print('{0:specs}'.format(factory.board_info))
        print('{0:headers}'.format(factory.board_info))

    "color" and "mono" can be prefixed to format specifications to force the
    use of `ANSI color codes`_. If neither is specified, ANSI codes will only
    be used if stdout is detected to be a tty::

        # force use of ANSI codes
        print('{0:color board}'.format(factory.board_info))
        # force plain ASCII
        print('{0:mono board}'.format(factory.board_info))

    .. _ANSI color codes: https://en.wikipedia.org/wiki/ANSI_escape_code

    .. automethod:: physical_pin

    .. automethod:: physical_pins

    .. automethod:: pprint

    .. automethod:: pulled_up

    .. automethod:: to_gpio

    .. attribute:: revision

        A string indicating the revision of the board. This is unique to each
        revision and can be considered the "key" from which all other
        attributes are derived. However, in itself the string is fairly
        meaningless.

    .. attribute:: model

        A string containing the model of the board (for example, "B", "B+",
        "A+", "2B", "CM", "Zero", etc.)

    .. attribute:: pcb_revision

        A string containing the PCB revision number which is silk-screened onto
        the board.

    .. attribute:: released

        A string containing an approximate release date for this board
        (formatted as yyyyQq, e.g. 2012Q1 means the first quarter of 2012).

    .. attribute:: soc

        A string indicating the SoC (`system on a chip`_) that powers this
        board.

    .. attribute:: manufacturer

        A string indicating the name of the manufacturer (e.g. "Sony").

    .. attribute:: memory

        An integer indicating the amount of memory (in Mb) connected to the
        SoC.

        .. note::

            This can differ substantially from the amount of RAM available to
            the operating system as the GPU's memory is typically shared with
            the CPU.

    .. attribute:: storage

        A string indicating the typical bootable storage used with this board,
        e.g. "SD", "MicroSD", or "eMMC".

    .. attribute:: usb

        An integer indicating how many USB ports are physically present on
        this board, of any type.

        .. note::

            This does *not* include any (typically micro-USB) port used to
            power the board.

    .. attribute:: usb3

        An integer indicating how many of the USB ports are USB3 ports on this
        board.

    .. attribute:: ethernet

        An integer indicating how many Ethernet ports are physically present
        on this board.

    .. attribute:: eth_speed

        An integer indicating the maximum speed (in Mbps) of the Ethernet ports
        (if any). If no Ethernet ports are present, this is 0.

    .. attribute:: wifi

        A bool indicating whether this board has wifi built-in.

    .. attribute:: bluetooth

        A bool indicating whether this board has bluetooth built-in.

    .. attribute:: csi

        An integer indicating the number of CSI (camera) ports available on
        this board.

    .. attribute:: dsi

        An integer indicating the number of DSI (display) ports available on
        this board.

    .. attribute:: headers

        A dictionary which maps header labels to :class:`HeaderInfo` tuples.
        For example, to obtain information about header P1 you would query
        ``headers['P1']``. To obtain information about pin 12 on header J8 you
        would query ``headers['J8'].pins[12]``.

        A rendered version of this data can be obtained by using the
        :class:`BoardInfo` object in a format string::

            from gpiozero.pins.native import NativeFactory

            factory = NativeFactory()
            print('{0:headers}'.format(factory.board_info))

    .. attribute:: board

        An ASCII art rendition of the board, primarily intended for console
        pretty-print usage. A more usefully rendered version of this data can
        be obtained by using the :class:`BoardInfo` object in a format string.
        For example::

            from gpiozero.pins.native import NativeFactory

            factory = NativeFactory()
            print('{0:board}'.format(factory.board_info))

    .. autoattribute:: description

    .. _system on a chip: https://en.wikipedia.org/wiki/System_on_a_chip
    """
    __slots__ = () # workaround python issue #24931

    def find_by_spec(self, spec):
        """
        A generator function which, given a pin *spec*, yields tuples of
        :class:`HeaderInfo` and :class:`PinInfo` instances for which *spec*
        appears in :attr:`PinInfo.specs`.
        """
        for header in self.headers.values():
            for pin in header.pins.values():
                if spec in pin.specs:
                    yield header, pin

    def physical_pins(self, function):
        """
        Return the physical pins supporting the specified *function* as tuples
        of ``(header, pin_number)`` where *header* is a string specifying the
        header containing the *pin_number*. Note that the return value is a
        :class:`set` which is not indexable. Use :func:`physical_pin` if you
        are expecting a single return value.

        :param str function:
            The pin function you wish to search for. Usually this is something
            like "GPIO9" for GPIO pin 9, or "GND" for all the pins connecting
            to electrical ground.
        """
        warnings.warn(
            DeprecationWarning(
                "PiBoardInfo.physical_pins is deprecated; please use "
                "BoardInfo.find_by_spec instead"))
        return {
            (header.name, pin.number)
            for header, pin in self.find_by_spec(function)
        }

    def physical_pin(self, function):
        """
        Return the physical pin supporting the specified *function* as a tuple
        of ``(header, pin_number)`` where *header* is a string specifying the
        header containing the *pin_number*. If no pins support the desired
        *function*, this function raises :exc:`PinNoPins`. If multiple pins
        support the desired *function*, :exc:`PinMultiplePins` will be raised
        (use :func:`physical_pins` if you expect multiple pins in the result,
        such as for electrical ground).

        :param str function:
            The pin function you wish to search for. Usually this is something
            like "GPIO9" for GPIO pin 9.
        """
        warnings.warn(
            DeprecationWarning(
                "PiBoardInfo.physical_pin is deprecated; please use "
                "BoardInfo.find_by_spec instead"))
        result = self.physical_pins(function)
        if len(result) > 1:
            raise PinMultiplePins(
                'multiple pins can be used for {function}'.format(
                    function=function))
        elif result:
            return result.pop()
        else:
            raise PinNoPins('no pins can be used for {function}'.format(
                function=function))

    def pulled_up(self, function):
        """
        Returns a bool indicating whether a physical pull-up is attached to
        the pin supporting the specified *function*. Either :exc:`PinNoPins`
        or :exc:`PinMultiplePins` may be raised if the function is not
        associated with a single pin.

        :param str function:
            The pin function you wish to determine pull-up for. Usually this is
            something like "GPIO9" for GPIO pin 9.
        """
        warnings.warn(
            DeprecationWarning(
                "PiBoardInfo.pulled_up is deprecated; please use "
                "BoardInfo.find_by_spec and PinInfo.pull instead"))
        for header, pin in self.find_by_spec(function):
            return pin.pull == 'up'
        return False

    def to_gpio(self, spec):
        """
        Parses a pin *spec*, returning the equivalent GPIO port number or
        raising a :exc:`ValueError` exception if the spec does not represent a
        GPIO port.

        The *spec* may be given in any of the following forms:

        * An integer, which will be accepted as a GPIO number
        * 'GPIOn' where n is the GPIO number
        * 'h:n' where h is the header name and n is the physical pin number
        """
        warnings.warn(
            DeprecationWarning(
                "PiBoardInfo.to_gpio is deprecated; please use "
                "BoardInfo.find_by_spec and PinInfo.spec instead"))
        for header, pin in self.find_by_spec(spec):
            if pin.is_gpio:
                # XXX This assumes GPIO specs are always of the form "GPIOn"
                # which is *probably* reasonable but should we enforce this
                # somewhere?
                return int(pin.spec[4:])
            else:
                raise PinInvalidPin('{spec} is not a GPIO pin'.format(
                    spec=spec))
        raise PinInvalidPin('{spec} is not a valid pin spec'.format(
            spec=spec))

    def __repr__(self):
        return '{cls}({fields})'.format(
            cls=self.__class__.__name__,
            fields=', '.join(
                (
                    '{name}=...' if name in ('headers', 'board') else
                    '{name}={value!r}').format(name=name, value=value)
                for name, value in zip(self._fields, self)
                )
            )

    def __format__(self, format_spec):
        style, content = Style.from_style_content(format_spec)
        if content == 'full':
            return dedent("""\
                {self:{style} board}

                {self:{style} specs}

                {self:{style} headers}"""
                ).format(self=self, style=style)
        elif content == 'board':
            kw = self._asdict()
            kw.update({
                name: header
                for name, header in self.headers.items()
                })
            return self.board.format(style=style, **kw)
        elif content == 'specs':
            if self.memory < 1024:
                memory = '{self.memory}MB'.format(self=self)
            else:
                memory = '{mem}GB'.format(mem=int(self.memory / 1024))
            return dedent("""\
                {style:bold}Description        {style:reset}: {self.description}
                {style:bold}Revision           {style:reset}: {self.revision}
                {style:bold}SoC                {style:reset}: {self.soc}
                {style:bold}RAM                {style:reset}: {memory}
                {style:bold}Storage            {style:reset}: {self.storage}
                {style:bold}USB ports          {style:reset}: {self.usb} (of which {self.usb3} USB3)
                {style:bold}Ethernet ports     {style:reset}: {self.ethernet} ({self.eth_speed}Mbps max. speed)
                {style:bold}Wi-fi              {style:reset}: {self.wifi}
                {style:bold}Bluetooth          {style:reset}: {self.bluetooth}
                {style:bold}Camera ports (CSI) {style:reset}: {self.csi}
                {style:bold}Display ports (DSI){style:reset}: {self.dsi}"""
                ).format(self=self, style=style, memory=memory)
        elif content == 'headers':
            return '\n\n'.join(
                dedent("""\
                {style:bold}{header.name}{style:reset}:
                {header:{style} full}"""
                ).format(header=header, style=style)
                for header in sorted(self.headers.values(), key=attrgetter('name'))
                )
        else:
            raise ValueError('Invalid format specifier')

    def pprint(self, color=None):
        """
        Pretty-print a representation of the board along with header diagrams.

        If *color* is :data:`None` (the default), the diagram will include ANSI
        color codes if stdout is a color-capable terminal. Otherwise *color*
        can be set to :data:`True` or :data:`False` to force color or monochrome
        output.
        """
        print('{0:{style} full}'.format(self, style=Style(color)))

    @property
    def description(self):
        """
        A string containing a textual description of the board typically
        containing the :attr:`model`, for example "Raspberry Pi 3B"
        """
        return '{self.model} rev {self.pcb_revision}'.format(self=self)
