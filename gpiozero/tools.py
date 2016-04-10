# vim: set fileencoding=utf-8:

from __future__ import (
    unicode_literals,
    print_function,
    absolute_import,
    division,
)
str = type('')


from random import random
from time import sleep
try:
    from itertools import izip as zip
except ImportError:
    pass
from itertools import cycle
from math import sin, cos, radians
try:
    from statistics import mean
except ImportError:
    from .compat import mean


def negated(values):
    """
    Returns the negation of the supplied values (``True`` becomes ``False``,
    and ``False`` becomes ``True``). For example::

        from gpiozero import Button, LED
        from gpiozero.tools import negated
        from signal import pause

        led = LED(4)
        btn = Button(17)
        led.source = negated(btn.values)
        pause()
    """
    for v in values:
        yield not v


def inverted(values):
    """
    Returns the inversion of the supplied values (1 becomes 0, 0 becomes 1,
    0.1 becomes 0.9, etc.). For example::

        from gpiozero import MCP3008, PWMLED
        from gpiozero.tools import inverted
        from signal import pause

        led = PWMLED(4)
        pot = MCP3008(channel=0)
        led.source = inverted(pot.values)
        pause()
    """
    for v in values:
        yield 1 - v


def scaled(values, output_min, output_max, input_min=0, input_max=1):
    """
    Returns *values* scaled from *output_min* to *output_max*, assuming that
    all items in *values* lie between *input_min* and *input_max* (which
    default to 0 and 1 respectively). For example, to control the direction of
    a motor (which is represented as a value between -1 and 1) using a
    potentiometer (which typically provides values between 0 and 1)::

        from gpiozero import Motor, MCP3008
        from gpiozero.tools import scaled
        from signal import pause

        motor = Motor(20, 21)
        pot = MCP3008(channel=0)
        motor.source = scaled(pot.values, -1, 1)
        pause()

    .. warning::

        If *values* contains elements that lie outside *input_min* to
        *input_max* (inclusive) then the function will not produce values that
        lie within *output_min* to *output_max* (inclusive).
    """
    input_size = input_max - input_min
    output_size = output_max - output_min
    for v in values:
        yield (((v - input_min) / input_size) * output_size) + output_min


def clamped(values, output_min=0, output_max=1):
    """
    Returns *values* clamped from *output_min* to *output_max*, i.e. any items
    less than *output_min* will be returned as *output_min* and any items
    larger than *output_max* will be returned as *output_max* (these default to
    0 and 1 respectively). For example::

        from gpiozero import PWMLED, MCP3008
        from gpiozero.tools import clamped
        from signal import pause

        led = PWMLED(4)
        pot = MCP3008(channel=0)
        led.source = clamped(pot.values, 0.5, 1.0)
        pause()
    """
    for v in values:
        yield min(max(v, output_min), output_max)


def absoluted(values):
    """
    Returns *values* with all negative elements negated (so that they're
    positive). For example::

        from gpiozero import PWMLED, Motor, MCP3008
        from gpiozero.tools import absoluted, scaled
        from signal import pause

        led = PWMLED(4)
        motor = Motor(22, 27)
        pot = MCP3008(channel=0)
        motor.source = scaled(pot.values, -1, 1)
        led.source = absoluted(motor.values)
        pause()
    """
    for v in values:
        yield abs(v)


def quantized(values, steps, output_min=0, output_max=1):
    """
    Returns *values* quantized to *steps* increments. All items in *values* are
    assumed to be between *output_min* and *output_max* (use :func:`scaled` to
    ensure this if necessary).

    For example, to quantize values between 0 and 1 to 5 "steps" (0.0, 0.25,
    0.5, 0.75, 1.0)::

        from gpiozero import PWMLED, MCP3008
        from gpiozero.tools import quantized
        from signal import pause

        led = PWMLED(4)
        pot = MCP3008(channel=0)
        led.source = quantized(pot.values, 4)
        pause()
    """
    output_size = output_max - output_min
    for v in scaled(values, 0, 1, output_min, output_max):
        yield ((int(v * steps) / steps) * output_size) + output_min


def all_values(*values):
    """
    Returns the `logical conjunction`_ of all supplied values (the result is
    only ``True`` if and only if all input values are simultaneously ``True``).
    One or more *values* can be specified. For example, to light an
    :class:`LED` only when *both* buttons are pressed::

        from gpiozero import LED, Button
        from gpiozero.tools import all_values
        from signal import pause

        led = LED(4)
        btn1 = Button(20)
        btn2 = Button(21)
        led.source = all_values(btn1.values, btn2.values)
        pause()

    .. _logical conjunction: https://en.wikipedia.org/wiki/Logical_conjunction
    """
    for v in zip(*values):
        yield all(v)


def any_values(*values):
    """
    Returns the `logical disjunction`_ of all supplied values (the result is
    ``True`` if any of the input values are currently ``True``). One or more
    *values* can be specified. For example, to light an :class:`LED` when
    *any* button is pressed::

        from gpiozero import LED, Button
        from gpiozero.tools import any_values
        from signal import pause

        led = LED(4)
        btn1 = Button(20)
        btn2 = Button(21)
        led.source = any_values(btn1.values, btn2.values)
        pause()

    .. _logical disjunction: https://en.wikipedia.org/wiki/Logical_disjunction
    """
    for v in zip(*values):
        yield any(v)


def averaged(*values):
    """
    Returns the mean of all supplied values. One or more *values* can be
    specified. For example, to light a :class:`PWMLED` as the average of
    several potentiometers connected to an :class:`MCP3008` ADC::

        from gpiozero import MCP3008, PWMLED
        from gpiozero.tools import averaged
        from signal import pause

        pot1 = MCP3008(channel=0)
        pot2 = MCP3008(channel=1)
        pot3 = MCP3008(channel=2)
        led = PWMLED(4)
        led.source = averaged(pot1.values, pot2.values, pot3.values)
        pause()
    """
    for v in zip(*values):
        yield mean(v)


def queued(values, qsize):
    """
    Queues up readings from *values* (the number of readings queued is
    determined by *qsize*) and begins yielding values only when the queue is
    full. For example, to "cascade" values along a sequence of LEDs::

        from gpiozero import LEDBoard, Button
        from gpiozero.tools import queued
        from signal import pause

        leds = LEDBoard(5, 6, 13, 19, 26)
        btn = Button(17)
        for i in range(4):
            leds[i].source = queued(leds[i + 1].values, 5)
            leds[i].source_delay = 0.01
        leds[4].source = btn.values
        pause()
    """
    q = []
    it = iter(values)
    for i in range(qsize):
        q.append(next(it))
    for i in cycle(range(qsize)):
        yield q[i]
        try:
            q[i] = next(it)
        except StopIteration:
            break


def pre_delayed(values, delay):
    """
    Waits for *delay* seconds before returning each item from *values*.
    """
    for v in values:
        sleep(delay)
        yield v


def post_delayed(values, delay):
    """
    Waits for *delay* seconds after returning each item from *values*.
    """
    for v in values:
        yield v
        sleep(delay)


def random_values():
    """
    Provides an infinite source of random values between 0 and 1. For example,
    to produce a "flickering candle" effect with an LED::

        from gpiozero import PWMLED
        from gpiozero.tools import random_values
        from signal import pause

        led = PWMLED(4)
        led.source = random_values()
        pause()

    If you require a wider range than 0 to 1, see :func:`scaled`.
    """
    while True:
        yield random()


def sin_values():
    """
    Provides an infinite source of values representing a sine wave (from -1 to
    +1), calculated as the result of applying sign to a simple degrees counter
    that increments by one for each requested value. For example, to produce a
    "siren" effect with a couple of LEDs::

        from gpiozero import PWMLED
        from gpiozero.tools import sin_values, scaled, inverted
        from signal import pause

        red = PWMLED(2)
        blue = PWMLED(3)
        red.source_delay = 0.01
        blue.source_delay = 0.01
        red.source = scaled(sin_values(), 0, 1, -1, 1)
        blue.source = inverted(red.values)
        pause()

    If you require a wider range than 0 to 1, see :func:`scaled`.
    """
    for d in cycle(range(360)):
        yield sin(radians(d))


def cos_values():
    """
    Provides an infinite source of values representing a cosine wave (from -1
    to +1), calculated as the result of applying sign to a simple degrees
    counter that increments by one for each requested value. For example, to
    produce a "siren" effect with a couple of LEDs::

        from gpiozero import PWMLED
        from gpiozero.tools import cos_values, scaled, inverted
        from signal import pause

        red = PWMLED(2)
        blue = PWMLED(3)
        red.source_delay = 0.01
        blue.source_delay = 0.01
        red.source = scaled(cos_values(), 0, 1, -1, 1)
        blue.source = inverted(red.values)
        pause()

    If you require a wider range than 0 to 1, see :func:`scaled`.
    """
    for d in cycle(range(360)):
        yield cos(radians(d))

