# -*- coding: cp1252 -*-
"""
MQ-7 CO sensor
- Heater 0V, 1.4V and 5V supplied from three PWM pins.
- Bias oo, 36k, 18k, 12k and 9k ohm from 5V supplied from internal pull-up resistances.
- Bias voltage drop measured by substracting pull-up voltage from 5V.
- Sensor voltage drop measured differentially in a 4-point measurement.
- timeH, timeL and valueH for ref, n, p, bias, nnx1 and pnx1 channels sent at 76800 baud.
"""

from serial import Serial

ser = Serial('COM26', 76800)
while(1):
    print ord(ser.read(size=1)),
