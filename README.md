# Fan controller in C Programming
Final submission for EE30186 | University of Bath

## About
The goal is to control the speed of the fan according to the user input using the DE1-SoC FPGA.
Author: Oscar Rovira

## Modes : Closed and Open Loop
- Open Loop Mode: User can select the desired speed of the fan by directly controlling the output duty cycle of the PWM waveform being sent to the fan.
- Closed Loop Mode: User can select the desired speed of the fan (RPM's) and the program will calculate the required duty cycle using a PID.

## Display
- Open Loop Mode: Fan Speed(RPM) or Duty Cycle(%)
- Closed Loop Mode: Error, Target Speed(RPM), Fan Speed(RPM) or Duty Cycle(%)

## Instructions:
Make sure all switches (SW0-9) are down before compiling to ensure complete initialisation of the system

1. To turn on the system toggle left switch (SW9) up.
2. To turn off the system toggle left switch (SW9) down.
3. To turn on Closed Mode toggle right switch (SW0) up.
4. To turn on Open Mode toggle right switch (SW0) down.
5. To display current duty cycle press KEY0.
6. To display fan speed press KEY1.
7. In closed mode, to display target fan speed press KEY2.
8. In closed mode, to display error press KEY3.
9. In closed mode, use the knob to select the target speed in RPM.
10. In open mode, use the knob to select the desired duty cycle.
