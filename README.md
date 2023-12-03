# Arduino DAC Multiport

This is an Arduino program to make a 6 bit DAC using an [R2R
ladder](https://en.wikipedia.org/wiki/Resistor_ladder) connected to the
corresponding pins on the Arduino.

The code is a slight modified version of a previously existing one provided by
[FIUBA](https://es.wikipedia.org/wiki/Facultad_de_Ingenier%C3%ADa_(Universidad_de_Buenos_Aires))'s
professors from lecture "Introducción a la Ingeniería Electrónica", and
corresponds to the Final Group Project. The modification adds the ability to use
pins from different ports instead of all the pins from the same port.

The ports are 8 bits registers that Arduino uses to internally represents the
output pins. You can find which pin correspond to which port by looking on any
Arduino pinout diagram, being the most common ports PORTB, PORTC and PORTD.

The code uses PORT manipulation since its faster than using builtin function
like pinMode(), and thus improving the softness of the recreated wave by the
R2R.

The following statements produce the same effect on the arduino output pins and
shows the difference in speed of using builtin functions and port manipulation
(Taken from [this arduino forum
question](https://forum.arduino.cc/t/ddr-vs-pinmode-solved/497927/3))

```c
pinMode(5, OUTPUT);  takes 71 clock cycles (~4.43us@16Mhz)
DDRD |= 0b00010000;  takes  2 clock cycles (~0.125us@16Mhz)
bitSet(PORTD, 5, 1); takes  2 clock cycles (~0.125us@16Mhz)
```

Also assigning the new value to the port allow for multiple ports to be updated
on a single statement, for example turning on both pin 4 and 5 at the same time
can be done with the following statement

```c
DDRD |= 0b00011000;
```

For more information I suggest reading [Arduino Reference on Port
Manipulation](https://www.arduino.cc/en/Reference/PortManipulation)
