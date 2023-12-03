# Arduino DAC Multiport

This is an Arduino program to make a 6Bit DAC using an [R2R
ladder](https://en.wikipedia.org/wiki/Resistor_ladder) connected to the
corresponding pins.

The code is a slight modified version of a previously existing one that adds the
ability to use pins from different ports instead of all the pins from the same
port.

The ports are 8 bits registers that Arduino uses to internally represents the
output pins. You can find which pin correspond to which port by looking on any
Arduino pinout diagram, being the most common ports PORTB, PORTC and PORTD.
