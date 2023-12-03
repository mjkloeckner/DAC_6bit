// En este ejemplo se genera una señal senoidal como salida. Se utilizan
// registros para manipular los puertos para mayor velocidad, obteniendo asi una
// señal mas suave.

// Se utilizan los pines D2, D4, D6, D8, D10 y D12 de Arduino UNO (o nano) para
// implementar un DAC de 6 bits. Dichos pines se deben conectar al circuito R2R
// correspondiente, siendo los pines D8, D10 y D12 los mas significativos

// Diferencia de tiempo entre usar registros para definir el estado de un pin y
// usar la funcion incorporada pinMode().
// https://forum.arduino.cc/t/ddr-vs-pinmode-solved/497927/3
// 
// Las sentencias siguientes tienen el mismo objetivo, activar el pin 5
// pinMode(5, OUTPUT);  tarda 71 ciclos de clock (~4.43us@16Mhz)
// DDRD |= 0b00010000;  tarda 2 ciclos de clock (~0.125us@16Mhz)
// bitSet(PORTD, 5, 1); tarda 2 ciclos de clock (~0.125us@16Mhz)
// 
// La diferencia entre usar la funcion bitSet() y asignar directamente el valor 
// al registro DDRD, es que con el ultimo se pueden activar varias pines al
// mismo tiempo, mientras que con la funcion bitSet() solo uno.

// Máscara correspondiente a los pines utilizados del Puerto B
// y del puerto D respectivamente
// https://www.arduino.cc/en/Reference/PortManipulation
#define DAC_MASK_PORTB B00010101
//                         | | |
//                         | | `-- D8
//                         | `-- D10
//                         `-- D12

#define DAC_MASK_PORTD B01010100
//                       | | |
//                       | | `-- D2
//                       | `-- D4
//                       `-- D6

// Tabla con un periodo de un seno muestreado (32 muestras, 8 bits)
const uint8_t tabla[] PROGMEM = {
  128,153,178,200,220,236,247,254,255,251,242,228,
  211,189,166,140,115,89,66,44,27,13,4,0,1,8,19,35,55,77,102
};

const uint8_t largo = sizeof(tabla); // Tamaño de la tabla
uint8_t indice = 0;     // Variable para indexación de la tabla
uint8_t paso = 1;       // Incremento del índice
uint8_t valor;          // Variable para almacenar el valor de cada elemento

void printR2R (uint8_t);

void setup() {
  DDRD |= DAC_MASK_PORTD; // Define como salidas a los pines D2, D4 y D6
                          // (pertenecen al puerto D)
  DDRB |= DAC_MASK_PORTB; // Define como salidas a los pines D8, D10 y D12
                          // (pertenecen al puerto B)
}

void loop() {
  // Se usa una función especial para leer de la memoria de programa
  valor = pgm_read_byte(&tabla[indice]);
  
  // Se envía el valor de 8 bits al DAC
  printR2R(valor);
  
  indice += paso;     // Se incrementa el índice a la próxima muestra
  indice %= largo;    // Se limita el índice al tamaño de la tabla

  delayMicroseconds(98);   // Limitación en el tiempo del bucle loop
                             // Permite generar senoidales de menor frecuencia
}

// Esta función desprecia los ultimos 2 bits menos significativos de `numero`
void printR2R (uint8_t numero) {
  // El parametro `numero` solo puede representar hasta 255 ya que es de 8 bits
  // Como el DAC es de 6 bits se desprecian los ultimos dos bits menos
  // significativos de `numero`. Los 6 bits restantes se mandan al R2R a través
  // de lo pines correspondientes

  // Ej 71 (0100 0110)
  //        |||| ||||
  //        |||| ||``- resolucion de 6 bits, se desprecian los
  //        |||| ||    bits menos significativos (sin pin asignado)
  //        |||| ||
  //        |||`-``---- PORTD (LSB)
  //        |||
  //        ```--- PORTB (MSB)

  // PORTD es un registro interno del microcontrolador utilizado 
  // por el arudino, internamente se representa con una variable de 8 bits
  // correspondiendo cada bit a un pin en del arduino, como se muestra
  // a continuacion

  // PORTD = `0bABCDEFGH` correspondiendo cada bit a los pines siguientes:
  //            ||||||||
  //            |||||||`- D0 (RXD)
  //            ||||||`- D1 (TXD)
  //            |||||`- D2
  //            ||||`- D3
  //            |||`- D4
  //            ||`- D5
  //            |`- D6
  //            `- D7

  // PORTB es otro registro de 8 bits similar a PORTD pero representa otros
  // pines

  // PORTB = `0bABCDEFGH` siendo:
  //            ||||||||
  //            |||||||`- D8
  //            ||||||`- D9
  //            |||||`- D10
  //            ||||`- D11
  //            |||`- D12
  //            ||`- D13
  //            |`- X (No disponible en Arduino)
  //            `- X (No disponible en Arduino)

  // Teniendo en cuenta que PORTB contiene los bits mas significativos
  // es conveniente modificar ese puerto primero, ya que hasta la siguiente
  // instruccion de modificacion del PORTD, el R2R recibira un valor mas
  // aproximado al valor final

  // Para asignar un nuevo valor a PORTB primero se borra el contenido
  // anterior de los pines utilizados, dejando los sin utilizar como
  // estaban. Luego adiciona el nuevo contenido con la operación OR

  // De PORTB se utilizan los pines D8, D10 y D12 siendo los
  // bits H, F y D de PORTB respectivamente
  PORTB = (PORTB &~ DAC_MASK_PORTB)|
             ((numero & (1<<7))>>3)|
             ((numero & (1<<6))>>4)|
             ((numero & (1<<5))>>5);

  // De PORTD se utilizan los pines D6, D4 y D2 siendo los bits B, D y F
  // de PORTD respectivamente
  PORTD = (PORTD &~ DAC_MASK_PORTD)| // Elimina el contenido anterior
             ((numero & (1<<4))<<2)| // obtiene el valor del bit
             ((numero & (1<<3))<<1)| // en la 6ta posicion y lo mueve
              (numero & (1<<2));     // para que corresponda con el lugar del
                                     // pin asignado en el registro de PORTD
}
