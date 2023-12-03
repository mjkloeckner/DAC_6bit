// Transceptor
//
// Diferencia de tiempo entre usar registros para definir el estado de un pin y
// usar la funcion incorporada pinMode().
//
// https://forum.arduino.cc/t/ddr-vs-pinmode-solved/497927/3
//
// DDRD |= 0b00010000;  tarda 2 ciclos de clock (~0.125us@16Mhz)
// bitSet(PORTD, 5, 1); tarda lo mismo que con la sentence previa
// pinMode(5, OUTPUT);  tarda 71 ciclos de clock (~4.43us@16Mhz)

#define DAC_MASK_PORTB B00010101
#define DAC_MASK_PORTD B01010100

void printR2R (uint8_t);
void USART_Transmit(uint8_t data);
uint8_t USART_Receive(void);

void setup() {
  DDRD |= DAC_MASK_PORTD; // Define como salidas a los pines D2, D4 y D6
  DDRB |= DAC_MASK_PORTB; // Define como salidas a los pines D8, D10 y D12

  Serial.begin(115200);

  DDRC = 0x00; // Define los pines del puerto C como inputs (0 INPUT, 1 OUTPUT)

  // Registro de Control del ADC
  ADCSRA = 0x86; // 1000 0110
                 // |     ||
                 // |     ``- Determinan el factor de division. Con ambos bits
                 // |         activos setea un factor de division de 128 (max)
                 // |
                 // `- Activa el ADC

  // Registro de selección de multiplexion del ADC
  ADMUX = 0x60;  // 0110 0000
}

void loop() {
  // Lee un valor del pin analogico, luego lo transmite por protocolo USART
  USART_Transmit(analogRead(A0));

  // Los bits recibidos los manda a la salida
  printR2R(USART_Receive());
}

void printR2R (uint8_t num) {
  // PORTB = 0bABCDEFGH
  // De PORTB se utilizan los pines D8, D10 y D12 siendo los
  // bits H, F y D de PORTB respectivamente
  PORTB = (PORTB &~ DAC_MASK_PORTB)| // Elimina el contenido anterior
             ((num & (1<<7))>>3)|    // Obtiene el valor del bit
             ((num & (1<<6))>>4)|    // en la 6ta posicion y lo mueve
             ((num & (1<<5))>>5);    // para que corresponda con el lugar del
                                     // pin asignado en el registro de PORTD

  // PORTD = 0bABCDEFGH
  // De PORTD se utilizan los pines D6, D4 y D2 siendo los bits B, D y F
  // de PORTD respectivamente
  PORTD = (PORTD &~ DAC_MASK_PORTD)|
             ((num & (1<<4))<<2)|
             ((num & (1<<3))<<1)|
              (num & (1<<2));
}

void USART_Transmit(uint8_t data) {
  //while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = data;
}

uint8_t USART_Receive(void) {
  return UDR0;
}
