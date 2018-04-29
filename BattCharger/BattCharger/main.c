/*
 * BattCharger.c
 *
 * Created: 4/21/2018 2:37:40 PM
 * Author : NJK
 */ 

#define F_CPU 14745600UL

#define OFF    0
#define ON     1
#define TOGGLE 2

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <i2cmaster.h>


ISR(TIMER1_COMPB_vect)
{
   
}

            // *** Serial Communication *** ///

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
void initUART(void);
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,_FDEV_SETUP_WRITE);

               // **** SPI  ***/

void initSPI(void);
void writeSPI(unsigned char c);
unsigned char readSPI(void);
void clockSPI(void);
void selectSPI(void);
void deselectSPI(void);

         // *** MAX31856 Temperature Sensor *** //

void configureMAX31856(void);
float getTemperature(void);

         // *** INA219 Current Sensor ***//

#define DevISENSE 0b10000000              // I2c Address of sensor
float getCurrent(void);
float getAveCurrent(unsigned char n);

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))

      // *** Alpha Numeric Display ***//

void initAlpha();
void sendAlphaByte(unsigned char c);
void sendAlphaWord(uint16_t c);
void selectAlpha(void);
void deselectAlpha(void);
void testAlpha(void);

      // *** DAC ***//

#define DevDAC 0b01011110 // I2C address of  DAC

void resetDAC(unsigned char channel);
void setDACValue(unsigned char channel,float voltage);

       //*** ADC ***//

void getValue(void);
void initADC(void);
uint16_t readADC(uint8_t ch);

      // *** Misc Other Items ***/ 

void setLED(unsigned char state);

void initPWM(void);
void setPWM(float duty);

int main(void)
{
   float temp,current;
   float Ilimit = 0.13;     // Limit current to 130 mA.
   float Vout = 3.50;
   char buff[10];
   uint16_t txt;
 
   // Initialize the alphanumeric display.
    
   initAlpha();
   _delay_ms(100);
   testAlpha();    
   
   // Fire up the USART and print a welcome message.    
   
   initUART();
   stdout = &mystdout;
   printf("Multiple Chemistry Battery Charger.  Version (1.0) \r\n");

   // Initialize the SPI and I2C buses and the peripherals.
       
   initSPI();              // SPI is used by the MAX31856 thermocouple interface
   configureMAX31856();   
   i2c_init();             // I2C is used by the INA219 current sensor
   _delay_ms(500.0);
       
   // Initialize the DAC.

   resetDAC(0);		      // Reset channel 0
   resetDAC(1);		      // Reset channel 1    
   setDACValue(0,Vout);
   
   // Initialize the ADC.
   
   initADC();
   getValue();
   
   while (1){ 
      
      // Current limit
      
      current = getAveCurrent(100);
      while (current >= Ilimit){
         Vout = Vout - 0.05;
         setDACValue(0,Vout);
         current = getAveCurrent(100);         
         printf("Reducing current to: %5.3f A\r\n",current);
          
      }
      
      initAlpha();
      sprintf(buff,"%3d",(char)(current*1000));
      printf("-> %s\r\n",buff);
      txt = (0<<8 | buff[0]);
      sendAlphaWord(txt);
      _delay_ms(20);
      txt = (1<<8 | buff[1]);
      sendAlphaWord(txt);
      _delay_ms(20);
      txt = (2<<8 | buff[2]);
      sendAlphaWord(txt);
       _delay_ms(20);      
      txt = (3<<8 | 'A');
      sendAlphaWord(txt);

                  
      temp = getTemperature();       
      printf("Current = %5.3f A, Temperature = %5.1f C\r\n",current,temp);
      getValue();
      _delay_ms(500);
      setLED(TOGGLE);    
    }   
              
}


void resetDAC(unsigned char channel)		// Resets/clears all the DAC registers
{
   i2c_start_wait(DevDAC+I2C_WRITE);		// Issue START and then send the address
   i2c_write(0b00010000+channel);			// Send RESET command
   i2c_stop();								      // Stop I2C conversation
   printf("DAC %d reset.\r\n",channel);
}

void setDACValue(unsigned char channel,float voltage)
{
   unsigned char d;
   float tmp;
   
   voltage = voltage +  0.00;
   tmp = (voltage*100)*(255.0);
   tmp = tmp/500.0;
   d = (unsigned char)(tmp+0.5);
   
   i2c_start_wait(DevDAC+I2C_WRITE);	// Issue START and then send the address
   i2c_write(0x00+channel);			// Select DAC0 or DAC1
   i2c_write(d);					    // Write data
   i2c_stop();							// Issue a STOP

  // printf("\r\nDAC channel %d is set to %5.2f V = (%dd)\r\n",channel,voltage,d);
}


void testAlpha(void)       // Driver to test the Alphanumeric display
{
   uint16_t data;
   
   initAlpha();
   _delay_ms(20);
      
   data = (0 << 8) | '1';
   sendAlphaWord(data);
   _delay_ms(20);
   
   data = (1 << 8) | '0';
   sendAlphaWord(data);
   _delay_ms(20);   

   data = (2 << 8) | '0';
   sendAlphaWord(data);
   _delay_ms(20);
   
   data = (3 << 8) | '%';
   sendAlphaWord(data);
   _delay_ms(10);   
}
   
void selectAlpha(void)
{
   CLEARBIT(PORTC,2);      // Select Alpha display by pulling SELECT line low  
}

void deselectAlpha(void)
{
   SETBIT(PORTC,2);        // Select Alpha display by pulling SELECT line low  
}

void sendAlphaByte(unsigned char c)
{
   int i;
   
   SETBIT(PORTC,0);                 // Make sure Clock line is high
   selectAlpha();
 
  // Now shift out 8 bits, with MSB first 
   for (i=0;i<=7;i++){
    
      if (CHECKBIT(c,(7-i)))
         SETBIT(PORTC,1);
      else
         CLEARBIT(PORTC,1);
   
      // Make a (negative) clock pulse         
      _delay_us(10.0);
      CLEARBIT(PORTC,0);            // Pull clock line low
      _delay_us(15.0);
      SETBIT(PORTC,0);              // Pull clock line high
   }
   SETBIT(PORTC,2);                 // Before we leave, make clock line high 
   deselectAlpha();  
}

// Send a word (16 bits) to the alphanumeric display
// The MSB is the digit to be written to (0,1,2,3)
// The LSB is the data to write at that digit.

void sendAlphaWord(uint16_t c)   
{
   int i;
   
   SETBIT(PORTC,0);                 // Make sure Clock line is high
   selectAlpha();
      
   // Now shift out 8 bits, with MSB first
   
   for (i=0;i<=15;i++){
      
      if (CHECKBIT(c,(15-i)))
         SETBIT(PORTC,1);
      else
         CLEARBIT(PORTC,1);
      
      // Make a (negative) clock pulse
      
      _delay_us(10.0);
      CLEARBIT(PORTC,0);            // Pull clock line low
      _delay_us(15.0);
      SETBIT(PORTC,0);              // Pull clock line high
   }
   SETBIT(PORTC,2);                 // Before we leave, make clock line high
   deselectAlpha();
}

void initAlpha()  // Initializes the alphanumeric display   
{
   DDRC = DDRC | _BV(0);   // Clock line
   DDRC = DDRC | _BV(1);   // Data line
   DDRC = DDRC | _BV(2);   // ~Select line
   
   SETBIT(PORTC,0);        // Clock high
   SETBIT(PORTC,2);        // Make SELECT high, which unselects the alpha interface   
}


 void setLED(unsigned char state)   // Sets the indicator LED ON or OFF
 {
    SETBIT(DDRB,4);            // Make PORTB.4 output
    switch(state)  {
      case ON:  
         CLEARBIT(PORTB,4);
         break;
      case OFF:
         SETBIT(PORTB,4);
         break;
      case TOGGLE:
         FLIPBIT(PORTB,4);
         break;
      default:
         ;
    }             
 }
 

 
float getCurrent(void)
{
   unsigned char c1,c2;      
   int16_t  current;
   float tmp;
    
   // The voltage across the shunt resistor is in register 1 of the INA219 chip.
   // The first thing we do is to tell the chip we will be reading from this
   // register.
   
   i2c_start_wait(DevISENSE+I2C_WRITE);   // This wait for the ACK from the INA219
   i2c_write(0x01);                       // If we were configuring the chip we would
   i2c_stop();                            // send config data.  For now just stop.

   // The read will come from the last address we wrote to, which is
   // register 1, where the shunt voltage is.     
      
   i2c_start_wait(DevISENSE+I2C_READ);		// Issue START and then send the address
   c1 = i2c_readAck();                    // Get MSB of the shunt voltage
   c2 = i2c_readNak();                    // Get LSB of the shunt voltage
   i2c_stop();
     
   current = (c1 << 8) | c2;
   
   // The voltage across the shunt is in mV and the shunt is 0.1 Ohm.
   // Multiply measured values by 1e-4 to convert to A.
   
   tmp = (float)(current)*(1.0e-4);
   return(tmp);
}

float getAveCurrent(unsigned char n)
{
   float total;
   int i;
   
   total = 0;
   for (i=0;i<=n;i++){
      total = total + getCurrent();
      _delay_ms(10);
   }
   return(total/(float)n);
}

float getTemperature(void)
{
   unsigned char c1,c2,c3;
   int32_t temp;
   float value;
      
   selectSPI();
   writeSPI(0x0C);      // Start reading at register 0x0C
   c1 = readSPI();      // Read register and increment to 0x0D 
   c2 = readSPI();      // Read register and increment to 0x0F
   c3 = readSPI();      // Read register
   deselectSPI();
   
   // Now pack bytes and convert to float.
   
   temp =((c1 & 0xFF) << 11); 
   temp |=((c2 & 0xFF) << 3); 
   temp |=((c3 & 0xFF) >> 5); 
   value = (temp/128.0f);
   return(value);
}
void configureMAX31856(void)
{
   selectSPI();
   writeSPI(0x80);   // Select MAX31856 register 0 in write mode
   writeSPI(0x80);   // Program is to do automatic conversions (every 100 ms)
   deselectSPI(); 
}

void selectSPI(void) // Pull CS line low.
{
    PORTB = PORTB & (~0x01);
}

void deselectSPI(void)  // Pull CS line high
{
   PORTB    = PORTB | 0x01; 
}
void clockSPI()
{
     PORTB = PORTB | 0b00000010; // make clock line high
     _delay_us(1.0);
     PORTB = PORTB & 0b11111101; // make clock line low 
}
void writeSPI(unsigned char c)
{
   unsigned char i, msk;
   
   msk = 0x80;
   for (i=0;i<=7;i++){
      if ((c & msk) == 0)
         PORTB = PORTB & 0x1111011;        
      else
         PORTB = PORTB | 0b0000100;    
      msk = msk >> 1;
     clockSPI();
   }
}

unsigned char readSPI()
{
  unsigned char i, msk, c; 
   c = 0;
   msk = 0x80;
   for (i=0;i<=7;i++){
      clockSPI();
      if ((PINB & 0b0001000)==0) 
         ;
      else
         c = c | msk;
      msk = msk >> 1;    
   } 
   return(c);
}

void initSPI(void)
{
   DDRB = DDRB | _BV(0) | _BV(1) | _BV(2);  // Set MOSI and SCK output, all others input
   PORTB = PORTB | 0x01;                    // Chip select high
   PORTB = PORTB & 0b11111101;              // make clock line low   
}

   

// Initialize serial port 0 to 9600 8N1

void initUART(void){  
   UBRR0L = BAUD_PRESCALE;
   UBRR0H = (BAUD_PRESCALE >> 8); 
   UCSR0B = ((1<<TXEN)|(1<<RXEN));
}

// Write one character to the stream associated with serial port 0
static int uart_putchar(char c, FILE *stream)
{
   loop_until_bit_is_set(UCSR0A, UDRE);
   UDR0 = c;
   return 0;
}

void initADC(void)
{
   // AREF = AVcc
   ADMUX = (1<<REFS0);
   
   // ADC Enable and prescaler of 128
   // 16000000/128 = 125000
   ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t readADC(uint8_t ch)
{
   // Select the corresponding channel 0 through 7.  For our
   // application we could have hard coded the channel number.
   
   ch &= 0b00000111;
   ADMUX = (ADMUX & 0xF8) |ch;
   
   // Start single conversion by writing a 1 to the ADSC bit in
   // ADCSRA. The wait for the ADSC bit to become 0, which signals
   // the end of the conversion. Delay needed for the stabilization 
   // of the ADC input voltage.
   
   _delay_us(10.0); 
   ADCSRA |= (1<<ADSC);
   while(ADCSRA & (1<<ADSC))
      ;
   return (ADC);
}

// Read the AD conversion result

void getValue(void)
{	uint16_t d;
   float v;
   
   d = readADC(0);
   v = ((float)(d*5))/(1023.0);
   printf("v = %6.3f V\r\n",v);
}

void initPWM(void)
{
   
// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Fast PWM top=0x03FF
// OC1A output: Non-Inverted PWM  (PB5)
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.069444 ms
// Output Pulse(s):
// OC1A Period: 0.069444 ms Width: 0.034756 ms
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (1<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;

OCR1AH=0x02;
OCR1AL=0x00;

OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

SETBIT(DDRB,5);
}

void setPWM(float duty)
{
   uint16_t di;
   
   // Scale dutu cycle to lie between 0 and 1023.
   
   di = (uint16_t)(duty*1023);
   cli();
   OCR1AH = (uint8_t)(di >> 8);
   OCR1AL = (uint8_t)(di & 0x0FF);
   sei();
}

#ifdef JUNK
// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
   ADMUX=adc_input | ADC_VREF_TYPE;
   // Delay needed for the stabilization of the ADC input voltage
   delay_us(10);
   // Start the AD conversion
   ADCSRA|=(1<<ADSC);
   // Wait for the AD conversion to complete
   while ((ADCSRA & (1<<ADIF))==0);
   ADCSRA|=(1<<ADIF);
   return ADCW;
}
#endif