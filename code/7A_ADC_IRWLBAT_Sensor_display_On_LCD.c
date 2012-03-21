#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"

#define kp 220
#define ki 30
#define kd 20

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value1,ADC_Value2;

unsigned char sensor1,sensor2;
int Disturbance_temp;
unsigned int Disturbance;
unsigned char vel;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void soft_stop (void)    //hard stop (Stop slowly)
{
  motion_set(0x0F);
}

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //higher byte constant frequency value of PWM cycle 
 TCNT1L = 0x01; //lower byte constant frequency value of PWM cycle 
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;  //set PORTA direction as input
 PORTA = 0x00; //set PORTA pins floating
}

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; //set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function to Initialize PORTs
void port_init()
{
 motion_pin_config();
 lcd_port_config();
 adc_pin_config();	
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
 while((ADCSRA&0x10)==0);	    //Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;          //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor;     // duty cycle 'ON' period of PWM out for Left motor 
 OCR1BH = 0x00;
 OCR1BL = right_motor;    // duty cycle 'ON' period of PWM out for Right motor 
}




void init_devices (void)
{
 cli();           //Clears the global interrupts
 port_init();
 adc_init();
 timer1_init();
 sei();           //Enables the global interrupts
}

//Main Function
int main(void)
{
 int error  = 0;
 int error1 = 0; 
 int error2 = 0;
 int error3 = 0;
 int error4 = 0; 
 int error5 = 0;
 int error6 = 0;
 int error7 = 0; 
 int error8 = 0;
 int error9 = 0;
 
 int error_i = 0;
 int error_d = 0;
 
 init_devices();
 lcd_set_4bit();
 lcd_init();

	int max = 0;
	int max1 = 0;
 while(1)
 {            
	sensor1=ADC_Conversion(2);
	lcd_print(1,1,sensor1,3);
		 
		 error9 = error8;
		 error8 = error7;
		 error7 = error6;
		 error6 = error5;
		 error5 = error4;
		 error4 = error3;
		 error3 = error2;
		 error2 = error1;
		 error1 = error;
		 
		 error  = (sensor1 - 140);
		 error_i = (error1 + error2 + error3 + error4 + error5 + error6 + error7 + error8 + error9);
		 error_d = (error-error1);

		 Disturbance_temp = ((error*kp) + (error_i*ki) + (error_d*kd));
		 Disturbance = (unsigned int) Disturbance_temp;
		 Disturbance = (Disturbance /500 ) ;
		 vel= (char)Disturbance;

	//	lcd print(1,5,vel,5);
	//	lcd_print(2,1,max1,5);
	//	float BATT_Voltage = (ADC_Conversion(6) * 0.03921) + 0.7;	//Prints Battery Voltage Status
	//	lcd_print(2,13,BATT_Voltage,4);
		if ( error < 0 ) {
			error *= -1;
		}
	//cd_print(2,1,error,4);
		if(sensor1 > 140)
		{
		 back();
		}
		else
		{
		 forward();
		}
		velocity(100+vel,100+vel);
		}
}
