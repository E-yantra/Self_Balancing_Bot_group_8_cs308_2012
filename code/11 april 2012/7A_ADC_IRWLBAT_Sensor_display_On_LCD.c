#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"

double kp = 4.5;
double ki = 0.0;
double kd = 0.0;
double dt = 0.001;
#define MAX 240
#define MIN -240

unsigned char ADC_Conversion(unsigned char);
unsigned char sensor2;
double Disturbance;

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

//////////////////////////////////////////////
typedef struct{
double q;
double p;
double r;
double x;
double k;
}kalman_state;


kalman_state kalman_init(double q, double r, double p, double initial_value)
{
	kalman_state result;
	result.q = q;
	result.r = r;
	result.p = p;
	result.x = initial_value;
	return result;
}

void kalman_update(kalman_state* state , double measurement)
{
	state->p = state->p + state->q;
 	state->k = state->p/(state->p + state->r);
	state->x = state->x +state->k*(measurement - state->x);
	state->p = (1 - state->k)*state->p;
}
//////////////////////////////////////////////

//Main Function
int main(void)
{
 double error_p = 0.0;
 double error_i = 0.0;
 double error_d = 0.0;
 double angle;
 kalman_state k = kalman_init(1, 16, 1, 137);
 init_devices();
 lcd_set_4bit();
 lcd_init();
 double preverror = 0.0;
	 while(1)
	 {            
		sensor2 = ADC_Conversion(2);
        kalman_update(&k,(double)sensor2);
		angle = k.x;  
     	lcd_print(1,5,(int)angle,3);	 
		error_p = (angle - 137);
		error_i += error_p*dt;
		error_d = (error_p - preverror)/dt;
		lcd_print(1,1,abs((int)error_p),3);
		Disturbance = ((error_p*kp) + (error_i*ki) + (error_d*kd));
		if(angle > 137) back();
		else { 	forward();}
		if (Disturbance > MAX) Disturbance = MAX;
		if (Disturbance < MIN) Disturbance = MIN;
		lcd_print(2,6,abs((int)Disturbance),5);
		velocity(35+abs((int)Disturbance),35+abs((int)Disturbance));
		preverror = error_p;
	}
}
