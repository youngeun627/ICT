/*
 * door_lock_system.c
 *
 * Created: 2020-02-29 오후 2:48:09
 * Author : chw91
 */ 

/*
TIM1 : SERVO
TIM0 : BUZZER
UART1 : BT
UART0 : Touch Sensor (USB to Serial : UART0)

GPIOA : LED (RGB? >> PWM을 통해 신호를 주면 확실하긴 함 )
*/

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)



void setup();

//TIM
void setOCRA(int num);
void setOCRB(int num);
void setICR(int num);
int convertServoAngle(int angle);
/*
SERVO MOTOR
Control Period : 20ms
angle : 1~2ms  >> OCR 2000~4000
*/
//UART
void UART0_Init(unsigned int ubrr);
void uartTx(unsigned data);
int stack=0;




int main(void)
{
	setup();
    /* Replace with your application code */
	setOCRA(convertServoAngle(90));
	
    while (1) 
    {
		_delay_ms(500);
		
		switch(stack){
		case 0:
			uartTx('a');
		break;
		case 1:
			uartTx('b');
		break;
		default:
		break;
		}
		stack++;
		stack%=2;
		
    }
}

void setup()
{
	SREG |= 1<<7; 
	DDRB = 0xff; 
	DDRC = 0xff;
	
	//UART0
	UART0_Init(MYUBRR);
	
	//TIM0
	//16Mhz > 1024 prescailing >>15.6kHz/count >> 0.064ms >> 16.38ms(pwm period)
	TCCR0 = ((1<<CS00) | (1<< CS01) | (1<<CS02) | (1<<WGM00)|(1<<WGM01) |(1<<COM01));
	
	//TIM1
	//COM : 10 : default HIGH, compare LOW 
	//WGM : 1110 : FAST PWM, TOP:ICR1
	TCCR1A = ((1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10));
	TCCR1B = ((0<<CS12)|(1<<CS11)|(0<<CS10)|(1<<WGM13)|(1<<WGM12) );
	
	TCNT0=0;
	TCNT1H = 0;
	TCNT1L = 0;
	
	setICR(39999);//0x9c3f
	
}
void setOCRA(int num)
{
	OCR1AH = (unsigned char)(num>>8);
	OCR1AL = (unsigned char)(num&0xff);
	
}

void setOCRB(int num)
{
	OCR1BH = (unsigned char)(num>>8);
	OCR1BL = (unsigned char)(num&0xff);
	
}

void setICR(int num)
{
	//39999 == 0x9c3f
	//high write first
	ICR1H = (unsigned char)(num>>8);
	ICR1L = (unsigned char)(num&0xff);
	

}

int convertServoAngle(int angle)
{
	//0~180 degree
	int converted = 2000+angle*11;
	return converted;
}


void UART0_Init(unsigned int ubrr)
{
	//set Baud Rate
	UBRR0H= (unsigned char )(ubrr>>8);
	UBRR0L= (unsigned char )(ubrr&0xff);
	
	// Tx, Rx Enabled
	UCSR0B = ((1<<TXEN0) | (1<RXEN0));
	
	// Set Frame format: data 8 bit, 1 stop bit
	UCSR0C = (0<<USBS0) |(1<<UCSZ01)| (1<<UCSZ00 );
}


void uartTx(unsigned data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=data;
}