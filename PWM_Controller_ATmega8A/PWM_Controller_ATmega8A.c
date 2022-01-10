
/*
 * PWM_Controller_ATmega8A.c
 * Created: 09.04.2021 7:44:38
 * Author: MikhnovetsA
 * ��������� ��� �������, ��������� ��� ������, 0-10 ����� � 4-20 ��. ���������� ����������� ���� ���������, ���� �� ����������� ����� 0-10 �����. ��������� �� ������ �������������� �����������.
 */ 

#define F_CPU 16000000			//������� ������ ����������������

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ADC_in	6				//��� ���

#define SegA	7				//		AAAAA		PORTD
#define SegB	1 				//	   F	 B		PORTC
#define SegC	3 				//	   F	 B		PORTC
#define SegD	3 				//		GGGGG		PORTD
#define SegE	4 				//	   E	 C		PORTD
#define SegF	0 				//	   E	 C		PORTB
#define SegG	2 				//		DDDDD		PORTC
#define DP		4 				//				DP	PORTC

#define Dig1	6				//������� ������	PORTD
#define Dig2	5				//������� ������	PORTD
#define Dig3	0				//������� ������	PORTC
#define Dig4	5				//������� ������	PORTB

#define But		2				//��� ��� ������	PORTD

#define Encoder1	1			//������� ��� �		PORTD
#define Encoder2	0			//������� ��� �		PORTD

#define PWM_OC1A	1			//��� ���			PORTB
#define PWM_OC1B	2			//��� ���			PORTB
#define PWM_OC2		3			//��� ���			PORTB

uint8_t btn_state = 1;			//���������� ��� ����������� ������

volatile uint8_t indi = 1;		//��� ������������ ������ ����������� 

uint8_t R1 = 0;
uint8_t R2 = 0; 
uint8_t R3 = 0;				
uint8_t R4 = 0;					//������ � ��������

volatile int16_t count = 0;		//������� �� ��������

uint16_t ADC_val = 0;			//������������� �������� ��� ������ �� �������

volatile uint8_t PWM5V = 0;		//���������� ��� 5-������������ ����
volatile uint8_t PWMGND = 0;	//���������� ��� ���������� ����

volatile int8_t analog_current = 0;
volatile int16_t analog_voltage = 0;
/*
uint16_t map_(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max){	//������� ��� ����������� �������� ������ �� ����� (��� � �������)
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
	����� ������, ���� ���� ���������� (������� ���� �������������) �������� �� ����� ��������� �������� � ��������� �� ������� �������� (���� ���� 8 ���, �� ��� 255).
}
*/

void seg_char(uint8_t seg){		//������� ��� ������ ��������
	switch (seg)
	{
		case 0: PORTD &= ~(_BV(SegA) | _BV(SegD) | _BV(SegE)); PORTC &= ~(_BV(SegB) | _BV(SegC)); PORTC |= _BV(SegG); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));							//0
			break;
		case 1: PORTD |= (_BV(SegA) | _BV(SegD) | _BV(SegE)); PORTC &= ~(_BV(SegB) | _BV(SegC)); PORTC |= (_BV(SegG)); PORTB |= (_BV(SegF)); PORTC |= (_BV(DP));							//1
			break;
		case 2: PORTD &= ~(_BV(SegA) | _BV(SegD) | _BV(SegE)); PORTC &= ~(_BV(SegB)); PORTC |= (_BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB |= (_BV(SegF)); PORTC |= (_BV(DP));				//2
			break;
		case 3: PORTD &= ~(_BV(SegA) | _BV(SegD)); PORTD |= (_BV(SegE)); PORTC &= ~(_BV(SegB) | _BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB |= (_BV(SegF)); PORTC |= (_BV(DP));				//3
			break;
		case 4: PORTD |= (_BV(SegA) | _BV(SegD)); PORTD |= (_BV(SegE)); PORTC &= ~(_BV(SegB) | _BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));				//4
			break;
		case 5: PORTD &= ~(_BV(SegA) | _BV(SegD)); PORTD |= (_BV(SegE)); PORTC |= (_BV(SegB)); PORTC &= ~(_BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));		//5
			break;
		case 6: PORTD &= ~(_BV(SegA) | _BV(SegD)); PORTD &= ~(_BV(SegE)); PORTC |= (_BV(SegB)); PORTC &= ~(_BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));	//6
			break;	
		case 7: PORTD &= ~(_BV(SegA)); PORTD |= (_BV(SegD) | _BV(SegE)); PORTC &= ~(_BV(SegB) | _BV(SegC)); PORTC |= (_BV(SegG)); PORTB |= (_BV(SegF)); PORTC |= (_BV(DP));					//7
			break;
		case 8: PORTD &= ~(_BV(SegA) | _BV(SegD)); PORTD &= ~(_BV(SegE)); PORTC &= ~(_BV(SegB)); PORTC &= ~(_BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));	//8
			break;
		case 9: PORTD &= ~(_BV(SegA) | _BV(SegD)); PORTD |= (_BV(SegE)); PORTC &= ~(_BV(SegB)); PORTC &= ~(_BV(SegC)); PORTC &= ~(_BV(SegG)); PORTB &= ~(_BV(SegF)); PORTC |= (_BV(DP));	//9
			break;
		case 10: PORTC &= ~(_BV(DP));																																						//DP
			break;
	}
}

void led_print(uint16_t number){	//������� ��� �������� ����� �� ������ ����������
	R1 = number/1000;				//������ ����� 
	R2 = number%1000/100;			//������ ����� 
	R3 = number%1000%100/10;		//������ �������� 
	R4 = number%1000%100%10;		//������ ������ 
}

void encoder(){
	static uint8_t new_state = 0;			//����� ��������� ����� ��������
	static uint8_t old_state = 0;			//������ ��������� ����� �������� (�������� �������� � ������� �����)
	new_state = (PIND & 0b00000011);		//������ ��������� �����, �������� �� � ������� ������� � ����������� ������� ���������
	switch (new_state << 2 | old_state)
	{
		case 0x01:  count++;						//������� ������
		if (btn_state == 1) PWM5V++;				//������ ���������� ������ � ���� ������
		else if(btn_state == 2) PWMGND++;			//������ ���������� ������ � ���� ������
		else if(btn_state == 4) analog_current++;	//������ ���������� ��� ����
		else if(btn_state == 5) analog_voltage++;	//������ ���������� ��� ����������
		break;
		
		case 0x04:  count--;						//������� �����
		if (btn_state == 1) PWM5V--;
		else if(btn_state == 2) PWMGND--;
		else if(btn_state == 4) analog_current--;
		else if(btn_state == 5) analog_voltage--;
		break;
	}
	
	if(count > 255) count = 0;
	if(count < 0) count = 255;
	
	if(PWM5V > 255) PWM5V = 0;
	if(PWM5V < 0) PWM5V = 255;
	
	if(PWMGND > 255) PWMGND = 0;
	if(PWMGND < 0) PWMGND = 255;
	
	if(analog_current > 21) analog_current = 21;
	if(analog_current < 0) analog_current = 0;
	
	if(analog_voltage > 255) analog_voltage = 255;
	if(analog_voltage < 0) analog_voltage = 0;
	
	old_state = new_state;
}

ISR(TIMER0_OVF_vect){		//���������� ��� ������������ ������� 0 ��� ������������ ���������
	
	switch (indi){
		case 1: PORTD &= ~(_BV(Dig1)); PORTD |= _BV(Dig2); PORTB |= _BV(Dig4); PORTC |= _BV(Dig3); seg_char(R1); //���������� ������ ���������, ��������� ���������. �� ��������� ��������� ����� �� �������� �������.
		break;
		case 2: PORTD |= _BV(Dig1); PORTD &= ~(_BV(Dig2)); PORTB |= _BV(Dig4); PORTC |= _BV(Dig3); seg_char(R2); 
		break;
		case 3: PORTD |= _BV(Dig1); PORTD |= _BV(Dig2); PORTB |= _BV(Dig4); PORTC &= ~(_BV(Dig3)); seg_char(R3); if(btn_state == 5) seg_char(10);
		break;
		case 4: PORTD |= _BV(Dig1); PORTD |= _BV(Dig2); PORTB &= ~(_BV(Dig4)); PORTC |= _BV(Dig3); seg_char(R4); 
		break;
	}
	indi++;
	if(indi > 4) indi = 1;
}

ISR(INT0_vect){				//��������� ������� ������ �� ���� �������� ���������� INT0
	btn_state++;
	if (btn_state > 5) btn_state = 1;
}

void setup(void)
{
	//��������� ������ ��� �����������
	DDRB |= _BV(SegF) | _BV(Dig4) | _BV(PWM_OC1A) | _BV(PWM_OC1B) | _BV(PWM_OC2);	//
	DDRC |= _BV(SegB) | _BV(SegC) | _BV(SegG) | _BV(DP) | _BV(Dig3);				//	��������� �� ����� ������ ��� �����������
	DDRD |= _BV(SegA) | _BV(SegD) | _BV(SegE) | _BV(Dig1) | _BV(Dig2);				//
	
	PORTB |= _BV(SegF) | _BV(Dig4);													//
	PORTC |= _BV(SegB) | _BV(SegC) | _BV(SegG) | _BV(DP) | _BV(Dig3);				//	��������� ����� � ������� ��� ���������� ���������, ��� ��� ��������� � ����� ������
	PORTD |= _BV(SegA) | _BV(SegD) | _BV(SegE) | _BV(Dig1) | _BV(Dig2);				//
	
	//��������� ������ ��� ������ � ��������
	DDRD &= ~(_BV(But) | _BV(Encoder2) | _BV(Encoder1));							//	��������� �� ����
	PORTD &= ~(_BV(But) | _BV(Encoder2) | _BV(Encoder1));							//	������������� ��������� ���������, ��� ��� ���� �� �����
	
	//��������� ������� ���������� �� INT0 ��� ������
	MCUCR |= (1 << ISC01);															//���������� ����������� �� ����� ������
	GICR |= (1 << INT0);															//�������� ���������� �� ���� INT0
	
	//��������� ������� 0 ��� ������������ ���������
	TCCR0 |= (1 << CS02);															//������������ �� 256 (��� 16 ��� 16000000/256/256 = 244)
	TCNT0 = 0;																		//�������� ������� �������
	TIMSK |= (1 << TOIE0);															//���������� ��� �����������
	
	//��������� ������� 1 ��� ���
	TCCR1A |= (1 << WGM10);															//����������� ��� (8 ���, � ������ �����)
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);										//����� ��� �� ���� ��1�(��1) � OCR1B(PB2)
	TCCR1B |= (1 << CS10) | (1 << CS11);											//������������ 64 (������� ��� ��� 16 ��� - 488 ��)
	TCNT1 = 0;																		//�������� ������� �������
	OCR1A = 0;	OCR1B = 0;															//�������� �������� ���������� (��������� � ���� ��������� ��������� �����������)
		
	//��������� ������� 2
	TCCR2 |=  (1 << CS22) | (1 << CS20);											//������������ �� 128 (�� ���� ������� ������� ��� �� ��2)
	TCCR2 |= (1 << WGM20) | (1 << WGM21);											//����� Fast PWM
	TCCR2 |= (1 << COM21);															//�� ���� ��2 ����������� ���
	TCNT2 = 0;																		//�������� ������� �������
		
	//��������� ���
	//ADMUX |= (1 << REFS1) | (1 << REFS0);											//�������� �������� �������� ���������� 5 �����
	ADMUX |= ((1 << MUX2) | (1 << MUX1));											//����� ��� ����� 6
	ADMUX &= ~(1 << ADLAR);															//�������������� ������������
	ADCSRA |= (1 << ADEN);															//�������� ���
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);							//�������� 128, ������� ������������� 125 ��� ��� 16 ��� ����������������
	ADCSRA |= (1 << ADFR);															//��� ��������� �������� ����������
}

int main(void)
{
	setup();								//��������� ������ � ���������
	
	sei();									//��������� ����������
	
    while(1)								//����
    {	
		switch(btn_state)					//����� ������ ������� �� ������� ������, ������� �� �������� ���������� ������������� ���������� btn_state
		{
			//����� ���������� ����� +5 ����� �� �������
			case 1: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 1)
				{
					OCR1A = PWM5V;				//����� � ������� ��������� �������� ���������� ��������
					led_print(PWM5V*100/255);	//������� �� ������� �������� ���������� ��������
					encoder();
				}
				break;
				
			//����� ���������� �����, ����������� �������. ����� �������� ������������ � ������ ��1�, ����� ��������� ������� � GND. ���������� �� 24 �����.
			case 2: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 2)
				{
					OCR1B = PWMGND;
					led_print(PWMGND*100/255);
					encoder();
				}
				break;
				
			//����� ���������� ����� ����� �� �������� ����������� ������� 0-10 �����. (���� ���������� ���!!!....)
			case 3: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 3)
				{
					ADCSRA |= (1 << ADSC);			//��������� ���������
					if (ADCSRA & (1 << ADIF))		//...���� ���������� ��� ������
					{
						OCR1B = ADC;				//������������ �������� �������� ��������� ��� �������� ��������� ������� 1
						ADCSRA |= (1 <<ADIF);		//������� ��� � ���������� ��������� (��������� ����)
					}
					led_print(PWMGND*100/1023);
				}
				break;
				
			//����� ���� 0-20 ��� 4-20 ��
			case 4: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 4)
				{
					OCR2 = analog_current;				//��������� ���������� �������� ��� - ���
					led_print(analog_current);
					encoder();
				}
				break;
				
			//����� ���������� 0-10 �����
			case 5: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 5)
				{
					OCR2 = analog_voltage;				//��������� ���������� �������� ��� - ����������
					led_print(analog_voltage*100/255);
					encoder();
				}
				break;	
		}
    }
}

  