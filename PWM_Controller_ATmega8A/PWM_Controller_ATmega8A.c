
/*
 * PWM_Controller_ATmega8A.c
 * Created: 09.04.2021 7:44:38
 * Author: MikhnovetsA
 * Программа для прибора, выдающего ШИМ сигнал, 0-10 Вольт и 4-20 мА. Управление скважностью либо энкодером, либо от аналогового входа 0-10 Вольт. Индикация на четырёх семисегментных индикаторах.
 */ 

#define F_CPU 16000000			//Частота работы микроконтроллера

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ADC_in	6				//Пин АЦП

#define SegA	7				//		AAAAA		PORTD
#define SegB	1 				//	   F	 B		PORTC
#define SegC	3 				//	   F	 B		PORTC
#define SegD	3 				//		GGGGG		PORTD
#define SegE	4 				//	   E	 C		PORTD
#define SegF	0 				//	   E	 C		PORTB
#define SegG	2 				//		DDDDD		PORTC
#define DP		4 				//				DP	PORTC

#define Dig1	6				//Старший разряд	PORTD
#define Dig2	5				//Средний разряд	PORTD
#define Dig3	0				//Младший разряд	PORTC
#define Dig4	5				//Младший разряд	PORTB

#define But		2				//Пин для кнопки	PORTD

#define Encoder1	1			//Энкодер пин А		PORTD
#define Encoder2	0			//Энкодер пин В		PORTD

#define PWM_OC1A	1			//Пин ШИМ			PORTB
#define PWM_OC1B	2			//Пин ШИМ			PORTB
#define PWM_OC2		3			//Пин ШИМ			PORTB

uint8_t btn_state = 1;			//Переменная для запоминания режима

volatile uint8_t indi = 1;		//Для переключения анодов индикаторов 

uint8_t R1 = 0;
uint8_t R2 = 0; 
uint8_t R3 = 0;				
uint8_t R4 = 0;					//Номера в разрядах

volatile int16_t count = 0;		//Счётчик от энкодера

uint16_t ADC_val = 0;			//Промежуточное значение для вывода на дисплей

volatile uint8_t PWM5V = 0;		//Переменная для 5-тивольтового ШИМа
volatile uint8_t PWMGND = 0;	//Переменная для минусового ШИМа

volatile int8_t analog_current = 0;
volatile int16_t analog_voltage = 0;
/*
uint16_t map_(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max){	//Функция для ограничения значения вывода на экран (как в Ардуино)
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
	Проще говоря, надо нашу переменную (которую надо преобразовать) умножить на новый требуемый максимум и разделить на прежний максимум (если было 8 бит, то это 255).
}
*/

void seg_char(uint8_t seg){		//Функция для вывода символов
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

void led_print(uint16_t number){	//Функция для разбивки числа на четыре индикаторы
	R1 = number/1000;				//Разряд тысяч 
	R2 = number%1000/100;			//Разряд сотен 
	R3 = number%1000%100/10;		//Разряд десятков 
	R4 = number%1000%100%10;		//Разряд единиц 
}

void encoder(){
	static uint8_t new_state = 0;			//Новое состояние ножек энкодера
	static uint8_t old_state = 0;			//Старое состояние ножек энкодера (значения хранятся в младших битах)
	new_state = (PIND & 0b00000011);		//Читаем состояние битов, сдвигаем их в младшие разряды и присваиваем старому состоянию
	switch (new_state << 2 | old_state)
	{
		case 0x01:  count++;						//Энкодер вправо
		if (btn_state == 1) PWM5V++;				//Меняем скважность только в этом режиме
		else if(btn_state == 2) PWMGND++;			//Меняем скважность только в этом режиме
		else if(btn_state == 4) analog_current++;	//Меняем переменную для тока
		else if(btn_state == 5) analog_voltage++;	//Меняем переменную для напряжения
		break;
		
		case 0x04:  count--;						//Энкодер влево
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

ISR(TIMER0_OVF_vect){		//Прерывание при переполеннии Таймера 0 для Динамической индикации
	
	switch (indi){
		case 1: PORTD &= ~(_BV(Dig1)); PORTD |= _BV(Dig2); PORTB |= _BV(Dig4); PORTC |= _BV(Dig3); seg_char(R1); //Включается первый индикатор, остальные выключены. На индикатор выводится число из старшего разряда.
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

ISR(INT0_vect){				//Обработка нажатия кнопки на пине внешнего прерывания INT0
	btn_state++;
	if (btn_state > 5) btn_state = 1;
}

void setup(void)
{
	//Настройка портов для индикаторов
	DDRB |= _BV(SegF) | _BV(Dig4) | _BV(PWM_OC1A) | _BV(PWM_OC1B) | _BV(PWM_OC2);	//
	DDRC |= _BV(SegB) | _BV(SegC) | _BV(SegG) | _BV(DP) | _BV(Dig3);				//	Настройка на выход портов для индикаторов
	DDRD |= _BV(SegA) | _BV(SegD) | _BV(SegE) | _BV(Dig1) | _BV(Dig2);				//
	
	PORTB |= _BV(SegF) | _BV(Dig4);													//
	PORTC |= _BV(SegB) | _BV(SegC) | _BV(SegG) | _BV(DP) | _BV(Dig3);				//	Установка пинов в единицу для выключения сегментов, так как индикатор с общим анодом
	PORTD |= _BV(SegA) | _BV(SegD) | _BV(SegE) | _BV(Dig1) | _BV(Dig2);				//
	
	//Настройка портов для кнопки и энкодера
	DDRD &= ~(_BV(But) | _BV(Encoder2) | _BV(Encoder1));							//	Настройка на вход
	PORTD &= ~(_BV(But) | _BV(Encoder2) | _BV(Encoder1));							//	Подтягивающие резисторы выключены, так как есть на плате
	
	//Настройка внешних прерываний на INT0 для кнопки
	MCUCR |= (1 << ISC01);															//Прерывание срабатывает на спаде уровня
	GICR |= (1 << INT0);															//Включили прерывание на пине INT0
	
	//Настройка Таймера 0 для динамической индикации
	TCCR0 |= (1 << CS02);															//Предделитель на 256 (при 16 МГц 16000000/256/256 = 244)
	TCNT0 = 0;																		//Обнуляем счётный регистр
	TIMSK |= (1 << TOIE0);															//Прерывание при переполении
	
	//Настройка Таймера 1 для ШИМ
	TCCR1A |= (1 << WGM10);															//Настраиваем ШИМ (8 бит, с точной фазой)
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);										//Вывод ШИМ на пины ОС1А(РВ1) и OCR1B(PB2)
	TCCR1B |= (1 << CS10) | (1 << CS11);											//Предделитель 64 (частота ШИМ при 16 МГц - 488 Гц)
	TCNT1 = 0;																		//Обнуляем счётный регистр
	OCR1A = 0;	OCR1B = 0;															//Обнуляем регистры совпадения (значением в этих регистрах управляем скважностью)
		
	//Настройка Таймера 2
	TCCR2 |=  (1 << CS22) | (1 << CS20);											//Предделитель на 128 (от него зависит частота ШИМ на ОС2)
	TCCR2 |= (1 << WGM20) | (1 << WGM21);											//Режим Fast PWM
	TCCR2 |= (1 << COM21);															//На пине ОС2 неинверсный ШИМ
	TCNT2 = 0;																		//Обнуляем счётный регистр
		
	//Настройка АЦП
	//ADMUX |= (1 << REFS1) | (1 << REFS0);											//Выбираем источник опорного напряжения 5 Вольт
	ADMUX |= ((1 << MUX2) | (1 << MUX1));											//Канал АЦП номер 6
	ADMUX &= ~(1 << ADLAR);															//Правостороннее выравнивание
	ADCSRA |= (1 << ADEN);															//Включаем АЦП
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);							//Делитель 128, частота дискредитации 125 кГц при 16 МГц микроконтроллера
	ADCSRA |= (1 << ADFR);															//АЦП постоянно измеряет напряжение
}

int main(void)
{
	setup();								//Настройка портов и периферии
	
	sei();									//Разрешаем прерывания
	
    while(1)								//Цикл
    {	
		switch(btn_state)					//Режим работы зависит от нажатий кнопки, которая по внешнему прерыванию икрементирует переменную btn_state
		{
			//Режим управления ШИМом +5 Вольт от прибора
			case 1: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 1)
				{
					OCR1A = PWM5V;				//Пишем в регистр сравнения значение переменной энкодера
					led_print(PWM5V*100/255);	//Выводим на дисплей значение переменной энкодера
					encoder();
				}
				break;
				
			//Режим управления ШИМом, управляющим минусом. Минус нагрузки подключается к клемме ОС1В, минус источника питания к GND. Напряжение до 24 Вольт.
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
				
			//Режим управления любым ШИМом от внешнего аналогового сигнала 0-10 Вольт. (Надо повозиться ещё!!!....)
			case 3: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 3)
				{
					ADCSRA |= (1 << ADSC);			//Запускаем измерение
					if (ADCSRA & (1 << ADIF))		//...если вычисление АЦП готово
					{
						OCR1B = ADC;				//Приравниваем значение регистра измерения АЦП регистру сравнения таймера 1
						ADCSRA |= (1 <<ADIF);		//Готовим АЦП к следующему измерению (поднимаем флаг)
					}
					led_print(PWMGND*100/1023);
				}
				break;
				
			//Режим тока 0-20 или 4-20 мА
			case 4: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 4)
				{
					OCR2 = analog_current;				//Энкодером выставляем значение ШИМ - ток
					led_print(analog_current);
					encoder();
				}
				break;
				
			//Режим напряжения 0-10 Вольт
			case 5: 
				led_print(btn_state);
				_delay_ms(1000);
				while(btn_state == 5)
				{
					OCR2 = analog_voltage;				//Энкодером выставляем значение ШИМ - напряжение
					led_print(analog_voltage*100/255);
					encoder();
				}
				break;	
		}
    }
}

  