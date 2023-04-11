#pragma once
#include <avrlibdefs.h>
//#include <avrlibtypes.h>
#include <Arduino.h>
//#include <>


#define STATUS_REG 			SREG
#define Interrupt_Flag		SREG_I
#define Disable_Interrupt	cli();
#define Enable_Interrupt	sei();

//RTOS Config
#define RTOS_ISR  			TIMER2_COMP_vect
//#define	TaskQueueSize		5
#define MainTimerQueueSize	5

//extern void RunRTOS (void);


// #define F_CPU 16000000L
// //RTOS Запуск системного таймера
// inline void RunRTOS (void)
// {
// 	TCCR2 = 1<<WGM21|4<<CS20; 				// Freq = CK/64 - Установить режим и предделитель
// 	// Автосброс после достижения регистра сравнения
// 	TCNT2 = 0;								// Установить начальное значение счётчиков
// 	OCR2  = LO(TimerDivider); 				// Установить значение в регистр сравнения
// 	TIMSK = 0<<TOIE0|1<<OCF2|0<<TOIE0;		// Разрешаем прерывание RTOS - запуск ОС
// 
// 	sei();
// }
