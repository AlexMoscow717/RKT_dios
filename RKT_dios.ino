/*
    Name:       RKT_dios.ino
    Created:	13.01.2021 10:41:24
    Author:     DESKTOP-2T997ND\Alex Moscow
*/

#define DS18B20_MAIN_PIN 3
#define DS18B20_INFO_PIN 7
#define DS18B20_WATER_PIN 5
#define PIN_RELAY 2
#define PIN_BUZZER 14
#define CS_PIN_MY_SD_CARD 4
#define PIN_BTN_MAIN 4
#define PIN_DEBUG 15
#define PIN_WATER_VALVE 13
#define WATER_SUPPLY_DELTA 1

#define ON_OUT 1
#define OFF_OUT 0

#define CLK 8
#define DT 9
//#define SW 12

#define F_PRINT_ON_OFF_VALVE_CNTL 1
#define FLAG_CLICKED_BTN 2
#define FLAG_DELTA_INCREMENT 3
#define FLAG_ONE_USE_VALVE_ON 7

#define F_CHANGE_DELTA_UP 10
#define F_VALVE_ON_OFF 11
#define F_BUZZER_ON_OFF 14
#define F_BZR_TIMER_START_STOP 15
#define F_BZR_ALTERNATIVE 16
#define F_PRINT_GUI_MAIN_ONE_USE 17
#define F_PRINT_PICTOGRAM_ONE_USE_TRUE 18
#define F_PRINT_PICTOGRAM_ONE_USE_FALSE 19
#define F_LATCH_W_VALVE_1 20
#define F_LATCH_W_VALVE_2 21
#define F_LATCH_TEMP_CTRL_BZR_1 22
#define F_LATCH_TEMP_CTRL_BZR_2 23
#define F_ENABLE_TIMER_30_S 24
#define F_END_TIME_30_S 25
#define F_LATCH_ON_OFF_TIMER_15_S 26
#define F_LATCH_RESET_TIMER_15_S 27
#define F_LATCH_PRINT_POWER_SET 28
#define F_ACTIVATION_PROC_CONTROLL_TEMP 9
#define F_TEMP_OUT_OF_RANGE 6
#define F_TEMP_OUT_OF_RANGE_DOUBLE 12
#define F_SD_CARD_ON_OFF_LOGGING 13
#define EMERGENCY_TEMP_MAIN 95
#define TIME_TIMER_S 9

#define LOCK_PROC_BTN_MNG 4
#define LOCK_TEMP_FIXED_OUT_LCD 5
#define LOCK_CLICK_FIXED_TEMP_IN_MENU 8

#define TEMP_MAX_VALUE_ALARM_BUZZ 90
#define TEMP_MIN_VALUE_ALARM_BUZZ 50
#define TEMP_STEP_CHANGE_ALARM_BUZZ 2

#define POWER_LEVEL_MIN 40
#define POWER_LEVEL_MAX 230
// #define POWER_LEVEL_MIN 100
// #define POWER_LEVEL_MAX 3310
#define POWER_LEVEL_CHANGED_STEP 5

#define DELAY_MAX_VALUE_OPEN_VALVE 60
#define DELAY_MIN_VALUE_OPEN_VALVE 1
#define DELAY_LOW_STEP_CHANGE_OPEN_VALVE 1
#define DELAY_MID_STEP_CHANGE_OPEN_VALVE 5
#define DELAY_HIGH_STEP_CHANGE_OPEN_VALVE 15
#define NUMBER_OF_MODES 4
#define DELTA_MODE 3
#define MODE_SOUND 1
#define MODE_MUTE 0
#define MODE_START_STOP 2
#define LINES_DISPLAY 4
#define ITEMS_MENU 7
#define ZERO_ITEM_MENU 2



#define MAIN_LOCATION 0
#define MENU_LOCATION 1
#define TEMP_BUZZ_LOCATION 2
#define DELTA_LOCATION 3
#define VALVE_LOCATION 4
#define BUZZER_ON_OFF_LOCATION 5
#define TIME_DELAY_OPEN_VALVE_LOCATION 6
#define SET_POWER_SUPPLY_LEVEL_LOCATION 7
#define EXIT_MENU_LOCATION 8


#define BUFF_TX_SIZE 16
#define BUFF_RX_SIZE 10
#define USART_RX_INT_ON UCSR0B|=(1<<RXCIE0);
#define USART_RX_INT_OFF UCSR0B&=~(1<<RXCIE0);
#define USART_TX_INT_ON UCSR0B|=1<<TXCIE0;
#define USART_TX_INT_OFF UCSR0B&=~(1<<TXCIE0);
#define USART_UDRE_INT_ON  UCSR0B|=(1<<UDRIE0);
#define USART_UDRE_INT_OFF UCSR0B &=~(1<<UDRIE0);
#define baudrate 9600L
#define bauddivider (F_CPU/(16*baudrate)-1)
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)
#define MODE_VOL_SET 0
#define MODE_VOL_OUT 1
#define RESIST_TEN 16.7
 






// #include <SPI.h>
// #include <SD.h>
//#include <ArduRTOS.h>
#include <ArduRTOS_AL.h>
//#include <ArduRTOS_AL2.h>
#include <GyverButton.h>
#include <GyverFilters.h>
#include <GyverTimers.h>
#include <GyverTimer.h>
#include <microDS18B20.h>
#include <QuickIO.h>
#include <bitPack.h>
#include <GyverWDT.h>
#include <microLiquidCrystal_I2C.h>
#include <GyverEncoder.h>
#include <C:\Arduino\hardware\arduino\avr\libraries\EEPROM\src\EEPROM.h>
#include <GyverBME280.h>





float getTempMainFiltered = 0;
float getTempInfoFiltered = 0;
float getTempWaterFiltered = 0;
float boiling_point_of_alcohol = 0;

float* ptrTempMainFiltered = nullptr;
float* ptrTempInfoFiltered = nullptr;
float* ptrTempWaterFiltered = nullptr;
float* ptrBoiling_point_of_alcohol = nullptr;
float* ptrValueEnc = nullptr;

byte value_press_btn = 0;
uint8_t mode_work_pointer = 0;
uint8_t display_modes = 0;
float delta = 0.05;
float delta_set = 0.05;
float threshold = 0;
float pressure = 0;

int RAM_SIZE;

//byte mode_global = 0;


byte location = 0;
byte bzr_mng_adm = 0;
byte pointer_menu = ZERO_ITEM_MENU;
byte water_supply_alarm_temp = TEMP_MIN_VALUE_ALARM_BUZZ;
byte water_supply_alarm_temp_set = TEMP_MIN_VALUE_ALARM_BUZZ;
byte voltage_level = POWER_LEVEL_MIN;
byte voltage_level_set = POWER_LEVEL_MIN;
int power_level;
uint16_t delay_open_valve = 1;
uint16_t delay_open_valve_set = 1;
boolean valve_state = true;
boolean buzzer_state = true;
//String dataLogString = "";

volatile char usartBufferTX[BUFF_TX_SIZE];
volatile char usartBufferRX[BUFF_RX_SIZE];
//= "AT+VS=230#013$";
volatile byte buff_index_tx = 0;
volatile byte buff_index_rx = 0;
volatile byte flag_end_recive = 0;
volatile byte index_arr_reverse_fnc = 0;

int* ptr_power_level = &power_level;
byte* ptr_voltage_level_set = &voltage_level_set;



float valueEnc = 0;
// float valueEncTest = 22;
float current_temp_main_fixed = 0;

byte customCharDeltaL[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00100,
	0b01110,
	0b11111
};


byte customCharCelsium[8] = {
	0b01000,
	0b10100,
	0b10100,
	0b01000,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

byte customCharRelayStateON[8] = {
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

byte customCharRelayStateOFF_L[8] = {
	0b11111,
	0b10000,
	0b10000,
	0b10000,
	0b10000,
	0b10000,
	0b10000,
	0b11111
};

byte customCharRelayStateOFF_R[8] = {
	0b11111,
	0b00001,
	0b00001,
	0b00001,
	0b00001,
	0b00001,
	0b00001,
	0b11111
};

struct time_clock_main
{
	volatile unsigned char second;
	volatile unsigned char minute;
	volatile unsigned char hours;
// 	volatile unsigned char timer30m = 30;
	volatile unsigned char timer30s = TIME_TIMER_S;
};


const char name1[] PROGMEM = "Temp Open Water";  // Каждое имя в своей строке, символ переноса не нужен
const char name2[] PROGMEM = "Temp Delta";
const char name3[] PROGMEM = "Valve On/Off";
const char name4[] PROGMEM = "Buzzer On/Off";
const char name5[] PROGMEM = "Delay Open Valve";
const char name6[] PROGMEM = "Set Power Level";
const char name7[] PROGMEM = "Exit Menu";

const char* const names[] PROGMEM = 
{
	name1,
	name2,
	name3,
	name4,
	name5,
	name6,
	name7	
};

struct STRCommand
{
	// 	const char *supply_voltage = "AT+VI?#013$";
	const char* voltage_OUT = "AT+VO?$";
	//const char *voltage_set_out = "AT+VS?#013$";
	const char* voltage_SET = "AT+VS=$";
	//const char *out_check_state = "AT+ON?#013$";
	//const char *out_on_off = "AT+ON=$";
	//const char *memory_select = "AT+SM=$";
};

STRCommand commandREGdata;

STRCommand* ptrComRegData = &commandREGdata;

Queue QueueMain;

MicroDS18B20 sensorTempMain(DS18B20_MAIN_PIN);
MicroDS18B20 sensorTempInfo(DS18B20_INFO_PIN);
MicroDS18B20 sensorTempWater(DS18B20_WATER_PIN);
GyverBME280 barometr;
//Sd2Card card;

GButton btn_main_clk(PIN_BTN_MAIN);
// GButton btn_switch_modeR(PIN_BTN_SW_MODE_R);
// GButton btn_switch_modeL(PIN_BTN_SW_MODE_L);

Encoder encMain(CLK,DT,TYPE1);

GKalman filterTempMain(0.08,0.75);
GKalman filterTempInfo(0.08,0.75);
GKalman filterTempWater(0.08,0.75);

GTimer oneTimer(MS);

QuickIO digital_pin_relay = PIN_RELAY;
QuickIO digital_pin_debug = PIN_DEBUG;
QuickIO digital_pin_buzzer = PIN_BUZZER;
QuickIO digital_pin_water_valve = PIN_WATER_VALVE;

LiquidCrystal_I2C lcd(0x27, 20, 4);

BitPack <28> flags;

time_clock_main time_local;

struct time_clock_main* ptr_struct_time = &time_local;



ISR(TIMER2_A)
{
	TimerService();	
}

ISR(TIMER1_A)
{
	if (++ptr_struct_time->second == 60)
	{
		ptr_struct_time->second = 0;
		
		if (++ptr_struct_time->minute == 60)
		{
			ptr_struct_time->minute = 0;
			
			if (++ptr_struct_time->hours == 24)
			{
				ptr_struct_time->hours = 0;
			}
		}
	}
	if (flags.read(F_ENABLE_TIMER_30_S))
	{
		flags.clear(F_LATCH_RESET_TIMER_15_S);
		if (--ptr_struct_time->timer30s == 0)
		{
			flags.clear(F_ENABLE_TIMER_30_S);
			flags.set(F_END_TIME_30_S);
			ptr_struct_time->timer30s = TIME_TIMER_S;
			
			
			
// 			if (--t.timer30m == 0)
// 			{
// 				t.timer30m = 0;
// 			}
		}
	}
	
	
}

ISR(USART_UDRE_vect)
{
	buff_index_tx++;													// Увеличиваем индекс
	
	if((usartBufferTX[buff_index_tx] == '$')||(buff_index_tx == BUFF_TX_SIZE))							// Вывели весь буффер?
	{
		USART_UDRE_INT_OFF
		buff_index_tx = 0;
		USART_TX_INT_ON
		// Запрещаем прерывание по опустошению - передача закончена
	}
	else
	{
		UDR0 = usartBufferTX[buff_index_tx];
		usartBufferTX[buff_index_tx] = 0;							// Берем данные из буффера.
	}
	

}

ISR(USART_TX_vect)
{
	//USART_TX_INT_OFF
	buff_index_rx = 0;
	USART_RX_INT_ON
}

ISR(USART_RX_vect)
{
	
	usartBufferRX[buff_index_rx] = UDR0;
	
	if((usartBufferRX[buff_index_rx] == '\r')||(buff_index_rx == BUFF_RX_SIZE))
	{
		USART_RX_INT_OFF
		index_arr_reverse_fnc = buff_index_rx;
		flag_end_recive = 1;
		buff_index_rx = 0;

	}
	
	buff_index_rx++;
}


void setup()
{
	
	UART_init();
	digital_pin_relay.mode(OUTPUT);
	digital_pin_buzzer.mode(OUTPUT);
	digital_pin_debug.mode(OUTPUT);
	digital_pin_water_valve.mode(OUTPUT);
	
	digital_pin_relay.write(ON_OUT);
	digital_pin_water_valve.write(OFF_OUT);
	
	lcd.init();
	lcd.backlight();
	
	EEPROM.get(0,delta);
	EEPROM.get(0,delta_set);
	
	EEPROM.get(4,delay_open_valve);
	EEPROM.get(6,delay_open_valve_set);
	
	EEPROM.get(8,water_supply_alarm_temp);
	EEPROM.get(9,water_supply_alarm_temp_set);
	
	Timer1.setFrequency(1);
	Timer1.enableISR();
	Timer2.setFrequency(1000);
	Timer2.enableISR();
	
	flags.clearAll();
	
	sensorTempMain.setResolution(12);
	sensorTempInfo.setResolution(12);
	sensorTempWater.setResolution(12);
	barometr.setMode(FORCED_MODE);
	barometr.setPressOversampling(OVERSAMPLING_16);
	barometr.begin();
	
	btn_main_clk.setDebounce(20);        // настройка антидребезга (по умолчанию 80 мс)
	btn_main_clk.setTimeout(600);        // настройка таймаута на удержание (по умолчанию 500 мс)
	btn_main_clk.setClickTimeout(150);   // настройка таймаута между кликами (по умолчанию 300 мс)
	barometr.oneMeasurement();
	
	for (int i = 0; i < BUFF_TX_SIZE ; i++)
	{
		usartBufferTX[i] = 0;
	}
	
	for (int i = 0; i < BUFF_RX_SIZE ; i++)
	{
		usartBufferRX[i] = 0;
	}
	
	ptrTempWaterFiltered = &getTempWaterFiltered;
	ptrTempMainFiltered = &getTempMainFiltered;
	ptrTempInfoFiltered = &getTempInfoFiltered;
	ptrBoiling_point_of_alcohol = &boiling_point_of_alcohol;
	//ptrValueEnc = &valueEnc;
	//digital_pin_relay.write(ON_OUT);
	//Serial.begin(9600);
	
	//SDcardInitialization();
	
	//delay(2000);
// 	lcd.setCursor(0,0);
// 	lcd.print(F("             "));
	
	lcd.clear();
	
	InitRTOS();
	
	//SetTask(tLedOn);
	
	QueueMain.push_back(taskRequestTempMain);
	QueueMain.push_back(taskRequestTempInfo);
	QueueMain.push_back(taskRequestTempWater);
	
	//SetTask(taskEncTick);
	
	QueueMain.push_back(taskEncScanRotary);
	
	//SetTask(taskLcdPrint1);
	//SetTask(buttonClock);
	//SetTask(buttonMNGmodes);
	
	
	QueueMain.push_back(bzrMng);
	QueueMain.push_back(taskMainProcess);
	QueueMain.push_back(taskControllTemperature);
	QueueMain.push_back(pressureComputing);
	QueueMain.push_back(task_Ctrl_Water_Valve_System);
	//QueueMain.push_back(task_buzzer_mng);
	//QueueMain.push_back(FREE_RAM_TEST);
	
	//SetTask(taskDataLogging);
	
	QueueMain.push_back(printGUImainLocation);
	
	lcd.createChar(0, customCharDeltaL);
	lcd.createChar(1, customCharCelsium);
	lcd.createChar(2, customCharRelayStateOFF_L);
	lcd.createChar(3, customCharRelayStateOFF_R);
	lcd.createChar(4, customCharRelayStateON);
	powerCalculate();
	
}

void loop()
{
	for (;;)
		{
			//TaskManager();
			QueueMain.pop_front();
		}
}


void pressureComputing(void)
{
	if (!barometr.isMeasuring())
	{
		pressure = pressureToMmHg(barometr.readPressure());
		
		boiling_point_of_alcohol = (float)(0.038 * pressure) + 49.27;
		
		barometr.oneMeasurement();
	}
	
	
	SetTimerTask(pressureComputing,7000);
}

void taskRequestTempMain(void)
{
	
	sensorTempMain.requestTemp();
	
	SetTimerTask(taskGetTempMain,900);
		
}

void taskGetTempMain(void)
{
	
	//getTempMain1 = sensorTempMain.getTemp();
	
	//SetTimerTask(taskRequestGetTempMain1,900);
	getTempMainFiltered = filterTempMain.filtered(sensorTempMain.getTemp());
	QueueMain.push_back(taskRequestTempMain);
	
}

void taskRequestTempInfo(void)
{
	
	sensorTempInfo.requestTemp();
	
	SetTimerTask(taskGetTempInfo,900);
		
}

void taskGetTempInfo(void)
{
	
	getTempInfoFiltered = filterTempInfo.filtered(sensorTempInfo.getTemp());
	
	
	QueueMain.push_back(taskRequestTempInfo);
	
}

void taskRequestTempWater(void)
{
	
	sensorTempWater.requestTemp();
	
	SetTimerTask(taskGetTempWater,900);
	
}

void taskGetTempWater(void)
{
	
	getTempWaterFiltered = filterTempWater.filtered(sensorTempWater.getTemp());
	
	
	QueueMain.push_back(taskRequestTempWater);
	
}

void taskMainProcess(void)
{
	
	switch (location)
	{
				case MAIN_LOCATION :{
										//digital_pin_debug.write(ON_OUT);
										printGUImainLocation();
										//SetTimerTask(printGUImainLocation,500);
										//digital_pin_debug.write(OFF_OUT);
										buttonControllMainLoc();
										
									}
				
				break;


				case MENU_LOCATION :{
										printGUImenuLocation();
										buttonControllMenuLocation();
										
									}

				break;


				case TEMP_BUZZ_LOCATION :{
											printGUITempBuzzAlarmLocation();
											buttonControllTempBuzzAlarmLocation();
										 }

				break;


				case DELTA_LOCATION :{
										printGUIdeltaLocation();
										buttonControllDeltaLocation();
									 }

				break;


				case VALVE_LOCATION :{
										printGUIvalveOnOffLocation();
										buttonControllValveOnOffLocation();
									 }

				break;


				case BUZZER_ON_OFF_LOCATION :{
												printGUIBuzzerOnOffLocation();
												buttonControllBuzzerOnOffLocation();
											 }

				break;

				case TIME_DELAY_OPEN_VALVE_LOCATION :{
															printGUIDelayOpenValveLocation();
															buttonControllDelayOpenValveLocation();
													 }

				break;

				case SET_POWER_SUPPLY_LEVEL_LOCATION : 
														{
															Print_GUI_Power_Set_Location();
															//digital_pin_debug.write(ON_OUT);
															Button_Control_Power_Set_Location();
															//digital_pin_debug.write(OFF_OUT);
														}
				break;
														
				case EXIT_MENU_LOCATION : buttonControllExitMenu();

				break;				
					
	}
	
	
	QueueMain.push_back(taskMainProcess);
}

void buttonControllMainLoc(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		location = MENU_LOCATION;
		pointer_menu = ZERO_ITEM_MENU;
		lcd.clear();
		//lcd.clear();
		
	}
	
	if (btn_main_clk.isClick())
	{
		current_temp_main_fixed = getTempMainFiltered;
		//current_temp_main_fixed = valueEnc;
		flags.set(F_ACTIVATION_PROC_CONTROLL_TEMP);
		flags.set(LOCK_TEMP_FIXED_OUT_LCD);
		flags.clear(F_TEMP_OUT_OF_RANGE);
	}
}

void printGUImainLocation(void)
{
	
	//digital_pin_debug.write(ON_OUT);
	
	static float tempMain = *ptrTempMainFiltered;
	static float tempInfo = *ptrTempInfoFiltered;
	static float tempWater = *ptrTempWaterFiltered;
	static float tempBoilAlc = *ptrBoiling_point_of_alcohol;
	//static float tempValueEnc = *ptrValueEnc;
	static byte hour,minute,second = 0;
	
	
	
	if (!flags.read(F_PRINT_GUI_MAIN_ONE_USE))
	{
		
		hour = ptr_struct_time->hours;
		minute = ptr_struct_time->minute;
		second = ptr_struct_time->second;
		
		flags.set(F_PRINT_GUI_MAIN_ONE_USE);
		lcd.setCursor(0,1);
		//digital_pin_debug.write(ON_OUT);
		lcd.print(F("K:"));
		//digital_pin_debug.write(OFF_OUT);
		lcd.print(*ptrTempMainFiltered);
		lcd.setCursor(7,1);
		lcd.write((uint8_t)1);
		//lcd.print(F("C"));
		
		lcd.setCursor(0,2);
		lcd.print(F("T:"));
		lcd.print(*ptrTempInfoFiltered);
		lcd.setCursor(7,2);
		lcd.write((uint8_t)1);
		//lcd.print(F("C"));
		
		lcd.setCursor(0,3);
		lcd.print(F("W:"));
		lcd.print(*ptrTempWaterFiltered);
		lcd.setCursor(7,3);
		lcd.write((uint8_t)1);
		
		lcd.setCursor(13,3);
		lcd.print(F("t"));
		
		lcd.write((uint8_t)0 );
		//lcd.print(F(" "));
		
		
		lcd.print(delta);
		lcd.write((uint8_t)1);
		//lcd.print(F("C"));
		
// 	lcd.setCursor(14,2);
// 	lcd.print(*ptrValueEnc);
// 	lcd.setCursor(19,2);
// 	lcd.write((uint8_t)1);
	
	lcd.setCursor(0,0);
	lcd.print(F("S:"));
	lcd.print(*ptrBoiling_point_of_alcohol);
	lcd.setCursor(7,0);
	lcd.write((uint8_t)1);
	
	lcd.setCursor(12,0);
	lcd.print( 0      );

	if (time_local.hours > 9)
	{
		
			lcd.setCursor(12,0);
			lcd.print(ptr_struct_time->hours);
		
		
	}else
	{
		
			lcd.setCursor(13,0);
			lcd.print(ptr_struct_time->hours);
		
		
	}

	 	lcd.print(':'     );
	 	lcd.print( 0      );

	if (time_local.minute > 9)
	{
		
			lcd.setCursor(15,0);
			lcd.print(ptr_struct_time->minute);
			
		
		
	}else
	{
			lcd.setCursor(16,0);
			lcd.print(ptr_struct_time->minute);
		
		
		
	}

		lcd.print(':'     );
		lcd.print( 0      );

	if (time_local.second > 9)
	{
		
			lcd.setCursor(18,0);
			lcd.print(ptr_struct_time->second);
		
		
		
	}else
	{
		
			lcd.setCursor(19,0);
			lcd.print(ptr_struct_time->second);
	
		
		
	}
	
	if (digital_pin_relay.read())
	{
		lcd.setCursor(9,0);
		lcd.write((uint8_t)4);
		lcd.write((uint8_t)4);
	}
	else
	{
		lcd.setCursor(9,0);
		lcd.write((uint8_t)2);
		lcd.write((uint8_t)3);
		
	}
	
	
   }
   
    if (tempBoilAlc != *ptrBoiling_point_of_alcohol)
    {
	    lcd.setCursor(2,0);
	    lcd.print(*ptrBoiling_point_of_alcohol);
	    tempBoilAlc = *ptrBoiling_point_of_alcohol;
    }
	
	if (tempMain != *ptrTempMainFiltered)
	{
		lcd.setCursor(2,1);
		lcd.print(*ptrTempMainFiltered);
		tempMain = *ptrTempMainFiltered;
	}
	
	if (tempInfo != *ptrTempInfoFiltered)
	{
		lcd.setCursor(2,2);
		lcd.print(*ptrTempInfoFiltered);
		tempInfo = *ptrTempInfoFiltered;
	}
	
	if (tempWater != *ptrTempWaterFiltered)
	{
		lcd.setCursor(2,3);
		lcd.print(*ptrTempWaterFiltered);
		tempWater = *ptrTempWaterFiltered;
	}
	
// 	if (tempValueEnc != *ptrValueEnc)
// 	{
// 		lcd.setCursor(14,2);
// 		lcd.print(*ptrValueEnc);
// 		tempValueEnc = *ptrValueEnc;
// 	}
	
	
	
	
	
	if (digital_pin_relay.read())
	{
		if (!flags.read(F_PRINT_PICTOGRAM_ONE_USE_TRUE))
		{
			lcd.setCursor(9,0);
			lcd.write((uint8_t)4);
			lcd.write((uint8_t)4);
			flags.set(F_PRINT_PICTOGRAM_ONE_USE_TRUE);
			flags.clear(F_PRINT_PICTOGRAM_ONE_USE_FALSE);
		}
		
	}
	 
	if(!digital_pin_relay.read())
	{
		if (!flags.read(F_PRINT_PICTOGRAM_ONE_USE_FALSE))
		{
			lcd.setCursor(9,0);
			lcd.write((uint8_t)2);
			lcd.write((uint8_t)3);
			flags.set(F_PRINT_PICTOGRAM_ONE_USE_FALSE);
			flags.clear(F_PRINT_PICTOGRAM_ONE_USE_TRUE);
		}
		
		
	}
	
	
// 	lcd.setCursor(13,2);
// 	lcd.print(pressure);
	
	
	if (flags.read(LOCK_TEMP_FIXED_OUT_LCD))
	{
		lcd.setCursor(14,1);
		lcd.print(current_temp_main_fixed);
		lcd.write((uint8_t)1);
		//lcd.print(F("C"));
	}else
	{
		lcd.setCursor(13,1);
		lcd.print(F("      "));
	}
	
	
// 	lcd.setCursor(12,0);
// 	lcd.print( 0      );



	if (time_local.hours > 9)
	{
		if (hour != ptr_struct_time->hours)
		{
			 	
			lcd.setCursor(12,0);
			lcd.print(ptr_struct_time->hours);
			hour = ptr_struct_time->hours;
		}
		
	}else
	{
		if (hour != ptr_struct_time->hours)
		{
			lcd.setCursor(12,0);
			lcd.print(0);
			lcd.setCursor(13,0);
			lcd.print(ptr_struct_time->hours);
			hour = ptr_struct_time->hours;
		}
		
	}

// 	lcd.print(':'     );
// 	lcd.print( 0      );

	if (time_local.minute > 9)
	{
		if (minute != ptr_struct_time->minute)
		{
			
			lcd.setCursor(15,0);
			lcd.print(ptr_struct_time->minute);
			minute = ptr_struct_time->minute;
		}
		
	}else
	{
		if (minute != ptr_struct_time->minute)
		{
			lcd.setCursor(15,0);
			lcd.print(0);
			lcd.setCursor(16,0);
			lcd.print(ptr_struct_time->minute);
			minute = ptr_struct_time->minute;
		}
		
		
	}

// 	lcd.print(':'     );
// 	lcd.print( 0      );

	if (time_local.second > 9)
	{
		if (second != ptr_struct_time->second)
		{
			
			lcd.setCursor(18,0);
			lcd.print(ptr_struct_time->second);
			second = ptr_struct_time->second;
		}
		
		
	}else
	{
		if (second != ptr_struct_time->second)
		{
			lcd.setCursor(18,0);
			lcd.print(0);
			lcd.setCursor(19,0);
			lcd.print(ptr_struct_time->second);
			second = ptr_struct_time->second;
		}
		
		
	}
	
// 	lcd.setCursor(9,1);
// 	lcd.print(RAM_SIZE);
	
	
	//SetTimerTask(printGUImainLocation,200);
	//digital_pin_debug.write(OFF_OUT);
}

void buttonControllMenuLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		location = pointer_menu;
		lcd.clear();
	}
	
	if (btn_main_clk.isClick())
	{
		//pointer_menu++;
		
		//constrain(pointer_menu,2,7);
		
		if (++pointer_menu > ITEMS_MENU +1)
		{
			pointer_menu = ZERO_ITEM_MENU;
		}
	}
}

void printGUImenuLocation(void)
{
	static byte screen_nuber = 0;
	static byte last_screen = 0;
	static byte pointer_menu_local = 0;
	
	pointer_menu_local = (pointer_menu - ZERO_ITEM_MENU);
	
	screen_nuber = pointer_menu_local / LINES_DISPLAY;
	
	if (last_screen != screen_nuber)
	{
		lcd.clear();
	}
	
	last_screen = screen_nuber;
	
	for (byte i = 0; i < LINES_DISPLAY; i++)
	{
		lcd.setCursor(0,i);
		
		if (pointer_menu_local == LINES_DISPLAY * screen_nuber + i)
		{
			
			lcd.setCursor(0,i);
			lcd.print(F(">"));
			
		}else
		{
			lcd.setCursor(0,i);
			lcd.print(F(" "));

		}

		if (LINES_DISPLAY * screen_nuber + i == ITEMS_MENU) break;
		
		printFromPGM(&names[LINES_DISPLAY * screen_nuber + i]);
		
	}
	
}

void buttonControllExitMenu(void)
{
		btn_main_clk.resetStates();
		location = MAIN_LOCATION;
		lcd.clear();
		flags.clear(F_PRINT_GUI_MAIN_ONE_USE);
		
// 		btn_main_clk.tick();
// 		
// 		if (btn_main_clk.isHold())
// 		{
// 			btn_main_clk.resetStates();
// 			location = MAIN_LOCATION;
// 			lcd.clear();
// 			//lcd.clear();
// 			
// 		}
	
}

void buttonControllDeltaLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isClick())
	{
		if (delta_set >= 0.5)
		{
			delta_set = 0.05;
		}else delta_set += 0.05;
	}
	
	
	if (btn_main_clk.isHolded())
	{
		
		location = MENU_LOCATION;
		
		if (delta_set > delta)
		{
			flags.set(F_CHANGE_DELTA_UP);
		}
			
		delta = delta_set;
		EEPROM.put(0,delta);
		lcd.clear();
		btn_main_clk.resetStates();
		
	}
}

void printGUIdeltaLocation(void)
{
	lcd.setCursor(5,2);
	lcd.print(F("t"));
	
	lcd.write((uint8_t)0 );
	lcd.print(F(" "));
	
	lcd.print(delta_set);
	lcd.write((uint8_t)1);
	lcd.print(F("C"));
}


void Button_Control_Power_Set_Location(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isClick())
	{
		if (voltage_level_set >= POWER_LEVEL_MAX)
		{
			voltage_level_set = POWER_LEVEL_MIN;
		}else
		{
			voltage_level_set += POWER_LEVEL_CHANGED_STEP;
		}
		
		powerCalculate();
	}
	
	if (btn_main_clk.isHolded())
	{
		location = MENU_LOCATION;
		
		
		voltage_level = voltage_level_set;
		
		QueueMain.push_back(task_Set_Voltage_RMVK);
		
		lcd.clear();
		flags.clear(F_LATCH_PRINT_POWER_SET);
		btn_main_clk.resetStates();
	}
}

void Print_GUI_Power_Set_Location(void)
{
// 	int* ptr_power_level = &power_level;
// 	byte* ptr_voltage_level_set = &voltage_level_set;
	static int local_power_level = *ptr_power_level;
	static byte local_voltage_level_set = *ptr_voltage_level_set;
	
	if (!flags.read(F_LATCH_PRINT_POWER_SET))
	{
		lcd.setCursor(2,0);
		lcd.print(F("Power   Level "));
		
		if (*ptr_power_level < 1000)
		{
			lcd.print(*ptr_power_level);
			lcd.print("  ");
		}else
		{
			lcd.print(*ptr_power_level);
		}
		
		lcd.setCursor(2,2);
		lcd.print(F("Voltage Level "));
		
		if (voltage_level_set < 100)
		{
			lcd.print(voltage_level_set);
			lcd.print(" ");
		}else
		{
			lcd.print(voltage_level_set);
		}
		
		flags.set(F_LATCH_PRINT_POWER_SET);
	}
	
	
// 	lcd.setCursor(2,0);
// 	lcd.print(F("Power   Level "));
	
	if (*ptr_power_level != local_power_level)
	{
		if (*ptr_power_level < 1000)
		{
			lcd.setCursor(16,0);
			lcd.print(*ptr_power_level);
			lcd.setCursor(19,0);
			lcd.print(" ");
		}else
		{
			lcd.setCursor(16,0);
			lcd.print(*ptr_power_level);
		}
		
		local_power_level = *ptr_power_level;
	}
	
	if (*ptr_voltage_level_set != local_voltage_level_set)
	{
		if (*ptr_voltage_level_set < 100)
		{
			lcd.setCursor(16,2);
			lcd.print(*ptr_voltage_level_set);
			lcd.setCursor(18,2);
			lcd.print(" ");
		}else
		{
			lcd.setCursor(16,2);
			lcd.print(*ptr_voltage_level_set);
		}
		
		local_voltage_level_set = *ptr_voltage_level_set;
	}
	
// 	lcd.setCursor(2,2);
// 	lcd.print(F("Voltage Level "));
// 	
// 	if (voltage_level_set < 100)
// 	{
// 		lcd.print(voltage_level_set);
// 		lcd.print(" ");
// 	}else
// 	{
// 		lcd.print(voltage_level_set);
// 	}

	
	//lcd.print(F("C"));
}


void buttonControllValveOnOffLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isClick())
	{
		valve_state = !valve_state;
	}
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		location = MENU_LOCATION;
		lcd.clear();
	}
}

void printGUIvalveOnOffLocation(void)
{
	lcd.setCursor(0,2);
	lcd.print(F("Valve control  "));
	
	if (valve_state)
	{
		if (!flags.read(F_PRINT_ON_OFF_VALVE_CNTL))
		{
			lcd.clear();
			
			flags.set(F_PRINT_ON_OFF_VALVE_CNTL);
		}
		lcd.setCursor(15,2);
		lcd.print(F("ON"));
	}else
	{
		if (flags.read(F_PRINT_ON_OFF_VALVE_CNTL))
		{
			lcd.clear();
			
			flags.clear(F_PRINT_ON_OFF_VALVE_CNTL);
		}
		lcd.setCursor(15,2);
		lcd.print(F("OFF"));
	}
}

void buttonControllBuzzerOnOffLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isClick())
	{
		buzzer_state = !buzzer_state;
	}
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		location = MENU_LOCATION;
		lcd.clear();
	}
}

void printGUIBuzzerOnOffLocation(void)
{
	lcd.setCursor(0,2);
	lcd.print(F("Buzzer control  "));
	
	if (buzzer_state)
	{
		if (!flags.read(F_PRINT_ON_OFF_VALVE_CNTL))
		{
			lcd.clear();
			
			flags.set(F_PRINT_ON_OFF_VALVE_CNTL);
		}
		lcd.setCursor(15,2);
		lcd.print(F("ON"));
	}else
	{
		if (flags.read(F_PRINT_ON_OFF_VALVE_CNTL))
		{
			lcd.clear();
			
			flags.clear(F_PRINT_ON_OFF_VALVE_CNTL);
		}
		lcd.setCursor(15,2);
		lcd.print(F("OFF"));
	}
}

void buttonControllTempBuzzAlarmLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isClick())
	{
		if (water_supply_alarm_temp_set >= TEMP_MAX_VALUE_ALARM_BUZZ)
		{
			water_supply_alarm_temp_set = TEMP_MIN_VALUE_ALARM_BUZZ;
		}else water_supply_alarm_temp_set += TEMP_STEP_CHANGE_ALARM_BUZZ;
	}
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		
		water_supply_alarm_temp = water_supply_alarm_temp_set;
		
		EEPROM.put(8,water_supply_alarm_temp);
		EEPROM.put(9,water_supply_alarm_temp_set);
		location = MENU_LOCATION;
		lcd.clear();
	}
	
}

void printGUITempBuzzAlarmLocation(void)
{
	lcd.setCursor(0,2);
	lcd.print(F("Alarm Water "));
	
	//lcd.setCursor(14,2);
	lcd.print(F("t"));

	//lcd.write((uint8_t)0 );
	lcd.print(F(" "));

	lcd.print(water_supply_alarm_temp_set);
	lcd.write((uint8_t)1);
	lcd.print(F("C"));
}

void buttonControllDelayOpenValveLocation(void)
{
	btn_main_clk.tick();
	
	if (btn_main_clk.isTriple())
	{
 
		delay_open_valve_set += DELAY_HIGH_STEP_CHANGE_OPEN_VALVE;
	}
	
	//btn_main_clk.tick();
	
	if (btn_main_clk.isDouble())
	{

		delay_open_valve_set += DELAY_MID_STEP_CHANGE_OPEN_VALVE;
	}
	
	//btn_main_clk.tick();
	
	if (btn_main_clk.isSingle())
	{
 
		delay_open_valve_set += DELAY_LOW_STEP_CHANGE_OPEN_VALVE;
	}
	
	
	if (delay_open_valve_set >= DELAY_MAX_VALUE_OPEN_VALVE)
	{
		delay_open_valve_set = DELAY_MIN_VALUE_OPEN_VALVE;
	}
	
	
	if (btn_main_clk.isHold())
	{
		btn_main_clk.resetStates();
		delay_open_valve = (delay_open_valve_set * 1000);
		EEPROM.put(4,delay_open_valve);
		EEPROM.put(6,delay_open_valve_set);
		location = MENU_LOCATION;
		lcd.clear();
	}
	
}

void printGUIDelayOpenValveLocation(void)
{
	lcd.setCursor(0,2);
	lcd.print(F("Delay ON Valve "));
	
	lcd.print(delay_open_valve_set);

	if (delay_open_valve_set < 10)
	{
		lcd.print(F(" s  "));
	}else lcd.print(F(" s"));
	
}

void taskControllTemperature(void)
{
// 	if (getTempInfoFiltered > EMERGENCY_TEMP_MAIN)
// 	{
// 		
// 	}
	
	
	
	if (flags.read(F_ACTIVATION_PROC_CONTROLL_TEMP))
	{
		threshold = current_temp_main_fixed + delta;
		
		//if (valueEnc <= current_temp_main_fixed)		//getTempMainFiltered
		if (getTempMainFiltered <= current_temp_main_fixed)
		{
			
			if (flags.read(F_TEMP_OUT_OF_RANGE))
			{
				//flags.clear(F_TEMP_OUT_OF_RANGE);
				
				if (!flags.read(FLAG_ONE_USE_VALVE_ON))
				{
					SetTimerTask(valveOn,delay_open_valve);
					
					flags.set(FLAG_ONE_USE_VALVE_ON);
				}
			} 
			else
			{
				flags.set(F_VALVE_ON_OFF);
				//flags.set(F_BUZZER_ON_OFF);
				
			}
			
			if (!flags.read(F_LATCH_TEMP_CTRL_BZR_1))
			{
				flags.clear(F_BUZZER_ON_OFF);
				
				flags.set(F_LATCH_TEMP_CTRL_BZR_1);
				flags.clear(F_LATCH_TEMP_CTRL_BZR_2);
			}
			
		} 
		else
		{
			//if (valueEnc >= threshold)
			if (getTempMainFiltered >= threshold)
			{
				flags.clear(F_VALVE_ON_OFF);
				
				if (!flags.read(F_LATCH_TEMP_CTRL_BZR_2))
				{
					flags.set(F_BUZZER_ON_OFF);
					
					flags.set(F_LATCH_TEMP_CTRL_BZR_2);
					flags.clear(F_LATCH_TEMP_CTRL_BZR_1);
				}
				
				
				flags.set(F_TEMP_OUT_OF_RANGE);
				
				if (flags.read(F_CHANGE_DELTA_UP))
				{
					flags.clear(F_CHANGE_DELTA_UP);
				}
				
			} 
			else
			{
				if (flags.read(F_TEMP_OUT_OF_RANGE))
				{
					if (flags.read(F_CHANGE_DELTA_UP))
					{
						
						flags.clear(F_CHANGE_DELTA_UP);
						
						flags.set(F_VALVE_ON_OFF);
						
						
					}
				} 
				
				//flags.clear(F_BUZZER_ON_OFF);
				
				if (!flags.read(F_LATCH_TEMP_CTRL_BZR_1))
				{
					flags.clear(F_BUZZER_ON_OFF);
					
					flags.set(F_LATCH_TEMP_CTRL_BZR_1);
					flags.clear(F_LATCH_TEMP_CTRL_BZR_2);
				}
			}
			
			
		}
		
		
		
		if (valve_state)
		{
			if (flags.read(F_VALVE_ON_OFF))
			{
				digital_pin_relay.write(ON_OUT);
				
			} 
			else
			{
				digital_pin_relay.write(OFF_OUT);
			}
		} 
		else
		{
			flags.clear(F_VALVE_ON_OFF);
			
			
			if (flags.read(F_VALVE_ON_OFF))
			{
				digital_pin_relay.write(ON_OUT);
			}
			else
			{
				digital_pin_relay.write(OFF_OUT);
			}
		}
		
		
// 		if (buzzer_state)
// 				{
// 					if (flags.read(F_BUZZER_ON_OFF))
// 					{
// 						//digital_pin_buzzer.write(ON_OUT);
// 						flags.set(F_BZR_TIMER_START_STOP);
// 						flags.set(F_BZR_ALTERNATIVE);
// 						
// 					}
// 					else
// 					{
// 						//digital_pin_buzzer.write(OFF_OUT);
// 						flags.clear(F_BZR_TIMER_START_STOP);
// 						
// 					}
// 				}
// 				else
// 				{
// 					flags.clear(F_BUZZER_ON_OFF);
// 					flags.clear(F_BZR_TIMER_START_STOP);
// 					
// 					
// 					if (flags.read(F_BUZZER_ON_OFF))
// 					{
// 						digital_pin_buzzer.write(ON_OUT);
// 					}
// 					else
// 					{
// 						digital_pin_buzzer.write(OFF_OUT);
// 					}
// 				}
		
		
		
	}
		
		
		
	
	QueueMain.push_back(taskControllTemperature);
}

void task_buzzer_mng(void)
{
	if (buzzer_state)
	{
		if (flags.read(F_BUZZER_ON_OFF))
		{
			//digital_pin_buzzer.write(ON_OUT);
			flags.set(F_BZR_TIMER_START_STOP);
			flags.set(F_BZR_ALTERNATIVE);
			
			if (!flags.read(F_LATCH_ON_OFF_TIMER_15_S))
			{
				flags.set(F_LATCH_ON_OFF_TIMER_15_S);
				flags.set(F_ENABLE_TIMER_30_S);
				
			}
			
			
		}

			
			if (flags.read(F_END_TIME_30_S))
			{
				flags.clear(F_BUZZER_ON_OFF);
				flags.clear(F_BZR_TIMER_START_STOP);
				flags.clear(F_END_TIME_30_S);
				
				
			}
			
			if (!flags.read(F_BUZZER_ON_OFF))
			{
				flags.clear(F_BZR_TIMER_START_STOP);
				flags.clear(F_LATCH_ON_OFF_TIMER_15_S);
				
				if (!flags.read(F_LATCH_RESET_TIMER_15_S))
				{
					flags.set(F_LATCH_RESET_TIMER_15_S);
					ptr_struct_time->timer30s = TIME_TIMER_S;
				}
				
			}

					
	}
	else
	{
		flags.clear(F_BUZZER_ON_OFF);
		flags.clear(F_BZR_TIMER_START_STOP);
		
		
		if (flags.read(F_BUZZER_ON_OFF))
		{
			digital_pin_buzzer.write(ON_OUT);
		}
		else
		{
			digital_pin_buzzer.write(OFF_OUT);
		}
	}
	
	QueueMain.push_back(task_buzzer_mng);
}

void task_Ctrl_Water_Valve_System(void)
{
	
	//if (*ptrTempInfoFiltered >= water_supply_alarm_temp)
	if (valueEnc >= water_supply_alarm_temp)
	{
		digital_pin_water_valve.write(ON_OUT);
		if (!flags.read(F_LATCH_W_VALVE_1))
		{
			flags.set(F_BUZZER_ON_OFF);
			
			flags.set(F_LATCH_W_VALVE_1);
			flags.clear(F_LATCH_W_VALVE_2);
		}
		
	//}else if (*ptrTempInfoFiltered <= water_supply_alarm_temp - WATER_SUPPLY_DELTA)
	}else if (valueEnc <= water_supply_alarm_temp - WATER_SUPPLY_DELTA)
	{
		digital_pin_water_valve.write(OFF_OUT);
		if (!flags.read(F_LATCH_W_VALVE_2))
		{
			flags.clear(F_BUZZER_ON_OFF);
			
			flags.set(F_LATCH_W_VALVE_2);
			flags.clear(F_LATCH_W_VALVE_1);
		}
		
	}
	
	QueueMain.push_back(task_Ctrl_Water_Valve_System);
}

void printFromPGM(int charMap)
{
	uint16_t ptr = pgm_read_word(charMap);
	    // получаем адрес из таблицы ссылок
	while (pgm_read_byte(ptr) != NULL)
	{      // всю строку до нулевого символа
		lcd.print(char(pgm_read_byte(ptr)));    // выводим в монитор или куда нам надо
		ptr++;                                  // следующий символ
	}
}

void valveOn(void)
{
	
	flags.set(F_VALVE_ON_OFF);
	//flags.clear(F_TEMP_OUT_OF_RANGE);
	flags.clear(FLAG_ONE_USE_VALVE_ON);
	
}

void taskEncTick(void)
{
	encMain.tick();
	
	QueueMain.push_back(taskEncTick);
}

void taskEncScanRotary(void)
{
	encMain.tick();
// 	if (encMain.isTurn())
// 	{
		// 		lcd.setCursor(0,0);
		// 		lcd.print("   ");
		//encMain.resetStates();
		//lcd.clear();
		
		//encMain.tick();
		if (encMain.isRight())
		{
			//Serial.print("R");
			 valueEnc += 0.10;
			 //pressure += 1;	
		}						// если был поворот направо, увеличиваем на 1
		//encMain.tick();
		if (encMain.isLeft())
		{
			//Serial.print("L");
			valueEnc -=0.10;
			//pressure -= 1;
			
		} 
		
		//Serial.print("turn");
		//valueEncTest 
	//}
	
	//encval = valueEnc;
	//SetTimerTask(taskEncScanRotary,1);
	QueueMain.push_back(taskEncScanRotary);
}

void bzrMng(void)
{
	
	if ((flags.read(F_BZR_TIMER_START_STOP))||(flags.read(F_BZR_ALTERNATIVE)))
	{
	
	
	
			switch (bzr_mng_adm)
			{
				case 0 : {
					digital_pin_buzzer.write(ON_OUT);
					oneTimer.setTimeout(55);
					oneTimer.start();
					bzr_mng_adm = 1;
				}
				break;
		
				case 1 :{
					if (oneTimer.isReady())
					{
						digital_pin_buzzer.write(OFF_OUT);
						oneTimer.stop();
						oneTimer.setTimeout(75);
						oneTimer.start();
						bzr_mng_adm = 2;
					}
				}
				break;
		
				case 2 :{
					if(oneTimer.isReady())
					{
						digital_pin_buzzer.write(ON_OUT);
						oneTimer.stop();
						oneTimer.setTimeout(55);
						oneTimer.start();
						bzr_mng_adm = 3;
					}
				}
				break;
		
				case 3 :{
			
					if (oneTimer.isReady())
					{
						digital_pin_buzzer.write(OFF_OUT);
						oneTimer.stop();
						oneTimer.setTimeout(75);
						oneTimer.start();
						bzr_mng_adm = 4;
					}
				}
				break;
		
				case 4 :{
					if(oneTimer.isReady())
					{
						digital_pin_buzzer.write(ON_OUT);
						oneTimer.stop();
						oneTimer.setTimeout(55);
						oneTimer.start();
						bzr_mng_adm = 5;
					}
				}
				break;
		
				case 5 :{
					if (oneTimer.isReady())
					{
						digital_pin_buzzer.write(OFF_OUT);
						oneTimer.stop();
						oneTimer.setTimeout(650);
						oneTimer.start();
						
						bzr_mng_adm = 6;
				
						
					}
				}
				break;
				
				case 6 :{
					
					if (oneTimer.isReady())
					{
						bzr_mng_adm = 0;
						oneTimer.stop();
					}
					
					
					
					if (!flags.read(F_BZR_TIMER_START_STOP))
					{
						flags.clear(F_BZR_ALTERNATIVE);
					}
				}
				break;
		
			}
			
	}
	
	QueueMain.push_back(bzrMng);
}



void Void(void)
{
	// void SDcardInitialization(void)
	// {
	//
	// 	if (!SD.begin())
	// 	{
	// 		lcd.setCursor(0,0);
	// 		lcd.print(F("No SD Card"));
	// 		flags.clear(F_SD_CARD_ON_OFF_LOGGING);
	// 	}
	// 	else
	// 	{
	// 		lcd.setCursor(0,0);
	// 		lcd.print(F("SD Card Ready"));
	//
	// 		flags.set(F_SD_CARD_ON_OFF_LOGGING);
	// 	}
	//
	// }


	// void taskDataLogging(void)
	// {
	//
	// 	if (flags.read(F_SD_CARD_ON_OFF_LOGGING))
	// 	{
	//
	// 		File log_file_data = SD.open("datalog.txt", FILE_WRITE);
	//
	// 		if (log_file_data)
	// 		{
	//
	//
	// 			log_file_data.print(time_local.hours);
	// 			log_file_data.print(":");
	// 			log_file_data.print(time_local.minute);
	// 			log_file_data.print(":");
	// 			log_file_data.print(time_local.second);
	// 			log_file_data.print("	");
	//
	// 			log_file_data.print(getTempMainFiltered);
	// 			log_file_data.print("	");
	//
	// 			log_file_data.print(getTempInfoFiltered);
	// 			log_file_data.print("	");
	//
	// 			log_file_data.print(getTempWaterFiltered);
	// 			log_file_data.print("	");
	//
	// 			log_file_data.print(boiling_point_of_alcohol);
	// 			log_file_data.print("	");
	//
	// 			if (digital_pin_relay.read() > 0)
	// 			{
	// 				log_file_data.println(digital_pin_relay.read() + 9);
	// 			}
	// 			else
	// 			{
	// 				log_file_data.println(digital_pin_relay.read());
	// 			}
	//
	// 			log_file_data.close();
	//
	// 		}else
	// 		{
	// 			flags.clear(F_SD_CARD_ON_OFF_LOGGING);
	// 		}
	//
	// 		SetTimerTask(taskDataLogging,2000);
	// 	}
	//
	// }

}

void FREE_RAM_TEST(void)
{
	RAM_SIZE = memoryFree();
	
	SetTimerTask(FREE_RAM_TEST,500);
}

// int freeRam () {
// 	extern int __heap_start, *__brkval;
// 	int v;
// 	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
// }

extern int __bss_end;
extern void *__brkval;
// Функция, возвращающая количество свободного ОЗУ
int memoryFree() {
	int freeValue;
	if ((int)__brkval == 0)
	freeValue = ((int)&freeValue) - ((int)&__bss_end);
	else
	freeValue = ((int)&freeValue) - ((int)__brkval);
	return freeValue;
}


void UART_init(void)
{
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B = 1<<RXEN0|1<<TXEN0|1<<RXCIE0|0<<TXCIE0;
	UCSR0C = 1<<UCSZ00|1<<UCSZ01;
	asm("sei");
}

unsigned int pow_int(unsigned int base,unsigned int exp)
{
	unsigned int result = 1;
	while (exp)
	{
		if (exp & 1)
		result *= base;
		exp >>= 1;
		base *= base;
	}
	
	return result;
}

inline void powerCalculate(void)
{
	//int power = 0;
	
	//power_level = pow_int(voltage_level_set,2) / RESIST_TEN;
	power_level = pow_int(voltage_level_set,2) >> 4;
	
	//return power;
}




inline void assemblyMessageToSend( STRCommand command_line, byte mode_transmitt = NULL, byte encoder_value = NULL)
{
	STRCommand* ptr_listing = &command_line;;
	byte index_arr = 0;
	//ptr_listing = &command_line;
	char* ptrTX = usartBufferTX;
	switch (mode_transmitt)
	{
		case MODE_VOL_SET :
		{
			while(*ptr_listing->voltage_SET != '$')
			{
				ptrTX[index_arr] = *ptr_listing->voltage_SET;
				
				ptr_listing->voltage_SET++;
				index_arr++;
			}
			
 			ptrTX[index_arr] = (encoder_value / 100) + '0';
			index_arr++;
			ptrTX[index_arr] = ((encoder_value % 100) / 10) + '0';
			index_arr++;
			ptrTX[index_arr] = (encoder_value % 10) + '0';
			index_arr++;
			ptrTX[index_arr] = '\r';
			index_arr++;
			ptrTX[index_arr] = '$';
	
			
		}
		break;
		
		case MODE_VOL_OUT :
		{
			while(*ptr_listing->voltage_OUT != '$')
			{
				ptrTX[index_arr] = *ptr_listing->voltage_OUT;
				index_arr++;
				*ptr_listing->voltage_OUT++;
			}
			
			ptrTX[index_arr] = '\r';
		}
		break;
	}

	
}




int reverseCharToNum(void)
{
	
	static unsigned int summary = 0;
	int po_w = 0;
	int index_arr = 0;
	if (flag_end_recive)
	{
		summary = 0;
		
		flag_end_recive = 0;
		index_arr_reverse_fnc--;

		po_w = index_arr_reverse_fnc;
		
		for (byte i = 0; i <= index_arr_reverse_fnc; i++)
		{
			summary += ((usartBufferRX[i] - '0') * pow_int(10,po_w));

			po_w--;
			if (po_w < 0 )
			{
				po_w = 0;
			}
		}
		
		
	}
	
	return summary;
	
}

void task_Set_Voltage_RMVK(void)
{
	//digital_pin_debug.write(ON_OUT);
	
	assemblyMessageToSend(commandREGdata,MODE_VOL_SET,voltage_level);
	//digital_pin_debug.write(OFF_OUT);
	cli();
	buff_index_tx = 0;
	UDR0 = usartBufferTX[0];
	USART_UDRE_INT_ON
	sei();
	
	
}

