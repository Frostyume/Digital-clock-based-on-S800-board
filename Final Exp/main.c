#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "ctype.h"
#include "string.h"
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "tm4c1294ncpdt.h"
#include "pwm.h"
#include "timer.h"
#include "eeprom.h"
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06


#define SYSTICK_FREQUENCY		20000000


void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void    S800_PWM_Init(void);
void		S800_Timer0_Init(void);
void 		SysTickInit(void);
void UARTStringPut(const char *cMessage);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void trim(char *str);
void boot(void);
void Key_scan(void);
void Display_Time(void);
void Display_Date(void);
void Display_Alarm(void);
void Ring(void);
void Display_Stopwatch(void);
void Set_With_Key(void);
void Set_Time(void);
void Set_Date(void);
void Set_Alarm(void);
void Set_Stopwatch(void);
void Set_With_UART(void);
void Print_Bitmap(void);


volatile uint8_t result;
uint8_t Key, usr_Key1, usr_Key2, lastKey, lastusr_Key1, lastusr_Key2;
uint8_t isPressUsrkey = 0;
uint8_t isReleaseUsrkey = 0;
int press_time, release_time, duration;
char usr_key_press_time[9];
char usr_key_release_time[9];
char usr_key_duration[9];
uint8_t Key_flag[8], Key_count[8];
uint8_t isShortPress[8], isLongPress[8], isRelease[8];
uint8_t mode_key, set_key, key_number[8];

uint16_t ms;
uint8_t clock10ms = 0;
uint8_t clock10ms_flag = 0;
uint8_t clock20ms = 0;
uint8_t clock20ms_flag = 0;
uint16_t clock100ms = 0;
uint8_t clock100ms_flag = 0;
uint16_t clock800ms = 0;
uint16_t clock1s = 0;
uint8_t clock1s_flag = 0;

uint16_t year = 23;
uint8_t month = 6;
uint8_t day = 11;
uint8_t year_H = 2;
uint8_t year_L = 3;
uint8_t month_H = 0;
uint8_t month_L = 6;
uint8_t day_H = 1;
uint8_t day_L = 1;

uint8_t hour = 8;
uint8_t minute = 0;
uint8_t second = 56;
uint8_t hour_H = 0;
uint8_t hour_L = 8;
uint8_t min_H = 0;
uint8_t min_L = 0;
uint8_t sec_H = 5;
uint8_t sec_L = 9;

uint8_t a_hour = 0;
uint8_t a_minute = 0;
uint8_t a_second = 0;
uint8_t a_hour_H = 0;
uint8_t a_hour_L = 0;
uint8_t a_min_H = 0;
uint8_t a_min_L = 0;
uint8_t a_sec_H = 0;
uint8_t a_sec_L = 0;


int t_min_H = 0;
int t_min_L = 0;
int t_sec_H = 0;
int t_sec_L = 0;
int t_ms_H = 0;
int t_ms_L = 0;


uint8_t isReverse = 0;
uint8_t seg_pos = 0;
uint8_t seg_blink = 1;
uint8_t seg_bit = 8;
uint16_t t = 10000;
uint8_t cnt = 0;
uint8_t isboot = 1;
uint8_t isSoftReset = 0;
uint8_t isSet = 0;
uint8_t set_flag;
uint8_t yes,cancel;
uint8_t isStartTiming = 0;
uint32_t BeepPeriod = 5000;
uint8_t Alarm_isEnable = 0;
uint8_t ringtone = 1;
uint8_t isRing = 0;
uint8_t SetStopwatch = 0;
uint32_t ui32SysClock;
uint32_t ui32EEPROMInit;

char numbers[10] = {'0','1','2','3','4','5','6','7','8','9'};
uint8_t StudentID[9] = {2,1,9,1,0,6,3,0};
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
uint8_t re_seg7[] = {0x3f,0x30,0x5b,0x79,0x74,0x69,0x6f,0x38,0x7f,0x7d};
uint8_t Serial[8];
int seg_data[8];
char State_Get[8];
char RxBuf[256]; 
uint8_t RxEndFlag = 0;
uint32_t pui32Data[16];
uint32_t pui32Read[16];

static const unsigned char Bitmap_Bytes[128] = {
    0x00, 0x08, 0x10, 0x20, 0x01, 0x00, 0x00, 0x40, 
    0x00, 0x08, 0x10, 0x20, 0x00, 0x80, 0x00, 0x41, 
    0x7c, 0x08, 0x3c, 0x20, 0x1f, 0xfc, 0x7c, 0x41, 
    0x44, 0x08, 0x20, 0x20, 0x10, 0x04, 0x00, 0x41, 
    0x45, 0xfe, 0x41, 0xfc, 0x10, 0x04, 0x01, 0xfd, 
    0x44, 0x08, 0xbd, 0x24, 0x10, 0x04, 0x00, 0x45, 
    0x44, 0x08, 0x11, 0x24, 0x1f, 0xfc, 0xfe, 0x45, 
    0x7c, 0x08, 0x11, 0x24, 0x10, 0x00, 0x20, 0x45, 
    0x44, 0x88, 0xfd, 0x24, 0x10, 0x00, 0x20, 0x45, 
    0x44, 0x48, 0x11, 0xfc, 0x10, 0x00, 0x20, 0x85, 
    0x44, 0x48, 0x11, 0x24, 0x17, 0xfc, 0x48, 0x84, 
    0x44, 0x08, 0x10, 0x20, 0x24, 0x04, 0x44, 0x84, 
    0x7c, 0x08, 0x14, 0x22, 0x24, 0x04, 0xfd, 0x05, 
    0x44, 0x08, 0x18, 0x22, 0x44, 0x04, 0x05, 0x05, 
    0x00, 0x28, 0x10, 0x24, 0x87, 0xfc, 0x02, 0x28, 
    0x00, 0x10, 0x00, 0x20, 0x04, 0x04, 0x04, 0x10
};
char Bitmap[1040];

typedef struct
{
  uint32_t freq; 
  uint32_t time; 
} note_t;

note_t little_star[28] = 
{
  {523, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
  {784, SYSTICK_FREQUENCY/8}, 
  {784, SYSTICK_FREQUENCY/8}, 
  {880, SYSTICK_FREQUENCY/8}, 
  {880, SYSTICK_FREQUENCY/8}, 
  {784, SYSTICK_FREQUENCY/4}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8},
	{659, SYSTICK_FREQUENCY/8},
	{587, SYSTICK_FREQUENCY/8},
	{587, SYSTICK_FREQUENCY/8},
	{523, SYSTICK_FREQUENCY/4},	
	{784, SYSTICK_FREQUENCY/8}, 
  {784, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {587, SYSTICK_FREQUENCY/4}, 
	{784, SYSTICK_FREQUENCY/8}, 
  {784, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {587, SYSTICK_FREQUENCY/4}
};


note_t two_tigers[32] = 
{
  {523, SYSTICK_FREQUENCY/8}, 
  {587, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
  {587, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
	
  {659, SYSTICK_FREQUENCY/8}, 
  {698, SYSTICK_FREQUENCY/8},
	{784, SYSTICK_FREQUENCY/4},
	{659, SYSTICK_FREQUENCY/8},
	{698, SYSTICK_FREQUENCY/8},
	{784, SYSTICK_FREQUENCY/4},	
	
	{784, SYSTICK_FREQUENCY/16}, 
  {880, SYSTICK_FREQUENCY/16}, 
  {784, SYSTICK_FREQUENCY/16}, 
  {698, SYSTICK_FREQUENCY/16}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
	{784, SYSTICK_FREQUENCY/16}, 
  {880, SYSTICK_FREQUENCY/16}, 
  {784, SYSTICK_FREQUENCY/16}, 
  {698, SYSTICK_FREQUENCY/16}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
	
  {587, SYSTICK_FREQUENCY/8}, 
  {392, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/4}, 
	{587, SYSTICK_FREQUENCY/8}, 
  {392, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/4},
};


note_t happy_birthday[25] = 
{
  {392, SYSTICK_FREQUENCY/16}, 
  {392, SYSTICK_FREQUENCY/16}, 
  {440, SYSTICK_FREQUENCY/8}, 
  {392, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
  {494, SYSTICK_FREQUENCY/4}, 
  {392, SYSTICK_FREQUENCY/16}, 
  {392, SYSTICK_FREQUENCY/16}, 
  {440, SYSTICK_FREQUENCY/8}, 
  {392, SYSTICK_FREQUENCY/8}, 
  {587, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/4}, 
  {392, SYSTICK_FREQUENCY/16}, 
  {392, SYSTICK_FREQUENCY/16}, 
  {784, SYSTICK_FREQUENCY/8}, 
  {659, SYSTICK_FREQUENCY/8}, 
  {523, SYSTICK_FREQUENCY/8}, 
  {494, SYSTICK_FREQUENCY/8},
  {440, SYSTICK_FREQUENCY/4},  
  {698, SYSTICK_FREQUENCY/16}, 
	{698, SYSTICK_FREQUENCY/16},
	{659, SYSTICK_FREQUENCY/8},
	{523, SYSTICK_FREQUENCY/8},
	{587, SYSTICK_FREQUENCY/8},
	{523, SYSTICK_FREQUENCY/4}
};
uint8_t change_Key = 0;

int main(void)
{
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
	}
	ui32EEPROMInit = EEPROMInit();
	if(ui32EEPROMInit != EEPROM_INIT_OK)
	{
		while(1){}
	}
	EEPROMRead(pui32Read, 0x400, sizeof(pui32Read));
	if(pui32Read[12] == 1)
	{
		int i;
		year_H = pui32Read[0];
		year_L = pui32Read[1];
		month_H = pui32Read[2];
		month_L = pui32Read[3];
		day_H = pui32Read[4];
		day_L = pui32Read[5];
		hour_H = pui32Read[6];
		hour_L = pui32Read[7];
		min_H = pui32Read[8];
		min_L = pui32Read[9];
		sec_H = pui32Read[10];
		sec_L = pui32Read[11];
		for(i = 0; i < 13; i++)
		{
			pui32Data[i] = 0;
		}
		EEPROMProgram(pui32Data, 0x400, sizeof(pui32Data));
	}
	else
	{
		year_H = 2;
		year_L = 3;
		month_H = 0;
		month_L = 6;
		day_H = 1;
		day_L = 1;
		hour_H = 0;
		hour_L = 8;
		min_H = 0;
		min_L = 0;
		sec_H = 5;
		sec_L = 7;
	}
	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_PWM_Init();
	S800_Timer0_Init();
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT );
	IntEnable(INT_TIMER0A);
	SysTickInit();   
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntEnable(INT_UART0);
  IntMasterEnable();
	Print_Bitmap();
	UARTStringPut((char*)"\nÆô¶¯³É¹¦£¡Äú¿ÉÒÔÊäÈë¡°?¡°À´²é¿´ËùÓÐÃüÁîÄó\n");
	while (1)
	{
		if(!isboot) Key_scan();
		if(Alarm_isEnable) Ring();
		Set_With_UART();
		if(isboot) boot();		
		else{		
			switch(mode_key)
			{
				case 1:					
					if(isSet) Set_Time();
					else Display_Time();
					break;
				case 2:					
					if(isSet) Set_Date();
					else Display_Date();
					break;
				case 3:
					if(isSet) Set_Alarm();
					else Display_Alarm();
					break;
				case 4:
					if(isSet) Set_Stopwatch();
					else
					{
						Display_Stopwatch();
						if(yes) isStartTiming = 1;
						if(cancel) isStartTiming = 0;
					}
					break;
				default:
					Display_Time();
			}
		}
	}
}


void trim(char *str)
{
	int i, j;
	for (i = 0, j = 0; str[i]; i++) {
			if (str[i] != ' ') 
					str[j++] = str[i];
	}
	str[j] = '\0'; 
}


void SysTick_Handler(void)    
{
	ms++;
	if(ms >= 1000) ms = 0;
	if(isboot){
		if(++clock100ms >= 100)
		{
			clock100ms_flag++;
			clock100ms = 0;
		}
		if(clock100ms_flag == 24)
		{
			isboot = 0;
		}
	}
	
	if(++clock800ms >= 800)
	{
		seg_blink = 1 - seg_blink;
	}
	if(++clock1s >= 1000)
	{
		sec_L++;
		clock1s = 0;
	}
	if(sec_L >= 10)
	{
			sec_H++;
			sec_L = 0;
	}
	if(sec_H >= 6)
	{
		min_L++;
		sec_H = 0;
	}
	if(min_L >= 10)
	{
		min_H++;
		min_L = 0;
	}
	if(min_H >= 6)
	{
		hour_L++;
		min_H = 0;
	}
	if(hour_L >= 10)
	{
		hour_H++;
		hour_L = 0;
	}
	hour = hour_H*10 + hour_L;
	minute = min_H*10 + min_L;
	second = sec_H*10 + sec_L;

	usr_Key1 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);
	usr_Key2 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1);
	
}


void boot(void)
{
	int i;
	for(i = 0; i < clock100ms_flag%8+1; i++)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,256-pow(2,clock100ms_flag%8+1));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[StudentID[i]]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<i);
		Delay(t);
	}
}


void reset(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
	}
	ui32EEPROMInit = EEPROMInit();
	if(ui32EEPROMInit != EEPROM_INIT_OK)
	{
		while(1){}
	}
	pui32Data[0] = year_H;
	pui32Data[1] = year_L;
	pui32Data[2] = month_H;
	pui32Data[3] = month_L;
	pui32Data[4] = day_H;
	pui32Data[5] = day_L;
	pui32Data[6] = hour_H;
	pui32Data[7] = hour_L;
	pui32Data[8] = min_H;
	pui32Data[9] = min_L;
	pui32Data[10] = sec_H;
	pui32Data[11] = sec_L;
	pui32Data[12] = 1;
	EEPROMProgram(pui32Data, 0x400, sizeof(pui32Data));
	SysCtlReset();
}


void Key_scan(void)
{
	int i,j;
	Key = ~I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
	change_Key = Key ^ lastKey; 
	
	if(usr_Key1 == lastusr_Key1 && usr_Key1 == 0)
	{
		yes = 1;
		cancel = 0;
	}
	else if(usr_Key2 == lastusr_Key2 && usr_Key2 == 0)
	{
		yes = 0;
		cancel = 1;
	}
	else
	{
		yes = 0;
		cancel = 0;
	}
	if((lastusr_Key1 != 0 && usr_Key1 == 0) || (lastusr_Key2 != 0 && usr_Key2 == 0))
	{
		usr_key_press_time[0] = sec_H + '0';
		usr_key_press_time[1] = sec_L + '0';
		usr_key_press_time[2] = '.';
		usr_key_press_time[3] = ms/100 + '0';
		usr_key_press_time[4] = (ms/10)%10 + '0';
		usr_key_press_time[5] = ms%10 + '0';
		usr_key_press_time[6] = 's';
		usr_key_press_time[7] = '\n';
		usr_key_press_time[8] = '\0';
		press_time = usr_key_press_time[0]*10000 + usr_key_press_time[1]*1000 + usr_key_press_time[3]*100 + usr_key_press_time[4]*10 + usr_key_press_time[5]*1;
		UARTStringPut((char*)"°´ÏÂ°´¼üÊ±¼äÎª£º");
		UARTStringPut(usr_key_press_time);
	}
	if((lastusr_Key1 == 0 && usr_Key1 != 0) || (lastusr_Key2 == 0 && usr_Key2 != 0))
	{
		usr_key_release_time[0] = sec_H + '0';
		usr_key_release_time[1] = sec_L + '0';
		usr_key_release_time[2] = '.';
		usr_key_release_time[3] = ms/100 + '0';
		usr_key_release_time[4] = (ms/10)%10 + '0';
		usr_key_release_time[5] = ms%10 + '0';
		usr_key_release_time[6] = 's';
		usr_key_release_time[7] = '\n';
		usr_key_release_time[8] = '\0';
		release_time = usr_key_release_time[0]*10000 + usr_key_release_time[1]*1000 + usr_key_release_time[3]*100 + usr_key_release_time[4]*10 + usr_key_release_time[5]*1;
		UARTStringPut((char*)"ËÉ¿ª°´¼üÊ±¼äÎª£º");
		UARTStringPut(usr_key_release_time);

		duration = release_time - press_time;
		usr_key_duration[0] = duration/10000 + '0';
		usr_key_duration[1] = (duration/1000)%10 + '0';
		usr_key_duration[2] = '.';
		usr_key_duration[3] = (duration/100)%10 + '0';
		usr_key_duration[4] = (duration/10)%10 + '0';
		usr_key_duration[5] = duration%10 + '0';
		usr_key_duration[6] = 's';
		usr_key_duration[7] = '\n';
		usr_key_duration[8] = '\0';
		UARTStringPut((char*)"³ÖÐøÊ±¼äÎª£º");
		UARTStringPut(usr_key_duration);
	}

	if(change_Key != 0 && Key != 0) 
	{
		for(i = 3; i >= 0; i--) 
		{
			if(change_Key & (1 << i)) 
			{
				if(Key & (1 << i))
				{
					mode_key = i+1;
				}
				else 
				{
					mode_key = 1;
				}
			}
		}
		for(j = 4; j < 8; j++) 
		{
			if(change_Key & (1 << j)) 
			{
				if(Key & (1 << j))
				{
					set_key = j+1;
				}
				else 
				{
					set_key = 0;
				}
			}
		}
	}
	lastKey = Key; 
	lastusr_Key1 = usr_Key1;
	lastusr_Key2 = usr_Key2;
}


void Display_Time(void)
{
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);
	if(isReverse)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,127);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[hour_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[hour_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[min_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[min_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[sec_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[sec_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);
		}
	else
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[min_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sec_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sec_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);
	}
}


void Display_Date(void)
{
	if(isReverse)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~64);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[year_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[year_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[month_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[month_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[day_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[day_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);	
	}
	else
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~2);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[year_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[year_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);	
	}
}


void Display_Alarm(void)
{
	if(isReverse)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~32);
		if(Alarm_isEnable)
		{
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_hour_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_hour_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_min_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_min_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_sec_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_sec_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
			Delay(t);
		}
		else
		{
			if(seg_blink)
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
				Delay(8*t);
			}
			else
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_hour_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_hour_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_min_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_min_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_sec_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[a_sec_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
				Delay(t);
			}
		}
	}
	
	else
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~4);
		if(Alarm_isEnable)
		{
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_hour_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_hour_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_min_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_min_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_sec_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_sec_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
			Delay(t);
		}
		else
		{
			if(seg_blink)
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
				Delay(8*t);
			}
			else
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_hour_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_hour_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_min_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_min_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_sec_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[a_sec_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
				Delay(t);
			}
		}
	}
}


void Display_Stopwatch(void)
{
	if(isReverse)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~16);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_min_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(64));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_min_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0X40);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(16));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_sec_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(8));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_sec_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_ms_H]|0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[t_ms_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(1));
		Delay(t);		
	}
	else
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~8);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_min_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(1));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_min_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0X40);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_sec_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(8));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_sec_L]|0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(16));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_ms_H]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[t_ms_L]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(64));
		Delay(t);	
	}
}
	

void Set_With_Key(void)
{
	int i;
	if(set_key == 8 && isShortPress[7] && isRelease[7])
	{
		if(seg_pos > 0)
		{
			if(seg_pos == 3 || seg_pos == 6)
			{
				seg_pos -= 2;
			}
			else
			{
				seg_pos--;
			}
		}
		else
		{
			seg_pos = 7;
		}
		isShortPress[7] = 0;
	}
	if(set_key == 7 && isShortPress[6] && isRelease[6])
	{
		if(seg_pos < 7)
		{
			if(seg_pos == 1 || seg_pos == 4)
			{
				seg_pos += 2;
			}
			else
			{
				seg_pos++;
			}
		}
		else
		{
			seg_pos = 0;
		}
		isShortPress[6] = 0;
	}
	if(set_key == 6 && isShortPress[5] && isRelease[5])
	{
		seg_data[seg_pos]--; 
		isShortPress[5] = 0;
	}
	if(set_key == 5 && isShortPress[4] && isRelease[4])
	{
		seg_data[seg_pos]++; 
		isShortPress[4] = 0;
	}
	
	for(i = 0; i < seg_bit; i++)
	{
		if(i == seg_pos && seg_blink)
		{
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
			Delay(t);
		}
		else if(i == 2 || i == 5)
		{
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);
			Delay(t);
		}
		else
		{
			if(isReverse)
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,re_seg7[seg_data[i]]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
				Delay(t);
			}
			else
			{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[seg_data[i]]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
				Delay(t);
			}
		}
	}
}


void Set_Time(void)
{
	seg_bit = 8;
	if(isReverse)
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,127);
	}
	else
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);
	}
	if(set_flag)
	{
		seg_data[0] = hour_H;
		seg_data[1] = hour_L;
		seg_data[3] = min_H;
		seg_data[4] = min_L;
		seg_data[6] = sec_H;
		seg_data[7] = sec_L;
		set_flag = 0;
	}
	Set_With_Key();
	if(seg_data[0] > 2 && seg_data[1] != 3) seg_data[0] = 2;
	if(seg_data[1] > 9) 
	{
		seg_data[1] = 0;
		seg_data[0]++;
	}
	if(seg_data[0] == 2 && seg_data[1] > 3)
	{
		seg_data[0] = 0;
		seg_data[1] = 0;
	}
	if(seg_data[3] > 5)
	{
		seg_data[3] = 0;
	}
	if(seg_data[4] > 9)
	{
		seg_data[3]++;
		seg_data[4] = 0;
	}
	if(seg_data[6] > 5)
	{
		seg_data[6] = 0;
	}
	if(seg_data[7] > 9)
	{
		seg_data[6]++;
		seg_data[7] = 0;
	}
	
	if(seg_data[0] < 0 && seg_data[1] != 0) seg_data[0] = 0;
	if(seg_data[1] < 0) 
	{
		seg_data[1] = 9;
		seg_data[0]--;
	}
	if(seg_data[0] == 0 && seg_data[1] < 0)
	{
		seg_data[0] = 2;
		seg_data[1] = 3;
	}
	if(seg_data[3] < 0)
	{
		seg_data[3] = 5;
	}
	if(seg_data[4] < 0)
	{
		seg_data[3]--;
		seg_data[4] = 9;
	}
	if(seg_data[6] < 0)
	{
		seg_data[6] = 5;
	}
	if(seg_data[7] < 0)
	{
		seg_data[6]--;
		seg_data[7] = 9;
	}
	if(yes)
	{
		hour_H = seg_data[0];
		hour_L = seg_data[1];
		min_H = seg_data[3];
		min_L = seg_data[4];
		sec_H = seg_data[6];
		sec_L = seg_data[7];
		isSet = 0;
	}
	if(cancel)
	{
		isSet = 0;
	}
}


void Set_Date(void)
{
	seg_bit = 8;
	if(isReverse)
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~64);
	}
	else
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~2);
	}
	if(set_flag)
	{
		seg_data[0] = year_H;
		seg_data[1] = year_L;
		seg_data[3] = month_H;
		seg_data[4] = month_L;
		seg_data[6] = day_H;
		seg_data[7] = day_L;
		set_flag = 0;
	}
	Set_With_Key();
	if(seg_data[0] > 9) seg_data[0] = 0;
	if(seg_data[1] > 9) seg_data[1] = 0;
	if(seg_data[0] < 0) seg_data[0] = 9;
	if(seg_data[1] < 0) seg_data[1] = 9;
	year = 2000 + seg_data[0]*10 + seg_data[1];
	if(seg_data[3] > 1)	seg_data[3] = 0;
	if(seg_data[4] > 9)
	{
		seg_data[3]++;
		seg_data[4] = 0;
	}
	if(seg_data[3] == 1 && seg_data[4] > 2)
	{
		seg_data[3] = 0;
		seg_data[4] = 0;
	}
	if(seg_data[3] < 0)	seg_data[3] = 1;
	if(seg_data[4] < 0)
	{
		seg_data[3]--;
		seg_data[4] = 9;
	}
	if(seg_data[3] == 0 && seg_data[4] < 1)
	{
		seg_data[3] = 1;
		seg_data[4] = 2;
	}
	month = seg_data[3]*10 + seg_data[4];
	if(month != 2)
	{
		if(seg_data[6] > 3) seg_data[6] = 0;
		if(seg_data[7] > 9 && seg_data[6] != 3)
		{
			seg_data[6]++;
			seg_data[7] = 0;
		}
		if(seg_data[6] == 3)
		{
			if(month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12)
			{
				if(seg_data[7] > 1)
				{
					seg_data[6] = 0;
					seg_data[7] = 1;
				}
			}
			else
			{
				if(seg_data[7] > 0)
				{
					seg_data[6] = 0;
					seg_data[7] = 1;
				}
			}
		}
	}
	else if(month == 2)
	{
		if(seg_data[6] > 2) seg_data[6] = 0;
		if(seg_data[7] > 9 && seg_data[6] != 2)
		{
			seg_data[6]++;
			seg_data[7] = 0;
		}
		if(seg_data[6] == 2)
		{
			if(year%400 == 0 ||(year%100 != 0 && year%4 == 0))
			{
				if(seg_data[7] > 9)
				{
					seg_data[6] = 0;
					seg_data[7] = 1;
				}
			}
			else
			{
				if(seg_data[7] > 8)
				{
					seg_data[6] = 0;
					seg_data[7] = 0;
				}
			}
		}
	}
	if(yes)
	{
		year_H = seg_data[0];
		year_L = seg_data[1];
		month_H = seg_data[3];
		month_L = seg_data[4];
		day_H = seg_data[6];
		day_L = seg_data[7];
		isSet = 0;
	}
	if(cancel)
	{
		isSet = 0;
	}
}

	
void Set_Alarm(void)
{
	seg_bit = 8;
	if(isReverse)
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~32);
	}
	else
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~4);
	}
	if(set_flag)
	{
		seg_data[0] = hour_H;
		seg_data[1] = hour_L;
		seg_data[3] = min_H;
		seg_data[4] = min_L;
		seg_data[6] = sec_H;
		seg_data[7] = sec_L;
		set_flag = 0;
	}
	Set_With_Key();
	if(seg_data[0] > 2 && seg_data[1] != 3) seg_data[0] = 2;
	if(seg_data[1] > 9) 
	{
		seg_data[1] = 0;
		seg_data[0]++;
	}
	if(seg_data[0] == 2 && seg_data[1] > 3)
	{
		seg_data[0] = 0;
		seg_data[1] = 0;
	}
	if(seg_data[3] > 5)
	{
		seg_data[3] = 0;
	}
	if(seg_data[4] > 9)
	{
		seg_data[3]++;
		seg_data[4] = 0;
	}
	if(seg_data[6] > 5)
	{
		seg_data[6] = 0;
	}
	if(seg_data[7] > 9)
	{
		seg_data[6]++;
		seg_data[7] = 0;
	}
	
	if(seg_data[0] < 0 && seg_data[1] != 0) seg_data[0] = 0;
	if(seg_data[1] < 0) 
	{
		seg_data[1] = 9;
		seg_data[0]--;
	}
	if(seg_data[0] == 0 && seg_data[1] < 0)
	{
		seg_data[0] = 2;
		seg_data[1] = 3;
	}
	if(seg_data[3] < 0)
	{
		seg_data[3] = 5;
	}
	if(seg_data[4] < 0)
	{
		seg_data[3]--;
		seg_data[4] = 9;
	}
	if(seg_data[6] < 0)
	{
		seg_data[6] = 5;
	}
	if(seg_data[7] < 0)
	{
		seg_data[6]--;
		seg_data[7] = 9;
	}
	if(yes)
	{
		a_hour_H = seg_data[0];
		a_hour_L = seg_data[1];
		a_min_H = seg_data[3];
		a_min_L = seg_data[4];
		a_sec_H = seg_data[6];
		a_sec_L = seg_data[7];
		isSet = 0;
		a_hour = a_hour_H*10 + a_hour_L;
		a_minute = a_min_H*10 + a_min_L;
		a_second = a_sec_H*10 + a_sec_L;
		Alarm_isEnable = 1;
	}
	if(cancel)
	{
		a_hour_H = seg_data[0];
		a_hour_L = seg_data[1];
		a_min_H = seg_data[3];
		a_min_L = seg_data[4];
		a_sec_H = seg_data[6];
		a_sec_L = seg_data[7];
		isSet = 0;
		Alarm_isEnable = 0;
	}
}


void Set_Stopwatch(void)
{
	int i;		
	seg_bit = 5;
	if(set_flag)
	{
		seg_data[0] = 0;
		seg_data[1] = 0;
		seg_data[3] = 0;
		seg_data[4] = 0;
		set_flag = 0;
	}
	if(isReverse)
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~16);
	}
	else
	{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~8);
	}
	Set_With_Key();
	if(seg_data[0] > 9) seg_data[i] = 0;
	if(seg_data[0] < 0) seg_data[i] = 9;
	if(seg_data[1] > 9) seg_data[i] = 0;
	if(seg_data[1] < 0) seg_data[i] = 9;
	if(seg_data[3] > 6) seg_data[i] = 0;
	if(seg_data[3] < 0) seg_data[i] = 5;
	if(seg_data[4] > 9)
	{
		seg_data[3]++;
		seg_data[i] = 0;
	}
	if(seg_data[4] < 0)
	{
		seg_data[3]--;
		seg_data[i] = 9;
	}
	
	if(yes)
	{
		t_min_H = seg_data[0];
		t_min_L = seg_data[1];
		t_sec_H = seg_data[3];
		t_sec_L = seg_data[4];
		isSet = 0;
		isStartTiming = 1;
	}
	if(cancel)
	{
		isSet = 0;
		isStartTiming = 0;
	}
}


void Set_With_UART(void)
{
	int i,j;
	int k = 0;
	trim(RxBuf);
	if (RxEndFlag)
	{
		for(i = 0; i < sizeof(RxBuf); i++)
		{
			for(j = 0; j < 10; j++)
			{
				if(numbers[j]==RxBuf[i+1])
				{
					Serial[k]=j;
					k++;
				}
			}	
		}
		if(strncmp(RxBuf, "RESET", 5) == 0)
		{
			reset();
		}
		
		else if(strncmp(RxBuf,"SET",3) == 0)
		{
			if(strncmp(RxBuf,"SETTIME",7) == 0)
			{
				hour_H = Serial[0];
				hour_L = Serial[1];
				min_H = Serial[2];
				min_L = Serial[3];
				sec_H = Serial[4];
				sec_L = Serial[5];
				UARTStringPut((char*)"ÉèÖÃÊ±¼ä³É¹¦ß÷~\n");
			}
			else if(strncmp(RxBuf,"SETDATE",7) == 0)
			{
				year_H = Serial[2];
				year_L = Serial[3];
				month_H = Serial[4];
				month_L = Serial[5];
				day_H = Serial[6];
				day_L = Serial[7];
				UARTStringPut((char*)"ÉèÖÃÈÕÆÚ³É¹¦ß÷~\n");
			}
			else if(strncmp(RxBuf,"SETALARM",8) == 0)
			{
				a_hour_H = Serial[0];
				a_hour_L = Serial[1];
				a_min_H = Serial[2];
				a_min_L = Serial[3];
				a_sec_H = Serial[4];
				a_sec_L = Serial[5];
				UARTStringPut((char*)"ÉèÖÃÄÖÖÓÊ±¼ä³É¹¦ß÷~\n");
			}
			else if(strncmp(RxBuf,"SETRING",7) == 0)
			{
				ringtone = Serial[0];
				UARTStringPut((char*)"ÉèÖÃÄÖÖÓÁåÉù³É¹¦ß÷~\n");
			}
			else if(strncmp(RxBuf,"SETSTWATCH",10) == 0)
			{
				t_min_H = Serial[0];
				t_min_L = Serial[1];
				t_sec_H = Serial[2];
				t_sec_L = Serial[3];
				UARTStringPut((char*)"ÉèÖÃµ¹¼ÆÊ±³É¹¦ß÷~\n");
			}
			else
			{
				UARTStringPut((char*)"ÊäÈëµÄÃüÁîÓÐÎó£¬Äú¿ÉÒÔÊäÈë¡°£¿¡±À´²é¿´ËùÓÐÃüÁîÅ¶~\n" );
			}
			RxEndFlag=0; 
		}
		
		else if(strncmp(RxBuf,"GET",3) == 0)
		{
			if(strncmp(RxBuf,"GETTIME",7) == 0)
			{
				State_Get[0] = hour_H + '0';
				State_Get[1] = hour_L + '0';
				State_Get[2] = ':';
				State_Get[3] = min_H + '0';
				State_Get[4] = min_L + '0';
				State_Get[5] = ':';
				State_Get[6] = sec_H + '0';
				State_Get[7] = sec_L + '0';
				UARTStringPut((char*)"µ±Ç°Ê±¼äÎª£º");
				UARTStringPut(State_Get);
			}
			else if(strncmp(RxBuf,"GETDATE",7) == 0)
			{
				State_Get[0] = year_H + '0';
				State_Get[1] = year_L + '0';
				State_Get[2] = '-';
				State_Get[3] = month_H + '0';
				State_Get[4] = month_L + '0';
				State_Get[5] = '-';
				State_Get[6] = day_H + '0';
				State_Get[7] = day_L + '0';
				UARTStringPut((char*)"µ±Ç°ÈÕÆÚÎª£º");
				UARTStringPut(State_Get);
			}
			else if(strncmp(RxBuf,"GETALARM",8) == 0)
			{
				State_Get[0] = a_hour_H + '0';
				State_Get[1] = a_hour_L + '0';
				State_Get[2] = ':';
				State_Get[3] = a_min_H + '0';
				State_Get[4] = a_min_L + '0';
				State_Get[5] = ':';
				State_Get[6] = a_sec_H + '0';
				State_Get[7] = a_sec_L + '0';
				UARTStringPut((char*)"Éè¶¨µÄÄÖÖÓÊ±¼äÎª£º");
				UARTStringPut(State_Get);
			}
			else
			{
				UARTStringPut((char*)"ÊäÈëµÄÃüÁîÓÐÎó£¬Äú¿ÉÒÔÊäÈë¡°£¿¡±À´²é¿´ËùÓÐÃüÁî\n" );
			}
			RxEndFlag = 0;
		}
		
		else if(strncmp(RxBuf,"RUN",3) == 0)
		{
			if(strncmp(RxBuf,"RUNTIME",7) == 0)
			{
				mode_key = 1;
				UARTStringPut((char*)"ÏÔÊ¾µ±Ç°Ê±¼ä\n" );
			}
			else if(strncmp(RxBuf,"RUNDATE",7) == 0)
			{
				mode_key = 2;
				UARTStringPut((char*)"ÏÔÊ¾µ±Ç°ÈÕÆÚ\n" );
			}
			else if(strncmp(RxBuf,"RUNALARM",8) == 0)
			{
				mode_key = 3;
				UARTStringPut((char*)"ÏÔÊ¾ÄÖÖÓ\n" );
			}
			else if(strncmp(RxBuf,"RUNSTWATCH",10) == 0)
			{
				mode_key = 4;
				isStartTiming = 1;
				UARTStringPut((char*)"µ¹¼ÆÊ±Æô¶¯\n" );
			}
			else
			{
				UARTStringPut((char*)"ÊäÈëµÄÃüÁîÓÐÎó£¬Äú¿ÉÒÔÊäÈë¡°£¿¡±À´²é¿´ËùÓÐÃüÁî\n" );
			}
			RxEndFlag = 0;
		}
		
		else if(strncmp(RxBuf, "OPENALARM", 9) == 0)
		{
			Alarm_isEnable = 1;
			RxEndFlag = 0;
			UARTStringPut((char*)"ÄÖÖÓÒÑ¿ªÆô\n" );
		}
		else if(strncmp(RxBuf, "CLOSEALARM", 10) == 0)
		{
			Alarm_isEnable = 0;
			RxEndFlag = 0;
			UARTStringPut((char*)"ÄÖÖÓÒÑ¹Ø±Õ\n" );
		}
		
		else if(strncmp(RxBuf, "PAUSE", 5) == 0)
		{
			isStartTiming = 0;
			RxEndFlag = 0;
			UARTStringPut((char*)"µ¹¼ÆÊ±ÔÝÍ£\n" );
		}
		
		else if(strncmp(RxBuf, "TOGGLE", 6) == 0)
		{
			isReverse = 1 - isReverse;
			RxEndFlag = 0;
			UARTStringPut((char*)"ÊýÂë¹ÜÏÔÊ¾ÒÑ·­×ª\n" );
		}
		else if(RxBuf[0] == '?')
		{
			UARTStringPut((char*)"±¾Êý×ÖÖÓ¿ÉÊ¹ÓÃÃüÁîÈçÏÂ£º\n");
			UARTStringPut((char*)"1.ÖØÆôÃüÁî£º\n");
			UARTStringPut((char*)"RESET			ÈíÖØÆôÊý×ÖÖÓ£¬±£Áôµ±Ç°Ê±¼äÈÕÆÚ\n\n");
			UARTStringPut((char*)"2.ÉèÖÃÃüÁî£º\n");
			UARTStringPut((char*)"SET TIME XX:XX:XX	ÉèÖÃÊ±¼äÎªXXÊ±XX·ÖXXÃë\n");
			UARTStringPut((char*)"SET DATE XXXX-XX-XX	ÉèÖÃÈÕÆÚÎªXXXXÄêXXÔÂXXÈÕ\n");
			UARTStringPut((char*)"SET ALARM XX:XX:XX	ÉèÖÃÄÖÖÓÊ±¼äÎªXXÊ±X·ÖXXÃë\n");
			UARTStringPut((char*)"SET RING 1/2/3		ÉèÖÃÄÖÖÓÁåÉùÎªÁåÉù1/2/3£¬ÁåÉù1£ºÐ¡ÐÇÐÇ£¬ÁåÉù2£ºÁ½Ö»ÀÏ»¢£¬ÁåÉù3£ºÉúÈÕ¿ìÀÖ\n");
			UARTStringPut((char*)"OPENALARM		´ò¿ªÄÖÖÓ\n");
			UARTStringPut((char*)"CLOSEALARM		¹Ø±ÕÄÖÖÓ\n");
			UARTStringPut((char*)"SET STWATCH XX:XX	ÉèÖÃµ¹¼ÆÊ±´ÓXX·ÖXXÃë¿ªÊ¼\n\n");
			UARTStringPut((char*)"3.»ñÈ¡×´Ì¬ÃüÁî£º\n");
			UARTStringPut((char*)"GET TIME		»ñÈ¡µ±Ç°Ê±¼ä\n");
			UARTStringPut((char*)"GET DATE		»ñÈ¡µ±Ç°ÈÕÆÚ\n");
			UARTStringPut((char*)"GET ALARM		»ñÈ¡µ±Ç°ÄÖÖÓÊ±¼ä\n\n");
			UARTStringPut((char*)"4.ÔËÐÐÃüÁî£º\n");
			UARTStringPut((char*)"RUN TIME		ÏÔÊ¾µ±Ç°Ê±¼ä\n");
			UARTStringPut((char*)"RUN DATE		ÏÔÊ¾µ±Ç°ÈÕÆÚ\n");
			UARTStringPut((char*)"RUN ALARM		ÏÔÊ¾µ±Ç°ÄÖÖÓÊ±¼ä\n");
			UARTStringPut((char*)"RUN STWATCH		 ÔËÐÐµ¹¼ÆÊ±Ãë±í\n");
			UARTStringPut((char*)"PAUSE	  		ÔÝÍ£µ¹¼ÆÊ±Ãë±í\n\n" );
			UARTStringPut((char*)"5.·­×ªÃüÁî£º\n");
			UARTStringPut((char*)"TOGGLE			·­×ªÊýÂë¹ÜÏÔÊ¾\n");
			RxEndFlag = 0;
		}
		
		else
		{
			UARTStringPut((char*)"ÊäÈëµÄÃüÁîÓÐÎó£¬Äú¿ÉÒÔÊäÈë¡°£¿¡±À´²é¿´ËùÓÐÃüÁî\n" );
			RxEndFlag = 0;
			}
	}
}
		
		
void Ring(void)
{
	int i;
	if(a_hour == hour && a_minute == minute && a_second == second)
	{
		isRing = 1;
	}
	if(yes || minute - a_minute >= 1)
	{
		isRing = 0;
	}
	if(isRing)
	{
		if(ringtone == 1){
			for(i = 0; i < 28; i++)
			{
				PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,SYSTICK_FREQUENCY/little_star[i].freq); 
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true); 
				Display_Time();
				if(yes || minute - a_minute >= 1)
				{
					isRing = 0;
					break;
				}
				Delay(little_star[i].time);
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false); 
			}
		}
		if(ringtone == 2){
			for(i = 0; i < 32; i++)
			{
				PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,SYSTICK_FREQUENCY/two_tigers[i].freq); 
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true); 
				Display_Time();
				if(yes || minute - a_minute >= 1)
				{
					isRing = 0;
					break;
				}
				Delay(two_tigers[i].time);
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false); 
			}
		}
		if(ringtone == 3){
			for(i = 0; i < 25; i++)
			{
				PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,SYSTICK_FREQUENCY/happy_birthday[i].freq); 
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true); 
				Display_Time();
				if(yes || minute - a_minute >= 1)
				{
					isRing = 0;
					break;
				}
				Delay(happy_birthday[i].time);
				PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false); 
			}
		}
		PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,false); 
		Delay(SYSTICK_FREQUENCY/2);
		Display_Time();
	}
}


void Print_Bitmap(void)
{
	int i,j,k;
	for(i = 0; i < 16; i++)
	{
		for(j = 0; j < 8; j++)
		{
			for(k = 7; k >= 0; k--)
			{
				if((Bitmap_Bytes[i*8+j]>>k)&1) Bitmap[i*64+j*8+7-k] = '$';
				else Bitmap[i*64+j*8+7-k] = ' ';
			}
		}
		Bitmap[i*64] = '\n';
	}
	UARTStringPut(Bitmap);
}
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void SysTickInit(void)
{
	SysTickPeriodSet(ui32SysClock/1000);
	SysTickEnable();  			
	SysTickIntEnable();		
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){}
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_5);		
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPinConfigure(GPIO_PK5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
}


void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);//³õÊ¼»¯i2cÄ£¿é
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//Ê¹ÓÃI2CÄ£¿é0£¬Òý½ÅÅäÖÃÎªI2C0SCL--PB2¡¢I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);//ÅäÖÃPB2ÎªI2C0SCL
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);//ÅäÖÃPB3ÎªI2C0SDA
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//I2C½«GPIO_PIN_2ÓÃ×÷SCL
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//I2C½«GPIO_PIN_3ÓÃ×÷SDA

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	//UARTStringPut((uint8_t *)"\r\nHello, world!\r\n¡");
}


void S800_PWM_Init(void) 
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { } 
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); 
	PWMGenConfigure(PWM0_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN 
	|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,BeepPeriod);
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,BeepPeriod/4);
	PWMGenEnable(PWM0_BASE,PWM_GEN_3); 
}


void S800_Timer0_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock/100));
	TimerEnable(TIMER0_BASE, TIMER_A);
}


void TIMER0A_Handler(void)
{	
	int i;
	uint32_t intStatus;
	intStatus = TimerIntStatus(TIMER0_BASE, true);
	TimerIntClear(TIMER0_BASE, intStatus ); 
	for(i = 0; i < 8; i++)
	{
		if(Key & (1 << i))
		{
			Key_flag[i] = 1;
			Key_count[i]++;
			isRelease[i] = 0;
		}
		else
		{
			Key_flag[i] = 0;
			Key_count[i] = 0;
			isRelease[i] = 1;
		}
		if(Key_flag[i] && Key_count[i] > 2 && Key_count[i] < 50)
		{
			isShortPress[i] = 1;
			isLongPress[i] = 0;
		}
		else if(Key_flag[i] && Key_count[i] > 100)
		{
			isShortPress[i] = 0;
			isLongPress[i] = 1;
			Key_flag[i] = 0;
			Key_count[i] = 0;
		}
		
		if(isLongPress[i] && i<4)
		{
			isSet = 1 - isSet;
			set_flag = isSet;
			isLongPress[i] = 0;
		}
		if(isLongPress[6]&&isLongPress[7]) reset();
		if(isLongPress[4]&&isLongPress[5]) isReverse = 1 - isReverse;
	}
		if(isStartTiming && !(t_ms_L == 0 && t_ms_H == 0 && t_sec_L == 0 && t_sec_H == 0 && t_min_L == 0 && t_min_H == 0))
	{
		t_ms_L--;
		if(t_ms_L < 0)
		{
			t_ms_H--;
			t_ms_L = 9;
		}
		if(t_ms_H < 0)
		{
			t_sec_L--;
			t_ms_H = 9;
		}
		if(t_sec_L < 0)
		{
			t_sec_H--;
			t_sec_L = 9;
		}
		if(t_sec_H < 0)
		{
			t_min_L--;
			t_sec_H = 5;
		}
		if(t_min_L < 0)
		{
			t_min_H--;
			t_min_L = 9;
		}
		if(t_ms_L == 0 && t_ms_H == 0 && t_sec_L == 0 && t_sec_H == 0 && t_min_L == 0 && t_min_H == 0)
		{
			isStartTiming = 0;
		}
	}
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};//Èç¹ûI2C0Ä£¿éÃ¦£¬µÈ´ý
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
		//ÉèÖÃÖ÷»úÒª·Åµ½×ÜÏßÉÏµÄ´Ó»úµØÖ·¡£false±íÊ¾Ö÷»úÐ´´Ó»ú£¬true±íÊ¾Ö÷»ú¶Á´Ó»ú
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//Ö÷»úÐ´Éè±¸¼Ä´æÆ÷µØÖ·
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//Ö´ÐÐÖØ¸´Ð´Èë²Ù×÷
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//µ÷ÊÔÓÃ

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//Ö´ÐÐÖØ¸´Ð´Èë²Ù×÷²¢½áÊø
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//µ÷ÊÔÓÃ

	return rop;//·µ»Ø´íÎóÀàÐÍ£¬ÎÞ´í·µ»Ø0
}


uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//Ö´ÐÐµ¥´ÊÐ´Èë²Ù×÷
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(200);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//ÉèÖÃ´Ó»úµØÖ·
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//Ö´ÐÐµ¥´Î¶Á²Ù×÷
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);//»ñÈ¡¶ÁÈ¡µÄÊý¾Ý
	Delay(200);
	return value;
}


void UART0_Handler(void) 
{ 
	uint32_t ulStatus;
	ulStatus = UARTIntStatus(UART0_BASE, true); 
	UARTIntClear(UART0_BASE, ulStatus); 
	while( UARTCharsAvail(UART0_BASE) )
	{ 
		RxBuf[cnt] = toupper(UARTCharGetNonBlocking(UART0_BASE));
		if(RxBuf[cnt] != '\n')
		{
			cnt++;
		}
		else
		{
			RxBuf[cnt] = '\0';
			RxEndFlag=1; 
			cnt = 0;
			break;
		}
	}
} 


void UARTStringGet(char *msg)
{
	while(1)
	{
		*msg = UARTCharGet(UART0_BASE); 
		if (*msg =='\r') 
		{ 
		 *msg = UARTCharGet(UART0_BASE); 
		 break; 
		}
		msg++; 
	}
	*msg = '\0';
}


void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

