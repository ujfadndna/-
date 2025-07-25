#include "ti_msp_dl_config.h"
#include "systick.h"



volatile uint32_t sysTickUptime = 0;
void SysTick_Handler(void)
{
	 sysTickUptime++;
}


//���� us
uint32_t micros(void)
{
	uint32_t systick_period=CPUCLK_FREQ/ 1000U;
	return sysTickUptime*1000+(1000*(systick_period-SysTick->VAL))/systick_period;
}


//���� ms
uint32_t millis(void) {
	return micros() / 1000UL;
}

//��ʱus
void delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while((int32_t)(micros() - start) < us) {
    // Do nothing
  };
}


void delay(uint32_t ms) {
  delayMicroseconds(ms * 1000UL);
}


void delay_ms(uint32_t x)
{
  delay(x);
}


void delay_us(uint32_t x)
{
  delayMicroseconds(x);
}

void Delay_Ms(uint32_t time)  //��ʱ����  
{   
	delay_ms(time);
}  

void Delay_Us(uint32_t time)  //��ʱ����  
{   
	delay_us(time);
}  


void get_systime(systime *sys)
{
  sys->last_time=sys->current_time;
  sys->current_time=micros()/1000.0f;//��λms
  sys->period=sys->current_time-sys->last_time;
  sys->period_int=(uint16_t)(sys->period+0.5f);//��������
}


float get_systime_ms(void)
{
  return millis();//��λms
}

uint32_t get_systick_ms(void)
{
  return (uint32_t)(2*sysTickUptime);//��λms
}
