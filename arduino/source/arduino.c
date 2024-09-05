/*
 * Arduino.c
 *
 *  Created on: Jan 12, 2023
 *      Author: larry
 */
#include "debug.h"
#include "arduino.h"

void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOD, &GPIO_InitStructure);

    { // init D1 as GPIO
        GPIOD->CFGLR &= ~( 0b11 << 6 );\
        u32 tmp = AFIO->PCFR1 & (~(0b111 << 24));
        tmp |= 0b100 << 24;
        AFIO->PCFR1 |= tmp;
    }

    { // init A1 A2 as GPIO
        AFIO->PCFR1 &= ~0x8000;
    }
}

void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* GPIOA ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int main(void){
    Delay_Init();
    EXTI0_INT_INIT();
    GPIO_Toggle_INIT();
    setup();
    while(1){
        loop();
    }
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 Handler.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{
    // go to bootloader
    if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);     /* Clear Flag */
        RCC_ClearFlag();
        SystemReset_StartMode(Start_Mode_BOOT);
        NVIC_SystemReset();
    }
}

void Serial_begin(uint32_t baudrate){
    USART_Printf_Init(baudrate);
}

void delay(unsigned int i)
{
	Delay_Ms(i);
}

void delayMicroseconds(unsigned int us){

    Delay_Us(us);
}

unsigned long micros(void){
    return Systick_micros();
}

unsigned long millis(void){
    return Systick_micros()/1000;
}
// Arduino-like API defines and function wrappers for WCH MCUs

void pinMode(uint8_t u8Pin, int iMode)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    if (u8Pin < 0xa0 || u8Pin > 0xdf) return; // invalid pin number

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 << (u8Pin & 0xf);
    if (iMode == OUTPUT)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    else if (iMode == INPUT)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    else if (iMode == INPUT_PULLUP)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    else if (iMode == INPUT_PULLDOWN)
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    switch (u8Pin & 0xf0) {
    case 0xa0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    	break;
    case 0xc0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    	break;
    case 0xd0:
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    	break;
    }
} /* pinMode() */

uint8_t digitalRead(uint8_t u8Pin)
{
    uint8_t u8GPIO = GPIO_Pin_0 << (u8Pin & 0xf);
    uint8_t u8Value = 0;
    switch (u8Pin & 0xf0) {
    case 0xa0:
    	u8Value = GPIO_ReadInputDataBit(GPIOA, u8GPIO);
    	break;
    case 0xc0:
    	u8Value = GPIO_ReadInputDataBit(GPIOC, u8GPIO);
    	break;
    case 0xd0:
    	u8Value = GPIO_ReadInputDataBit(GPIOD, u8GPIO);
    	break;
    }
    return u8Value;
} /* digitalRead() */

void digitalWrite(uint8_t u8Pin, uint8_t u8Value)
{
	uint8_t u8GPIO = GPIO_Pin_0 << (u8Pin & 0xf);
	u8Value = (u8Value) ? Bit_SET : Bit_RESET;

	switch (u8Pin & 0xf0) {
	case 0xa0:
		GPIO_WriteBit(GPIOA, u8GPIO, u8Value);
		break;
	case 0xc0:
		GPIO_WriteBit(GPIOC, u8GPIO, u8Value);
		break;
	case 0xd0:
		GPIO_WriteBit(GPIOD, u8GPIO, u8Value);
		break;
	}
} /* digitalWrite() */

// Put CPU into standby mode for a multiple of 82ms tick increments
// max ticks value is 63
void Standby82ms(uint8_t iTicks)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // init external interrupts
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Init GPIOs
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // init wake up timer and enter standby mode
    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
    PWR_AWU_SetPrescaler(PWR_AWU_Prescaler_10240);
    PWR_AWU_SetWindowValue(iTicks);
    PWR_AutoWakeUpCmd(ENABLE);
    PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFE);

    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);

} /* Standby82ms() */

//
// Ramp an LED brightness with PWM from 0 to 50%
// The period represents the total up+down time in milliseconds
//
void breatheLED(uint8_t u8Pin, int iPeriod)
{
	int i, j, iStep, iCount, iOnTime;

	pinMode(u8Pin, OUTPUT);
	// Use a pwm freq of 1000hz and 50 steps up then 50 steps down
	iStep = 10000/iPeriod; // us per step
	iCount = iPeriod / 20;
	// ramp up
	iOnTime = 0;
	for (i=0; i<iCount; i++) {
		for (j=0; j<20; j++) { // 20ms per step
			digitalWrite(u8Pin, 1); // on period
			Delay_Us(iOnTime);
			digitalWrite(u8Pin, 0); // off period
			Delay_Us(1000 - iOnTime);
		} // for j
		iOnTime += iStep;
	} // for i
	// ramp down
	iOnTime = 500;
	for (i=0; i<iCount; i++) {
		for (j=0; j<20; j++) { // 20ms per step
			digitalWrite(u8Pin, 1); // on period
			Delay_Us(iOnTime);
			digitalWrite(u8Pin, 0); // off period
			Delay_Us(1000 - iOnTime);
		} // for j
		iOnTime -= iStep;
	} // for i
} /* breatheLED() */

