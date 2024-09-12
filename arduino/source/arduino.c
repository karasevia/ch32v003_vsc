/*
 * Arduino.c
 *
 *  Created on: Jan 12, 2023
 *      Author: larry
 */
#include "debug.h"
#include "arduino.h"

void GPIO_PD1_PA1_PA2_C0_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};    
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    /* PC0 - boot button */
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

    { // init D1 as GPIO
        GPIOD->CFGLR &= ~( 0b11 << 6 );\
        u32 tmp = AFIO->PCFR1 & (~(0b111 << 24));
        tmp |= 0b100 << 24;
        AFIO->PCFR1 |= tmp;
    }

    { // init A1 A2 as GPIO
        AFIO->PCFR1 &= ~0x8000;
    }

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

static u8 RxBuffer1[128] = {0};

/*********************************************************************
 * @fn      DMA_INIT
 *
 * @brief   Configures the DMA for USART1.
 *
 * @return  none
 */
void DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer1;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = sizeof(RxBuffer1);
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
}

int main(void)
{
    Delay_Init();
    GPIO_PD1_PA1_PA2_C0_Init();

    DMA_INIT();
    USART_Printf_Init(serial_baudrate);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    setup();
    while(1)
    {
        loop();
        if (DMA_GetFlagStatus(DMA1_FLAG_TC5) != RESET) {
            DMA_ClearFlag(DMA1_FLAG_TC5);
            uint16_t cnt = DMA_GetCurrDataCounter(DMA1_Channel5);
            printf("DMA_GetFlagStatus %d\r\n", cnt);
        }
    }
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void reboot_bootloader()
{
    RCC_ClearFlag();
    SystemReset_StartMode(Start_Mode_BOOT);
    NVIC_SystemReset();
}
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
        reboot_bootloader();
    }
}

void delay(unsigned int i)
{
	Delay_Ms(i);
}

void delayMicroseconds(unsigned int us)
{
    Delay_Us(us);
}

unsigned long micros(void)
{
    return Systick_micros();
}

unsigned long millis(void)
{
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
