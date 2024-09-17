/*
 * Arduino.c
 *
 *  Created on: Jan 12, 2023
 *      Author: larry
 */
#include "debug.h"
#include "arduino.h"
#include "ch32v00x_dma.h"
#include "string.h"

void GPIO_PD1_PA1_PA2_C0_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};    
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    #ifdef USE_PC0_AS_BOOT_SOURCE
    {
        /* PC0 - boot button */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* GPIOC ----> EXTI_Line7 */
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
    #endif // USE_PC0_AS_BOOT_SOURCE

    #ifdef USE_PROGRAMMING_PIN_AS_GPIO
    { // init D1 as GPIO
        GPIOD->CFGLR &= ~( 0b11 << 6 );\
        u32 tmp = AFIO->PCFR1 & (~(0b111 << 24));
        tmp |= 0b100 << 24;
        AFIO->PCFR1 |= tmp;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    }
    #endif // USE_PROGRAMMING_PIN_AS_GPIO

    #ifdef USE_XTAL_PINS_AS_GPIO
    { // init A1 A2 as GPIO
        AFIO->PCFR1 &= ~0x8000;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    #endif // USE_XTAL_PINS_AS_GPIO
}

static u8 RxDmaBuffer[100] = {0};

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
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxDmaBuffer;
    
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_BufferSize = sizeof(RxDmaBuffer);

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE); /* USART1 Rx */
}

void TIM_INIT(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = 60000;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48000-1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    
    TIM_Cmd( TIM1, ENABLE );   
}

void uart_timer_stop()
{
    TIM_Cmd( TIM1, DISABLE );
    TIM_SetCounter( TIM1, 0 );
}

void uart_timer_start()
{
    TIM_Cmd( TIM1, ENABLE );
    TIM_SetCounter( TIM1, 0 );
}

uint8_t uart_timer_check()
{
    if (TIM_GetCounter(TIM1) > 100) {
        uart_timer_stop();
        return 1;
    }
    return 0;
}

uint8_t get_dma_count()
{
    return sizeof(RxDmaBuffer) - DMA_GetCurrDataCounter(DMA1_Channel5);
}

int8_t get_dma_string(uint8_t start, uint8_t end, uint8_t max_size, uint8_t* str)
{
    uint8_t len = 0;
    for (uint8_t s = start; s != end; s = (s + 1) % sizeof(RxDmaBuffer)) {
        str[len++] = RxDmaBuffer[s];
        if (len >= max_size)
        {
            return len;
        }
    }
    return len;
}

void reboot(uint8_t bootloader)
{
    RCC_ClearFlag();
    if (bootloader) SystemReset_StartMode(Start_Mode_BOOT);
    else SystemReset_StartMode(Start_Mode_USER);
    NVIC_SystemReset();
}

void try_run_uart_command(const char* cmd)
{
    if (strcmp(cmd, "command: reboot bootloader") == 0)
    {
        reboot(1);
    }
    else if (strcmp(cmd, "command: reboot") == 0)
    {
        reboot(0);
    }
}

int main(void)
{
    Delay_Init();
    GPIO_PD1_PA1_PA2_C0_Init();

    DMA_INIT();
    USART_Printf_Init(serial_baudrate);

    setup();

    #ifdef UART_COMMANDS_RECEIVE_SERVICE
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    TIM_INIT();

    uint8_t last_dma_count = get_dma_count();
    uint8_t last_dma_check = last_dma_count;
    #endif // UART_COMMANDS_RECEIVE_SERVICE

    while(1)
    {
        loop();
        
        #ifdef UART_COMMANDS_RECEIVE_SERVICE
        uint8_t current_dma_count = get_dma_count();
        if (current_dma_count != last_dma_count)
        {
            uart_timer_start();
        }
        last_dma_count = current_dma_count;

        if (uart_timer_check()) 
        {
            char command[30] = {0};
            if (get_dma_string(last_dma_check, current_dma_count, sizeof(command) - 1, command) > 0)
            {   
                printf("try_run_uart_command %d %d [%s]\r\n", last_dma_check, current_dma_count, command);
                try_run_uart_command(command);
            }
            last_dma_check = current_dma_count; 
        }
        #endif // UART_COMMANDS_RECEIVE_SERVICE
    }
}

#ifdef USE_PC0_AS_BOOT_SOURCE
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
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0); /* Clear Flag */
        reboot(1);
    }
}
#endif // USE_PC0_AS_BOOT_SOURCE

void delay(unsigned int i)
{
	Delay_Ms(i);
}

void delayMicroseconds(unsigned int us)
{
    Delay_Us(us);
}

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


static uint8_t calc_config_crc(config_t* config) {
	uint8_t res = 0x33;
	// simple crc
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		res |= config->data[i];
		res += config->data[i] + 0xAA;
	}
	return res;
}

static u32 sector_address(u32 s) {
	return FLASH_BASE + s * 64;
}

static void write_config_to_sector(config_t* config, u8 s)
{
	FLASH_ErasePage_Fast(sector_address(s));
	FLASH_BufReset();
	for (u8 i = 0; i < 60; i += 4) {
		FLASH_BufLoad(sector_address(s) + i, *(u32*)(&config->data[i]));
	}
	u8 tmp[4];
	tmp[0] = config->data[60];
	tmp[1] = config->data[61];
	tmp[2] = calc_config_crc(config);
	tmp[3] = 0x33;
	FLASH_BufLoad(sector_address(s) + 60, *(u32*)(tmp));
	FLASH_ProgramPage_Fast(sector_address(s));
}

void save_config(config_t* config) {
	u8 i;
	for (i = 0; i < sizeof(config_t); ++i) {
		if (config->data[i] != *(u8*)(sector_address(254) + i)) break;
	}
	if (i == sizeof(config_t)) {
		return;
	}
	FLASH_Unlock_Fast();
	write_config_to_sector(config, 255);
	write_config_to_sector(config, 254);
	FLASH_ErasePage_Fast(sector_address(255));
	FLASH_Lock_Fast();
}

static u8 read_config_from_sector(config_t* config, u8 s) {
	u8 status = *(u8*)(sector_address(s) + 63);
	if (status != 0x33) {
		return 0;
	}
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		config->data[i] = *(u8*)(sector_address(s) + i);
	}
	u8 crc = calc_config_crc(config);
	u8 readed = *(u8*)(sector_address(s) + 62);
	if (crc != readed) {
		return 0;
	}
	return 1;
}

void read_config(config_t* config) {
	if (read_config_from_sector(config, 255) || read_config_from_sector(config, 254)) {
        save_config(config);
		return;
	}
	for (u8 i = 0; i < sizeof(config_t); ++i) {
		config->data[i] = 0;
	}
}
