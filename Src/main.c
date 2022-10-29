/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Denis Petrunin
 * @brief          : Main program body
 ******************************************************************************
*/

#include <stdint.h>
#include "stm32f407xx.h"

#define PBA 0x40000000 // peripheral Base Address
#define PBA_AHB1 (PBA+0x20000) // AHB1 PBA

//светодиоды
//#define GPIOD_BASE (PBA_AHB1+0xC00) // порт со светодиодами на ногах 12-15
#define GPIOD ((GPIO_TypeDef *) GPIOD_BASE)

//PA0 клавиша
//#define GPIOA_BASE (PBA_AHB1+0x0) // порт с клавишей PA0 (без смещения)
#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)

//настройка частоты
#define HCLK		168000000
#define SysTicksClk	1000
#define SysTicks	HCLK/SysTicksClk

void delay_ms(uint16_t delay);
void timInit(void);

uint16_t delay_count=0;

/*
typedef struct
{
	uint32_t MODER;//  0x00 //нужен, 00==вход 01==выход
	uint32_t OTYPER;//  0x04
	uint32_t OSPEEDR;//  0x08
	uint32_t PUPDR;//  0x0C
	uint32_t IDR;//  0x10 //нужен
	uint32_t ODR;//  0x14 //нужен
	uint32_t BSRR;//  0x18
	uint32_t LCKR;//  0x1C
	uint32_t AFRL;//  0x20
	uint32_t AFRH;//  0x24
} GPIO_TypeDef;
*/

//#define peripheralBitBandRegionBaseAdress
//#define advancedHardwareBus1PBA

int main(void)
{
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // 5 в latency
	//RCC->CR |= 1<<16;  //устанавливаем 16-йбит для включения HSE
	RCC->CR |= RCC_CR_HSEON;
	//RCC->CR |= 1<<0;  //устанавливаем 0-йбит для включения HSI
	while (!(RCC->CR & (1<<17))); //читаем сr ready пока не будет готов

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB в HPRE
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 в PPRE1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 в PPRE2

	RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLM_Msk;
	RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLN_Msk;
	RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLP_Msk;
	RCC->PLLCFGR &= ~ RCC_PLLCFGR_PLLSRC_Msk;
	//RCC->CFGR |= 1<<24; // PLL_P selected as system clock

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

	RCC->PLLCFGR |= 4<<0; //PLLM
	RCC->PLLCFGR |= 168<<6; //PLLN
	//C->PLLCFGR |= 2<<0; //PLLP
	RCC->PLLCFGR |= 1<<22; //PLLSRC
	//RCC->CFGR |= RCC_CFGR_SWS_PLL;

	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLON));
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	//RCC->AHB1ENR |= 0b11111111; //вкл все порты ввода-вывода
	RCC->AHB1ENR |= 0b11<<21; //вкл DMA, на всякий :|

	SysTick_Config(SysTicks);

	timInit();

	*((uint32_t*)0x40023830)|=0b1001; // RCC_AHB1ENR GPIODEN GPIOAEN
	GPIOD->MODER |= (1<<24) | (1<<26) | (1<<28) | (1<<30); //пины leds на вывод
	//GPIOD->MODER |= 0x55000000;
	GPIOA->MODER &= ~(0b11<<0); // PA0 на вход

	GPIOD->ODR = 0xF000;
	while(28)
	{

		if ((GPIOA->IDR & (1<<0)) != 0){
			GPIOD->ODR |= 1<<15;
			delay_ms(1000);
		}
		else {
			GPIOD->ODR &= ~(1<<15);
		}

		__asm("nop");
	}
}

void timInit()
{
	TIM4->CR1 |= TIM_CR1_ARPE; // включить авто перегрузку
	TIM4->CCER |= TIM_CCER_CC1E; //вкл режим захвата/сравнения
	TIM4->CR1 |= TIM_CR1_CEN; //включение таймера
	TIM4->DIER |= TIM_DIER_CC1IE; //прерывание захвата/сравнения
	//TIM4->SR |=
}

void SysTick_Handler(void)
{
	if (delay_count>0){delay_count--;}
}

void delay_ms(uint16_t delay)
{
	delay_count = delay;
	while(delay_count) {};
}