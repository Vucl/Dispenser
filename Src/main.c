/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Denis Petrunin
 * @brief          : Main program body
 ******************************************************************************
 */

#include <stdint.h>

#define PBA 0x40000000 // peripheral Base Address
#define PBA_AHB1 (PBA+0x2000) // AHB1 PBA
#define GPIOD_BASE (PBA_AHB1+0xC00) // порт со светодиодами на ногах 12-15

typedef struct
{
	uint32_t MODER;// = 0x00; //нужен, 00==вход 01==выход
	uint32_t OTYPER;// = 0x04;
	uint32_t OSPEEDR;// = 0x08;
	uint32_t PUPDR;// = 0x0C;
	uint32_t IDR;// = 0x10; //нужен
	uint32_t ODR;// = 0x14; //нужен
	uint32_t BSRR;// = 0x18;
	uint32_t LCKR;// = 0x1C;
	uint32_t AFRL;// = 0x20; //на этом порту пока хз
	uint32_t AFRH;// = 0x24; //на этом порту пока хз
} GPIO_TypeDef;

//#define peripheralBitBandRegionBaseAdress
//#define advancedHardwareBus1PBA

int main(void)
{
	*((uint32_t*)0x40023830)|=1|4;
    /* Loop forever */
	for(;;);
}
