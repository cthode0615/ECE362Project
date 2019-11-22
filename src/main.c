/**
  ******************************************************************************
  * @file    main.c
  * @author  oldham2
  * @version V1.0
  * @date    23 October 2019
  * @brief   Main function for STM32 Slot machine
  ******************************************************************************
*/
/*PINS
 * A0: OEN
 * A1: CLK
 * A2: LATCH
 * A3: R1
 * A4: G1
 * A5: B1
 * A6: R2
 * A7: G2
 * A8: B2
 * A9: A
 * A10: B
 * A11: C
 *
 * LED MATRIX PINOUT
 * R1	G1
 * B1	GND
 * R2	G2
 * B2	GND
 * A 	B
 * C 	GND
 * CLK  LAT
 * OE   GND
 */

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include "imagedata.h"
#include <string.h>

unsigned char  imagedata[32*3*16] = {0};


void picture_blank()
{
	for(int x = 0; x < 32*3; x++)
	{
		for(int y = 0; y < 16; y++)
		{
			imagedata[(32*3*y)+x] = 0;
		}
	}
}


void load_bg()
{
	memcpy(imagedata, bgimagedata, 32*16*3);
}

void printSprite(int spritenum, int x, int y)
{
	switch(spritenum)
	{
	case 0:
	{

		for(int i = 0; i < 8; i++)
		{
			memcpy(&imagedata[(32*3*(y+i))+x], &heartsprite[8*3*i], 8*3);
		}

		break;
	}
	case 1:
		for(int i = 0; i < 8; i++)
		{
			memcpy(&imagedata[(32*3*(y+i))+x], &coinsprite[8*3*i], 8*3);
		}
	}
}


void init_GPIO()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= 0x555555;
}

void init_tim6()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 10-1;
    TIM6->ARR = 5-1;
    TIM6->CR1 |= 1;
    TIM6->DIER |= 1;
    NVIC->ISER[0] |= 1 << (TIM6_DAC_IRQn);
    NVIC->IP[4]= 0x0000ff00; //Timer 6 has lower priority than timer 3
}

void init_tim3()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 8000-1;
	TIM3->ARR = 100-1;
	TIM3->CR1 |= 1;
	TIM3->DIER |= 1;
	NVIC->ISER[0] |= 1 << (TIM3_IRQn);
}

/*
 * The timer 6 interrupt handler updates the display 2 lines at a time using
 * the data in imagedata
 */
void TIM6_DAC_IRQHandler()
{
	static int imageline = 0;
	GPIOA->ODR &= ~(0xe00); //Clear bits 9-11 for ABC
    GPIOA->ODR |= 1; //OEN
	GPIOA->ODR |= (imageline & 1) << 9;
	GPIOA->ODR |= ((imageline & 2) >> 1) << 10;
	GPIOA->ODR |= ((imageline & 4) >> 2) << 11;
    for(int x = 0; x < 32*3; x+=3)
    {
    	GPIOA->ODR &= ~(0x1f8); //clear bits 3-8 for RGB
    	GPIOA->ODR |= (1 & (imagedata[(imageline *3 * 32)+x])) << 3;
    	GPIOA->ODR |= (1 & (imagedata[(imageline *3 * 32)+x+1])) << 4;
    	GPIOA->ODR |= (1 & (imagedata[(imageline *3 * 32)+x + 2])) << 5;
    	GPIOA->ODR |= (1 & (imagedata[((imageline+8) *3 * 32)+x])) << 6;
    	GPIOA->ODR |= (1 & (imagedata[((imageline+8) *3 * 32)+x+1])) << 7;
    	GPIOA->ODR |= (1 & (imagedata[((imageline+8) *3 * 32)+x + 2])) << 8;

    	GPIOA->ODR |= 0x2; //Clock high
    	GPIOA->ODR &= ~(0x2); //Clock low
    }
    GPIOA->ODR |= 0x4; //Latch high
    GPIOA->ODR &= ~(0x4); //Latch low
    GPIOA->ODR &= ~(1);
    imageline +=1;
    if(imageline > 7) imageline = 0;
    TIM6->SR &= !(1);
}

void TIM3_IRQHandler()
{

	int sprite1 = 0;
	int sprite2 = 1;
	int sprite3 = 0;
	static int yshift = 4;

	load_bg();
	printSprite(sprite1, 2*3, yshift);
	printSprite(sprite2, 12*3, yshift);
	printSprite(sprite3, 22*3, yshift);
	yshift += 1;
	if(yshift > 16) yshift = 0;
	TIM3->SR &= ~(1);

}

//BEGIN SPI/LCD DISPLAY SECTION
//global variables:
int scrollcounter = 0;
int difficulty = 0;
const char *select_diff = "****************Select difficulty level****************";

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

void spi_cmd(char b) {
    while((SPI2->SR & SPI_SR_TXE) == 0);     //wait for TXE to be 1
    SPI2->DR = b;
}

void spi_data(char b) {
    while((SPI2->SR & SPI_SR_TXE) == 0);     //wait for TXE to be 1
    SPI2->DR = 0x200 + b;
}

void generic_lcd_startup(void) {
    nano_wait(100000000); // Give it 100ms to initialize
    spi_cmd(0x38);  // 0011 NF00 N=1, F=0: two lines
    spi_cmd(0x0c);  // 0000 1DCB: display on, no cursor, no blink
    spi_cmd(0x01);  // clear entire display
    nano_wait(6200000); // clear takes 6.2ms to complete
    spi_cmd(0x02);  // put the cursor in the home position
    spi_cmd(0x06);  // 0000 01IS: set display to increment
}

void spi_init_lcd(void) {
    //CONFIGURE GPIO PINS
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    //set PB12,13,15 to alternate function
    GPIOB->MODER |= GPIO_Mode_AF<<(2*12);
    GPIOB->MODER |= GPIO_Mode_AF<<(2*13);
    GPIOB->MODER |= GPIO_Mode_AF<<(2*15);
    //alternate functions should already be configured (AF 0000 = SPI)

    //CONFIGURE SPI2 REGISTERS
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 |= SPI_CR1_BIDIMODE;  //bidirectional mode
    SPI2->CR1 |= SPI_CR1_BIDIOE;    //bidirectional output enabled
    SPI2->CR1 |= SPI_CR1_MSTR;      //set as master
    SPI2->CR1 |= SPI_CR1_BR_2;      //slowest possible baud rate (BR=111, Baud rate = 187.5 kHz)
    SPI2->CR1 |= SPI_CR1_BR_1;
    SPI2->CR1 |= SPI_CR1_BR_0;
    SPI2->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0 | SPI_CR2_SSOE | SPI_CR2_NSSP;   //10 bit word size (DS = 1001)
    SPI2->CR1 |= SPI_CR1_SPE;       //enable the channel

    generic_lcd_startup();

}

void spi_display1(const char *s) {
    //Displays a string on the first line of the LCD
    spi_cmd(0x80 + 0);
    int x;
    for(x=0; x<16; x+=1)
        if (s[x])
            spi_data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        spi_data(' ');
}

void spi_display2(const char *s) {
    //Displays a string on the second line of the LCD
    spi_cmd(0x80 + 64);
    int x;
    for(x=0; x<16; x+=1)
        if (s[x])
            spi_data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        spi_data(' ');
}

void spi_char_display2(char b)
{
    //Displays a single character on the second line of the LCD
    spi_cmd(0x80 + 64);
    spi_data(b);
}

void spi_scroll1(const char *msg)
{
    //Every time this function runs, it steps through 1 step of the scroll
    //Therefore msg will continue scrolling as long as spi_scroll1 is looping
    //The scroll appears on line 2 of the LCD
    spi_display1(&msg[scrollcounter]);
    nano_wait(100000000);
    scrollcounter++;
    if(scrollcounter >= 40)
        scrollcounter = 0;
}

void init_button(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(3 << (2*7));      //set PC7 to input
}

void init_exti(void)
{
    //Trigger an interrupt on the rising edge of PC7 (button)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PC;
    EXTI->RTSR |= EXTI_RTSR_TR7;
    EXTI->IMR |= EXTI_IMR_MR7;
    NVIC->ISER[0] |= 1<<EXTI4_15_IRQn;
}

void EXTI4_15_IRQHandler(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    EXTI->PR |= EXTI_PR_PR7;

    difficulty++;
    if(difficulty > 5)
        difficulty = 1;
    spi_display1("Level selected:");
    spi_char_display2((char) (difficulty + 48));

}



int main(void)
{
	picture_blank();
	init_GPIO();
	init_tim6();
	init_tim3();
    	spi_init_lcd();
    	init_button();
    	init_exti();	
	
	while((GPIOC->IDR & GPIO_IDR_7) != GPIO_IDR_7)
        	spi_scroll1(select_diff);
	while(1){
		asm("nop");
	}
}
