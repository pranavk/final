#include "motor.h"

uint32_t debouncer = 0;

void testButtonSetup(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable peripheral clock for GPIOA
	GPIOA->MODER 		&= ~((1 >> 0) | (1 >> 1)); // set digital input mode for user button
	GPIOA->OSPEEDR	|= ~((1 >> 0) | (1 >> 1)); // set low speed for user button
	GPIOA->PUPDR 		|= (1 << 1); // set pull-down resistor for user button
}

void motorPinSetup(void)
{
	// setup pin PC1 to enable; PC2 to IN1; PC3 to IN2
	
	// Enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable peripheral clock for GPIOC
	
	// Clear GPIOC MODER for enable, 
	GPIOC->MODER &= 0xFFFFFF03;
	
	// Enable pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER13_Pos); // set output mode for enable pin output
	GPIOC->OTYPER		&= (1 << 13); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(3 << GPIO_OSPEEDR_OSPEEDR13_Pos); // set low speed for enable pin output
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR13_Pos); // set pull-down resistor for enable pin output
	GPIOC->BRR		|= (1 << 13); // initially set h bridge enable pin low; 

	// IN1 pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER14_Pos); // set output mode for h bridge input 1 out (STM32 output)
	GPIOC->OTYPER		&= (1 << 14); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(3 << GPIO_OSPEEDR_OSPEEDR14_Pos); // set low speed for h bridge input 1 out (STM32 output)
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR14_Pos); // set pull-down resistor for h bridge input 1 out (STM32 output)
	GPIOC->BSRR		|= (1 << 14); // initially set h bridge input 1 pin high;

	// IN2 pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER15_Pos); // set output mode for h bridge input 2 out (STM32 output)
	GPIOC->OTYPER		&= (1 << 15); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(1 << GPIO_OSPEEDR_OSPEEDR15_Pos); // set low speed for h bridge input 2 out (STM32 output)
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR15_Pos); // set pull-down resistor for h bridge input 2 out (STM32 output)
	GPIOC->BRR		|= (1 << 15); // initially set h bridge input 2 pin low;
}

void enablePinHigh(void)
{
	GPIOC->BSRR		|= (1 << 13); // set h bridge enable pin high;
	//sendStr("Enable pin set high. ODR value for pin 1:");
	//sendInt(GPIOC->ODR & (1 << 13));
}

void enablePinLow(void)
{
	GPIOC->BRR		|= (1 << 13); // set h bridge enable pin low;
	//sendStr("Enable pin set low. ODR value for pin 1:");
	//sendInt(GPIOC->ODR & (1 << 1));
}

void setMotorDirectionForward(void)
{
	GPIOC->BSRR		|= (1 << 14); // set h bridge input 1 pin high; 
	GPIOC->BRR		|= (1 << 15); // set h bridge input 2 pin low;
	//sendStr("Motor set to forward. ODR value for pin 2:");
	//sendInt(GPIOC->ODR & (1 << 14));
	//sendStr("Motor set to forward. ODR value for pin 3:");
	//sendInt(GPIOC->ODR & (1 << 15));
}

void setMotorDirectionBackward(void)
{
	GPIOC->BRR		|= (1 << 14); // set h bridge input 1 pin low; 
	GPIOC->BSRR		|= (1 << 15); // set h bridge input 2 pin high; 
	//sendStr("Motor set to backward. ODR value for pin 2:");
	//sendInt(GPIOC->ODR & (1 << 14));
	//sendStr("Motor set to backward. ODR value for pin 3:");
	//sendInt(GPIOC->ODR & (1 << 15));
}

void motorForward(void)
{
	setMotorDirectionForward();
	enablePinHigh();
}

void motorBackward(void)
{
	setMotorDirectionBackward();
	enablePinHigh();
}

void motorStop(void)
{
	enablePinLow();
}

void motorButton(void)
{
	debouncer = (debouncer << 1); // Always shift every loop iteration
	
	if (GPIOA->IDR & 0x00000001) // If input signal is set/high
	{ 
		debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) { // If high, do this
		motorForward();
	}
	
	if (debouncer == 0x00000000) { // If low, do this
		motorStop();
	}
}


void motorTest(void)
{
	//motorForward();
//	HAL_Delay(3000);
	//motorStop();
//	HAL_Delay(1000);
	motorBackward();
	//HAL_Delay(3000);
	//motorStop();
}

