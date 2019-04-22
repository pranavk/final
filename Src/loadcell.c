#include "loadcell.h"

static char pulsesAfter = 1;
static int load_offset = 0;

// Clock is PC6; Data is pC7
uint32_t HXGetValue() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		HAL_Delay(1);
	}
	
	// run the clock
	uint32_t data = 0;
	for (int i = 0; i < 24; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		data = data << 1;
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
			data |= 0x1;
		}
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	
	// yes, we want to keep reading from this channel; so, spit out the 25th cycle.
	for (int i = 0; i < pulsesAfter; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	return data - load_offset;
}

uint32_t HXGetAvgValue() {
	return HXGetValue();
}

// Tare
void HX711_Tare(void) {
	uint32_t data = HXGetAvgValue();
	load_offset = data;
}

void HX711_Init(LoadMode mode) {
	if (mode == CHA_128) {
		pulsesAfter = 1;
	} else if (mode == CHB_32) {
		pulsesAfter = 2;
	} else if (mode == CHA_64) {
		pulsesAfter = 3;
	}
	HXGetValue();

	HX711_Tare();
}

