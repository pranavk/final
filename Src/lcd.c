/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : Header for motor.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stdlib.h"
#include "string.h"
#include "assert.h"

#include "lcd.h"

uint8_t lcdCommandBuffer[6] = {0x00};

static LCDParams lcdParams;

// Write @Data according to the format it expects
static bool write4Bits(uint8_t Data, bool isData) {
	// Data goes in upper 4 bits; lower 4 bits contain control information
	Data <<= 4;
	// Backlight is always on:
	Data |= lcdParams.backlight;
	
	// We are writing
	Data &= ~LCD_BIT_RW;
	
	// RS pin on?
	if (isData) {
		Data |= LCD_BIT_RS;
	} else
		Data &= ~LCD_BIT_RS;
		
	// Turn the Enable bit on
	Data |= LCD_BIT_E;
	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address, &Data, 1, 100000) != HAL_OK) return false;
	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address, &Data, 1, 100000) != HAL_OK) return false;

	// Enable off
	Data &= ~LCD_BIT_E;
	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address, &Data, 1, 100000) != HAL_OK) return false;
	return true;
}

static bool writeByte(uint8_t Data, bool isData) {
	if (!write4Bits((Data >> 4) & 0x0F, isData)) return false;
	if (!write4Bits((Data) & 0x0F, isData)) return false;
	return true;
}

bool lcdInit(I2C_HandleTypeDef *hi2c, uint8_t address) {
    lcdParams.hi2c      = hi2c;
    lcdParams.address   = address << 1;
    lcdParams.backlight = LCD_BIT_BACKIGHT_ON;
		
    for (uint8_t i = 0; i < 3; ++i) {
				if (!write4Bits(0x03, false)) return false;
		
        while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
						HAL_Delay(1);
        }

				switch(i) {
					case 0: HAL_Delay(5); break; // 4.1 ms delay according to datasheet
					case 1: HAL_Delay(5); break; // 100 us delay
					case 2: HAL_Delay(1); break; // no delay mentioned but let's be conservative
				}
    }

		// start using 4 bit interface
		write4Bits(LCD_MODE_4BITS, false);

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
			HAL_Delay(1);
    }

		writeByte(LCD_MODE_4BITS | LCD_BIT_2LINE, false);
		while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
			HAL_Delay(1);
    }
    
		/* Now lets set display, cursor and blink all on */
		HAL_Delay(5);
		writeByte(LCD_BIT_DISPLAY_CONTROL | LCD_BIT_DISPLAY_ON, false);

    /* LTR cursor */
		writeByte(LCD_BIT_ENTRY_MODE | LCD_BIT_CURSOR_DIR_RIGHT, false);

    /* Clear display and Set cursor at Home */
		writeByte(LCD_BIT_DISP_CLEAR, false);
		writeByte(LCD_BIT_CURSOR_HOME, false);

		return true;
}

/**
 * @brief  Send command to display
 * @param  command  One of listed in LCDCommands enum
 * @param  action   LCD_PARAM_SET or LCD_PARAM_UNSET
 * @return          true if success
 */
bool lcdCommand(LCDCommands command, LCDParamsActions action) {
    uint8_t lcdData = 0x00;

    /* First of all lest store the command */
    switch (action) {
        case LCD_PARAM_SET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits |=  LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits |= LCD_BIT_CURSOR_ON;
                    break;

                case LCD_CLEAR:
                    lcdData = LCD_BIT_DISP_CLEAR;

                    if (writeByte(lcdData, false) == false) {
                        return false;
                    } else {
                        //vTaskDelay(2);
												HAL_Delay(2);
                        return true;
                    }

                case LCD_CURSOR_HOME:
                    lcdData = LCD_BIT_CURSOR_HOME;

                    if (writeByte(lcdData, false) == false) {
                        return false;
                    } else {
                        //vTaskDelay(2);
												HAL_Delay(2);
                        return true;
                    }

                default:
                    return false;
            }

            break;

        case LCD_PARAM_UNSET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits &= ~LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits &= ~LCD_BIT_CURSOR_ON;
                    break;

                default:
                    return false;
            }

            break;

        default:
            return false;
    }

    /* Now lets send the command */
    switch (command) {
        case LCD_DISPLAY:
        case LCD_CURSOR:
        case LCD_CURSOR_BLINK:
            lcdData = LCD_BIT_DISPLAY_CONTROL | lcdParams.modeBits;
            break;

        default:
            break;
    }

    return writeByte(lcdData, false);
}

/**
 * @brief  Turn display's Backlight On or Off
 * @param  command LCD_BIT_BACKIGHT_ON to turn display On
 *                 LCD_BIT_BACKIGHT_OFF (or 0x00) to turn display Off
 * @return         true if success
 */
bool lcdBacklight(uint8_t command) {
		writeByte(command, false);
    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
				HAL_Delay(1);
    }

    return true;
}

/**
 * @brief  Set cursor position on the display
 * @param  column counting from 0
 * @param  line   counting from 0
 * @return        true if success
 */
bool lcdSetCursorPosition(uint8_t column, uint8_t line) {
    // We will setup offsets for 4 lines maximum
    static const uint8_t lineOffsets[4] = { 0x00, 0x40, 0x14, 0x54 };

		if (line > 1) line = 1;
		
    uint8_t lcdCommand = LCD_BIT_SETDDRAMADDR | (column + lineOffsets[line]);

    return writeByte(lcdCommand, false);
}

/**
 * @brief  Print string from cursor position
 * @param  data   Pointer to string
 * @param  length Number of symbols to print
 * @return        true if success
 */
bool lcdPrintPrimaryStr(char* data) {
		// we don't want to print more than 16 characters
		assert(strlen(data) <= 16);
	
		lcdSetCursorPosition(0, 0);
	
		uint8_t length = strlen(data);
    for (uint8_t i = 0; i < length; ++i) {
        if (writeByte(data[i], true) == false) {
            return false;
        }
    }
    return true;
}

bool lcdPrintSecondaryStr(char* data) {
		// we don't want to print more than 16 characters
		assert(strlen(data) <= 16);
	
		lcdSetCursorPosition(0, 1);

		uint8_t length = strlen(data);
    for (uint8_t i = 0; i < length; ++i) {
        if (writeByte(data[i], true) == false) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  Print single char at cursor position
 * @param  data Symbol to print
 * @return      true if success
 */
bool lcdPrintChar(uint8_t data) {
    return writeByte(data, true);
}
