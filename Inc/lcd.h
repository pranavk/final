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
	

// The values of register in this file is taken from a library by Nikita Bulaev 

#ifndef LCD_HD44780_I2C_H
#define LCD_HD44780_I2C_H 120

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

#define LCD_BIT_RS                 ((uint8_t)0x01U)
#define LCD_BIT_RW                 ((uint8_t)0x02U)
#define LCD_BIT_E                  ((uint8_t)0x04U)
#define LCD_BIT_BACKIGHT_ON        ((uint8_t)0x08U)
#define LCD_BIT_BACKIGHT_OFF       ((uint8_t)0x00U)

#define LCD_MODE_4BITS             ((uint8_t)0x02U)
#define LCD_BIT_1LINE              ((uint8_t)0x00U)
#define LCD_BIT_2LINE              ((uint8_t)0x08U)
#define LCD_BIT_4LINE              LCD_BIT_2LINE
#define LCD_BIT_5x8DOTS            ((uint8_t)0x00U)
#define LCD_BIT_5x10DOTS           ((uint8_t)0x04U)
#define LCD_BIT_SETCGRAMADDR       ((uint8_t)0x40U)
#define LCD_BIT_SETDDRAMADDR       ((uint8_t)0x80U)

#define LCD_BIT_DISPLAY_CONTROL    ((uint8_t)0x08U)
#define LCD_BIT_DISPLAY_ON         ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_ON          ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_OFF         ((uint8_t)0x00U)
#define LCD_BIT_BLINK_ON           ((uint8_t)0x01U)
#define LCD_BIT_BLINK_OFF          ((uint8_t)0x00U)

#define LCD_BIT_DISP_CLEAR         ((uint8_t)0x01U)
#define LCD_BIT_CURSOR_HOME        ((uint8_t)0x02U)

#define LCD_BIT_ENTRY_MODE         ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_DIR_RIGHT   ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_DIR_LEFT    ((uint8_t)0x00U)
#define LCD_BIT_DISPLAY_SHIFT      ((uint8_t)0x01U)

// TODO: Update commands with this defines
#define LCD_BIT_CURSOR_SHIFT_MODE  ((uint8_t)0x10U)
#define LCD_BIT_CURSOR_DISP_SHIFT  ((uint8_t)0x08U)
#define LCD_BIT_CURSOR_MOVE        ((uint8_t)0x00U)
#define LCD_BIT_CURSOR_SHIFT_DIR_R ((uint8_t)0x40U)
#define LCD_BIT_CURSOR_SHIFT_DIR_L ((uint8_t)0x00U)


/* Function defines */
#define lcdBacklightOn()           lcdBacklight(LCD_BIT_BACKIGHT_ON)
#define lcdBacklightOff()          lcdBacklight(LCD_BIT_BACKIGHT_OFF)
#define lcdAutoscrollOn()          lcdCommand(LCD_DISPLAY_SHIFT, LCD_PARAM_SET)
#define lcdAutoscrollOff()         lcdCommand(LCD_DISPLAY_SHIFT, LCD_PARAM_UNSET)
#define lcdDisplayClear()          lcdCommand(LCD_CLEAR, LCD_PARAM_SET)
#define lcdDisplayOn()             lcdCommand(LCD_DISPLAY, LCD_PARAM_SET)
#define lcdDisplayOff()            lcdCommand(LCD_DISPLAY, LCD_PARAM_UNSET)
#define lcdCursorOn()              lcdCommand(LCD_CURSOR, LCD_PARAM_SET)
#define lcdCursorOff()             lcdCommand(LCD_CURSOR, LCD_PARAM_UNSET)
#define lcdBlinkOn()               lcdCommand(LCD_CURSOR_BLINK, LCD_PARAM_SET)
#define lcdBlinkOff()              lcdCommand(LCD_CURSOR_BLINK, LCD_PARAM_UNSET)
#define lcdCursorDirToRight()      lcdCommand(LCD_CURSOR_DIR_RIGHT, LCD_PARAM_SET)
#define lcdCursorHome()            lcdCommand(LCD_CURSOR_HOME, LCD_PARAM_SET)

#ifndef bool
typedef enum {
    false,
    true
} bool;
#endif

typedef struct {
    I2C_HandleTypeDef * hi2c;  // I2C Struct
    uint8_t address;           // I2C address shifted left by 1
    uint8_t backlight;         // Backlight
    uint8_t modeBits;          // Display on/off control bits
    uint8_t entryBits;         // Entry mode set bits
} LCDParams;

typedef enum {
    LCD_PARAM_UNSET = 0,
    LCD_PARAM_SET
} LCDParamsActions;

typedef enum {
    LCD_BACKLIGHT = 0,
    LCD_DISPLAY,
    LCD_CLEAR,
    LCD_CURSOR,
    LCD_CURSOR_BLINK,
    LCD_CURSOR_HOME,
    LCD_CURSOR_DIR_LEFT,
    LCD_CURSOR_DIR_RIGHT,
    LCD_DISPLAY_SHIFT
} LCDCommands;


bool lcdInit(I2C_HandleTypeDef *hi2c, uint8_t address);
bool lcdCommand(LCDCommands command, LCDParamsActions action);
bool lcdBacklight(uint8_t command);
bool lcdSetCursorPosition(uint8_t line, uint8_t row);
bool lcdPrintPrimaryStr(char *data);
bool lcdPrintSecondaryStr(char *data);
bool lcdPrintChar(uint8_t data);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
