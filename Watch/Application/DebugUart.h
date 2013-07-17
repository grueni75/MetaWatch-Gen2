//==============================================================================
//  Copyright 2011 Meta Watch Ltd. - http://www.MetaWatch.org/
// 
//  Licensed under the Meta Watch License, Version 1.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//  
//      http://www.MetaWatch.org/licenses/license-1.0.html
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//==============================================================================

/******************************************************************************/
/*! \file DebugUart.h
 *
 * The debug uart is used to send messages to a terminal. The settings
 * are 115200, 8 bits, no parity, 1 stop bit
 *
 * This only is useful on the development board and modified watches.
 */
/******************************************************************************/

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

// Critical code identification numbers
#if TRACK_CRITICAL_CODE
#define CC_IDLE_TASK 1
#define CC_BUTTON_PORT_ISR 2
#define CC_DEBUG_UART_ISR 3
#define CC_TASK_CHECKIN 4
#define CC_START_TIMER 5
#define CC_DMA_ISR 6
#define CC_CTS_ISR 7
#define CC_ACCELEROMETER 8
#define CC_ACCELEROMETER_ISR 9
#define CC_ENABLE_SM_CLK_USER 10
#define CC_DISABLE_SM_CLK_USER 11
#define CC_ENABLE_RTC 12
#define CC_DISABLE_RTC 13
#define CC_RTC_ISR 14
#define CC_ADD_USER 15
#define CC_REMOVE_USER 16
#define CC_RTOS_TIMER_ISR 17
#define CC_SOFTWARE_FLL_ISR1 18
#define CC_SOFTWARE_FLL_ISR2 19
// Must be the last one
#define CC_FREERTOS 20
extern unsigned int LastCriticalCode;
#define LAST_CRITICAL_CODE(nr) { LastCriticalCode =nr; }
#else
#define LAST_CRITICAL_CODE(nr)
#endif

extern const char OK[];
extern const char NOK[];
extern const char SPACE;
extern const char ZERO;

void EnableDebugUart(unsigned char Enable);
void EnableTimeStamp(void);

/******************************************************************************/

/*! Print a  character */
void PrintC(char Char);
void PrintR(void);
void PrintH(unsigned char Value);
void PrintS(const char *pString);
void PrintF(const char *pFormat, ...);

#endif
