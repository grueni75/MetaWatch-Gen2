//==============================================================================
//  Copyright 2013 Meta Watch Ltd. - http://www.MetaWatch.org/
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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"

#include "Messages.h"
#include "MessageQueues.h"

#include "hal_lpm.h"
#include "hal_board_type.h"
#include "hal_miscellaneous.h"
#include "hal_calibration.h"
#include "hal_boot.h"
#include "hal_rtos_timer.h"

#include "DebugUart.h"
#include "Wrapper.h"
#include "Utilities.h"
#include "IdleTask.h"

#if __IAR_SYSTEMS_ICC__
__no_init __root tWatchdogInfo WatchdogInfo @ WATCHDOG_INFO_ADDR;
__no_init __root unsigned int niWdtCounter @ WATCHDOG_COUNTER_ADDR;
#else
extern tWatchdogInfo WatchdogInfo;
extern unsigned int niWdtCounter;
#endif

extern unsigned int niReset;
extern unsigned char niWrapperCurrentMsgType;
extern unsigned char niDisplayCurrentMsgType;

static void PrintResetSource(unsigned int Source);

/*
static unsigned char enableTaskTrace = 0;
static unsigned char taskTraceEnabled = 0;
#define TASK_TRACE_BUFFER_SIZE (1024)
static unsigned int taskTraceBufferUsedSize = 0;
static char taskTraceBuffer[TASK_TRACE_BUFFER_SIZE];
static unsigned char enableTaskStateListing = 0;
static char taskStateBuffer[128];
*/

void vApplicationIdleHook(void)
{

  /* Put the processor to sleep if the serial port indicates it is OK and
   * all of the queues are empty.
   * This will stop the OS scheduler.
   */ 

  /* enter a critical section so that the flags can be checked */
  __disable_interrupt();
  __no_operation();
  LAST_CRITICAL_CODE(CC_IDLE_TASK);
  CODE_START(idleTaskCriticalSection);

  /* the watchdog is set at 16 seconds.
   * the battery interval rate is set a 10 seconds
   * each task checks in at the battery interval rate
   */
  UpdateWatchdogInfo();

  /* Support tracing of tasks
  if ((enableTaskTrace)&&(!taskTraceEnabled)) {
    vTaskStartTrace(taskTraceBuffer,TASK_TRACE_BUFFER_SIZE);
    taskTraceEnabled=1;
  }
  if ((!enableTaskTrace)&&(taskTraceEnabled)) {
    taskTraceBufferUsedSize=ulTaskEndTrace();
    taskTraceEnabled=0;
  }*/

  /* Store information of a task
  if (enableTaskStateListing)
    vTaskList(taskStateBuffer);*/

#if SUPPORT_LPM
  if (WatchdogInfo.SppReadyToSleep &&
      WatchdogInfo.DisplayMessagesWaiting == 0 &&
      WatchdogInfo.SppMessagesWaiting == 0)
  {
    /* Call MSP430 Utility function to enable low power mode 3.     */
    /* Put OS and Processor to sleep. Will need an interrupt        */
    /* to wake us up from here.   */
//    DISABLE_LCD_LED();
    EnterLpm3();
//    ENABLE_LCD_LED();

  __enable_interrupt();
  __no_operation();
    /* If we get here then interrupts are enabled */
    return;
  }
#endif

  /* workaround: reenable tick count if it has stopped
  unsigned int CurrentTickCount = GetTickCount();
  unsigned int NextTickCount = TA0CCR0;
  if ((NextTickCount-CurrentTickCount)>1)
    PrintS("Unexpected RTOS tick value");*/

  /* we aren't going to sleep so enable interrupts */
  CODE_END(idleTaskCriticalSection);
  __enable_interrupt();
  __no_operation();
}

/******************************************************************************/

/* 8 us */
void UpdateWatchdogInfo(void)
{
  WatchdogInfo.SppReadyToSleep = SerialPortReadyToSleep();

  WatchdogInfo.DisplayMessagesWaiting =
    QueueHandles[DISPLAY_QINDEX]->uxMessagesWaiting;

  WatchdogInfo.SppMessagesWaiting = 
    QueueHandles[WRAPPER_QINDEX]->uxMessagesWaiting;
}

void ShowWatchdogInfo(void)
{
  if (niReset == FLASH_RESET_CODE) {
    niWdtCounter = 0;
    niWrapperCurrentMsgType = 0;
    niDisplayCurrentMsgType = 0;
  }

  unsigned int ResetSource = GetResetSource();
  PrintResetSource(ResetSource);
  PrintF("SppReadyToSleep %d", WatchdogInfo.SppReadyToSleep);
  PrintF("DisplayMsgWaiting %d", WatchdogInfo.DisplayMessagesWaiting);
  PrintF("SppMsgWaiting %d", WatchdogInfo.SppMessagesWaiting);

  if (ResetSource == SYSRSTIV_WDTTO || ResetSource == SYSRSTIV_WDTKEY)
  {
    PrintF("# WDT %s", ResetSource == SYSRSTIV_WDTTO ? "Failsafe" : "Forced");
    niWdtCounter ++;
  }
  
  PrintF("Total Watchdogs: %d", niWdtCounter);

  if (niWrapperCurrentMsgType!=0)
    PrintF("Wrapper processed message type 0x%02x",niWrapperCurrentMsgType);
  if (niDisplayCurrentMsgType!=0)
    PrintF("Display processed message type 0x%02x",niDisplayCurrentMsgType);
  niWrapperCurrentMsgType = 0;
  niDisplayCurrentMsgType = 0;
}

void ResetWatchdog(void)
{
  /* Turn off the watchdog timer
  WDTCTL = WDTPW + WDTHOLD;
  return;*/

  /* set watchdog for 16 second timeout
   * write password, select aclk, WDTIS_3 means divide by 512*1024 = 16 s;
   * WDTIS_2: 4 mins 
   */
#if USE_FAILSAFE_WATCHDOG
  
  WDTCTL = WDTPW + WDTCNTCL + WDTSSEL__ACLK + WDTIS_3;
  SFRIE1 &= ~WDTIE;
  
#else
  
  WDTCTL = WDTPW + WDTCNTCL + WDTSSEL__ACLK + WDTIS_3 + WDTTMSEL;

  /* enable watchdog timer interrupt */
  SFRIE1 |= WDTIE;

#endif
}

#define WATCHDOG_LED_DELAY() { __delay_us(2000000); }

/* this is for unrecoverable errors */
void WatchdogReset(void)
{
  __disable_interrupt();

#if USE_LED_FOR_WATCHDOG_DEBUG
  ENABLE_LCD_LED();
  WATCHDOG_LED_DELAY();
#endif
  
#if USE_FAILSAFE_WATCHDOG
  while(1);
#else
  /* write the inverse of the password and cause a reset */
  WDTCTL = ~WDTPW;
#endif
}

/******************************************************************************/

/* the timer mode is used when the option USE_FAILSAFE_WATCHDOG == 0 
 * this is for debugging only
 */

#ifndef __IAR_SYSTEMS_ICC__
#pragma CODE_SECTION(WatchdogTimerIsr,".text:_isr");
#endif

#ifndef BOOTLOADER
#pragma vector=WDT_VECTOR
__interrupt void WatchdogTimerIsr(void)
#else
void WatchdogTimerIsr(void)
#endif
{
  /* add your debug code here */
  __no_operation();
  
#if USE_LED_FOR_WATCHDOG_DEBUG
  ENABLE_LCD_LED();
  WATCHDOG_LED_DELAY();
#endif

  // BOR reset
//  PMMCTL0 = PMMPW | PMMSWBOR;
  /* write the inverse of the password and cause a PUC reset */
  WDTCTL = ~WDTPW; 
}

/******************************************************************************/

/* the clearing of the flags cannot be done in the idle loop because
 * it may be interrupted
 */
void TaskCheckIn(etTaskCheckInId TaskId)
{
  static unsigned char TaskCheckInFlags = 0;

  portENTER_CRITICAL();
  LAST_CRITICAL_CODE(CC_TASK_CHECKIN);
  
  TaskCheckInFlags |= (1 << TaskId);

  if (TaskCheckInFlags == ALL_TASKS_HAVE_CHECKED_IN)
  {
    /* all tasks have checked in - so the flags can be cleared
     * and the watchdog can be kicked
     */
    TaskCheckInFlags = 0;
    ResetWatchdog();
  }
  
  portEXIT_CRITICAL();
}

/* PrintS( reset code and the interrupt type */
static void PrintResetSource(unsigned int Source)
{  
  PrintF("ResetSource 0x%02X", Source);

#if 0
  PrintS(" - ");
  switch (Source)
  {
  case 0x0000: PrintS("No interrupt pending"); break;
  case 0x0002: PrintS("Brownout (BOR) (highest priority)"); break;
  case 0x0004: PrintS("RST/NMI (BOR)"); break;
  case 0x0006: PrintS("PMMSWBOR (BOR)"); break;
  case 0x0008: PrintS("Wakeup from LPMx.5 (BOR)"); break;
  case 0x000A: PrintS("Security violation (BOR)"); break;
  case 0x000C: PrintS("SVSL (POR)"); break;
  case 0x000E: PrintS("SVSH (POR)"); break;
  case 0x0010: PrintS("SVML_OVP (POR)"); break;
  case 0x0012: PrintS("SVMH_OVP (POR)"); break;
  case 0x0014: PrintS("PMMSWPOR (POR)"); break;
  case 0x0016: PrintS("WDT time out (PUC)"); break;
  case 0x0018: PrintS("WDT password violation (PUC)"); break;
  case 0x001A: PrintS("Flash password violation (PUC)"); break;
  case 0x001C: PrintS("PLL unlock (PUC)"); break;
  case 0x001E: PrintS("PERF peripheral/configuration area fetch (PUC)"); break;
  case 0x0020: PrintS("PMM password violation (PUC)"); break;
  default:     PrintS("Unknown"); break;
  }
#endif
}
