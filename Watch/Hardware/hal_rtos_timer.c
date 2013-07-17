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
/*! \file hal_rtos_timer.c
*
* This also includes the crystal timers.
*/
/******************************************************************************/

#include "FreeRTOS.h"

#include "hal_board_type.h"
#include "hal_rtos_timer.h"
#include "hal_crystal_timers.h"
#include "hal_lpm.h"

#include "DebugUart.h"

/* these are shared with the assembly code */
unsigned char RtosTickEnabled = 0;
unsigned int RtosTickCount = RTOS_TICK_COUNT;

static unsigned char (*pCrystalCallback1)(void);
static unsigned char (*pCrystalCallback2)(void);
static unsigned char (*pCrystalCallback3)(void);
static unsigned char (*pCrystalCallback4)(void);

static unsigned char Timer0Users;

static unsigned char TimerInterruptActive=0;

#define TIMER0_RTOS_USER ( 0 )
#define TIMER0_USER1     ( 1 )
#define TIMER0_USER2     ( 2 )
#define TIMER0_USER3     ( 3 )
#define TIMER0_USER4     ( 4 )

static void AddUser(unsigned char User, unsigned int Ticks);
static void RemoveUser(unsigned char User);

/******************************************************************************/

/* Synchronize read of timer using MajorityVote
 *
 * When the timer clock is asynchronous to the CPU clock, any read from TAxR 
 * should occur while the timer is not operating or the results may be 
 * unpredictable. Alternatively, the timer may be read multiple times while 
 * operating, and a majority vote taken in software to determine the correct 
 * reading. Any write to TAxR takes effect immediately.
 */

static inline unsigned int GetTickCount(void)
{
  unsigned int value1;
  unsigned int value2 = TA0R;
  
  /* Majority Vote
   * this copies last value and does 1 read each loop
   * alternate would be to do 2 reads each loop
   */
  do
  {
    value1 = value2;
    value2 = TA0R;
  } while (value1 != value2);
  
  return value1;
}

/******************************************************************************/

#if PROFILE_PROCESSING_TIME

static tProcessingTime ProcessingTimes[CODE_COUNT];

void CodeStart(eCodePieces CodePiece)
{
  unsigned int ticks = GetTickCount();
  unsigned int diff = ticks - ProcessingTimes[CodePiece].PrevStartTime;
  if (diff<ProcessingTimes[CodePiece].MinCallDistance)
    ProcessingTimes[CodePiece].MinCallDistance=diff;
  ProcessingTimes[CodePiece].PrevStartTime=ticks;
}

void CodeEnd(eCodePieces CodePiece)
{
  unsigned int ticks = GetTickCount();
  unsigned int diff = ticks - ProcessingTimes[CodePiece].PrevStartTime;
  if (diff>ProcessingTimes[CodePiece].MaxProcessingTime)
    ProcessingTimes[CodePiece].MaxProcessingTime=diff;
  ProcessingTimes[CodePiece].PrevStartTime=ticks;
}

#endif

/******************************************************************************/

/*
 * Setup timer to generate the RTOS tick
 */
void SetupRtosTimer(void)
{
#if PROFILE_PROCESSING_TIME
  /* Init structure */
  int i;
  for (i=0;i<CODE_COUNT;i++) {
    ProcessingTimes[i].PrevStartTime=0;
    ProcessingTimes[i].MaxProcessingTime=0;
    ProcessingTimes[i].MinCallDistance=0xFFFF;
  }
#endif

  /* Ensure the timer is stopped */
  TA0CTL = 0;
  
  /* Clear everything to start with */
  TA0CTL |= TACLR;
  
  /* divide clock by 8
   * the total divide is by 32
   * 32768 kHz -> 1024 kHz -> 0.9765625 ms
   */
  TA0EX0 = 0x7;
  Timer0Users = 0;
  EnableRtosTick();

#if PROFILE_PROCESSING_TIME
  /* Ensure that timer is not stopped */
  Timer0Users |= (1<<7);
#endif
}

/* the timer is not stopped unless the rtos is off and all of the other timers
 * are inactive 
 */
void EnableRtosTick(void)
{
  RtosTickEnabled = 1;
  AddUser(TIMER0_RTOS_USER,RTOS_TICK_COUNT);
}

void DisableRtosTick(void)
{
  RtosTickEnabled = 0;
  RemoveUser(TIMER0_RTOS_USER);
}

static void AddUser(unsigned char User,unsigned int CrystalTicks)
{
  // Only use portENTER_CRITICAL() when not called from ISR
  // Otherwise, interrupts get enabled and the time counting can get stuck
  if (!TimerInterruptActive) {
    portENTER_CRITICAL();
    LAST_CRITICAL_CODE(CC_ADD_USER);
  }


  /* minimum value of 1 tick */
  if ( CrystalTicks < 1 ) CrystalTicks = 1;
  
  unsigned int CaptureTime = GetTickCount() + CrystalTicks;

  /* clear ifg, add to ccr register, enable interrupt */
  switch (User)
  {
  case 0: TA0CCTL0 = 0; TA0CCR0 = CaptureTime; TA0CCTL0 = CCIE; break;
  case 1: TA0CCTL1 = 0; TA0CCR1 = CaptureTime; TA0CCTL1 = CCIE; break;
  case 2: TA0CCTL2 = 0; TA0CCR2 = CaptureTime; TA0CCTL2 = CCIE; break;
  case 3: TA0CCTL3 = 0; TA0CCR3 = CaptureTime; TA0CCTL3 = CCIE; break;
  case 4: TA0CCTL4 = 0; TA0CCR4 = CaptureTime; TA0CCTL4 = CCIE; break;
  default: break;
  }
  
  /* start counting up in continuous mode if not already doing so */
  if (Timer0Users == 0) TA0CTL |= TASSEL_1 | MC_2 | ID_2;
  
  /* keep track of users */
  Timer0Users |= (1 << User);
  
  // Only use portEXIT_CRITICAL() when not called from ISR
  // Otherwise, interrupts get enabled and the time counting can get stuck
  if (!TimerInterruptActive)
    portEXIT_CRITICAL();
}

static void RemoveUser(unsigned char User)
{
  // Only use portENTER_CRITICAL() when not called from ISR
  // Otherwise, interrupts get enabled
  if (!TimerInterruptActive) {
    portENTER_CRITICAL();
    LAST_CRITICAL_CODE(CC_REMOVE_USER);
  }

  switch (User)
  {
  case 0: TA0CCTL0 = 0; break;
  case 1: TA0CCTL1 = 0; break;
  case 2: TA0CCTL2 = 0; break;
  case 3: TA0CCTL3 = 0; break;
  case 4: TA0CCTL4 = 0; break;
  default: break;
  }
  
  /* remove a user */
  Timer0Users &= ~(1 << User);
    
  /* disable timer if no one is using it */
  if (Timer0Users == 0) TA0CTL = 0;
 
  // Only use portENTER_CRITICAL() when not called from ISR
  // Otherwise, interrupts get enabled
  if (!TimerInterruptActive)
    portEXIT_CRITICAL();
}


/* 0 means off */
unsigned char QuerySchedulerState(void)
{
  return RtosTickEnabled;  
}

void StartCrystalTimer(unsigned char TimerId,
                       unsigned char (*pCallback) (void),
                       unsigned int Ticks)
{   
  if (!pCallback) return;
  
  /* assign callback */
  switch (TimerId)
  {
  case 1: pCrystalCallback1 = pCallback; break;
  case 2: pCrystalCallback2 = pCallback; break;
  case 3: pCrystalCallback3 = pCallback; break;
  case 4: pCrystalCallback4 = pCallback; break;
  default: break;
  }
  
  AddUser(TimerId,Ticks);
}

void RearmCrystalTimer(unsigned char TimerId,
                       unsigned int Ticks)
{
  AddUser(TimerId,Ticks);
}

void StopCrystalTimer(unsigned char TimerId)
{
  RemoveUser(TimerId);  
}

/* 
 * timer0 ccr0 has its own interrupt (TIMER0_A0) 
 */
#ifndef __IAR_SYSTEMS_ICC__
#pragma CODE_SECTION(TIMER0_A1_VECTOR_ISR,".text:_isr");
#endif
 
 
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_VECTOR_ISR(void)
{
  LAST_CRITICAL_CODE(CC_RTOS_TIMER_ISR);
  CODE_START(rtosTimerISR);

  unsigned char ExitLpm = 0;
  
  /* callback when timer expires */
  TimerInterruptActive=1;
  switch (__even_in_range(TA0IV,8))
  {
  /* remove the user first in case the callback is re-enabling this user */
  case 0: break;                  
  case 2: RemoveUser(1); ExitLpm = pCrystalCallback1(); break;
  case 4: RemoveUser(2); ExitLpm = pCrystalCallback2(); break;
  case 6: RemoveUser(3); ExitLpm = pCrystalCallback3(); break;
  case 8: RemoveUser(4); ExitLpm = pCrystalCallback4(); break;
  default: break;
  }
  TimerInterruptActive=0;
  
  if (ExitLpm) EXIT_LPM_ISR();

  CODE_END(rtosTimerISR);
}
