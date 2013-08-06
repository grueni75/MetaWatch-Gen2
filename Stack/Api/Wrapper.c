/*
 * Copyright (C) 2009-2013 by Matthias Ringwald, Matthias Gruenewald
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY MATTHIAS RINGWALD, MATTHIAS GRUENEWALD
 * AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * MATTHIAS RINGWALD, MATTHIAS GRUENEWALD OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at btstack@ringwald.ch
 * and gruenewald75@yahoo.de
 *
 */

/******************************************************************************/
/*! \file Wrapper.c
*
*/
/******************************************************************************/

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"

#include "Messages.h"
#include "MessageQueues.h"
#include "BufferPool.h"

#include "hal_lpm.h"
#include "hal_board_type.h"
#include "hal_rtc.h"
#include "hal_miscellaneous.h"
#include "hal_analog_display.h"
#include "hal_battery.h"
#include "hal_miscellaneous.h"
#include "hal_calibration.h"
#include "hal_boot.h"

#include "DebugUart.h"
#include "Utilities.h"
#include "IdleTask.h"
#include "Wrapper.h"

#include <string.h>
#include <btstack/hci_cmds.h>
#include <btstack/run_loop.h>
#include <btstack/sdp_util.h>
#include <btstack/version.h>
#include <run_loop_private.h>

#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"
#include "bt_control_cc256x.h"
#include "spp.h"

#include <stdarg.h>
#include <stdio.h>

#define SPP_MSG_QUEUE_LEN   8
#define SPP_STACK_SIZE    (configMINIMAL_STACK_SIZE + 300)
#define SPP_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)

static tMessage SPPMsg;

static eBluetoothState BluetoothStateInt = Unknown;
static uint8_t Discoverable=0;
static uint8_t OnceConnectedInt=0;
static uint8_t StackInitialized=0;

#if __IAR_SYSTEMS_ICC__
__no_init __root unsigned char niWrapperCurrentMsgType @WRAPPER_CURRENT_MSG_TYPE_ADDR;
#else
extern unsigned char niWrapperCurrentMsgType;
#endif

uint8_t ReadyToSleep=1;
char BDAddr[15] = { 0 };

//static uint32_t last_reset=0;

xTaskHandle xSPPTaskHandle;

/*! Sends a bluetooth state message */
void BluetoothStateChanged(eBluetoothState BS) {
  tMessage OutgoingMsg;
  switch (BS) {
    case Off:
      OnceConnectedInt=0;
    case Connect:
      OnceConnectedInt=1;
  }
  BluetoothStateInt=BS;
  SetupMessage(&OutgoingMsg, BluetoothStateChangeMsg, BS);
  RouteMsg(&OutgoingMsg);
}

/*! Sets the visibility of the device */
void SetDiscoverability(unsigned char Value) {
  hci_discoverable_control(Value);
  Discoverable=Value;
}

/*! Turns of the bluetooth chip */
void BluetoothPower(unsigned char power) {
  if (!StackInitialized) {
    btstack_init();
    ClearResetCode();
    StackInitialized=1;
  }
  switch(BluetoothStateInt) {
    case Unknown:
    case Off:
      if (power) {
        hci_power_control(HCI_POWER_ON);
        SetDiscoverability(1);
        BluetoothStateChanged(Initializing);
      }
      break;
    case On:
    case Connect:
      if (!power) {
        SetDiscoverability(0);
        hci_power_control(HCI_POWER_OFF);
        BluetoothStateChanged(Initializing);
      }
  }
}

/*! Handle the messages routed to the SPP task */
static unsigned char SPPMessageHandler(tMessage* pMsg)
{
  //tMessage OutgoingMsg;

  switch(pMsg->Type)
  {
    case SetHeartbeatMsg:
      run_loop_execute();
      /*uint32_t t=embedded_get_ticks()-last_reset;
      if (t%2==0) {
        DISABLE_LCD_LED();
      } else {
        ENABLE_LCD_LED();
      }
      if (t>4*30) {
        btstack_error_handler();
        last_reset=embedded_get_ticks();
      }*/
      break;

    case WrapperTaskCheckInMsg:
      TaskCheckIn(eWrapperTaskCheckInId);
      //PrintS("BTS: task checkin done");
      break;

    case TurnRadioOnMsg:
      //PrintS("BTS: turn radio on start");
      BluetoothPower(1);
      //PrintS("BTS: turn radio on end");
      break;

    case TurnRadioOffMsg:
      //PrintS("BTS: turn radio off start");
      BluetoothPower(0);
      //PrintS("BTS: turn radio off end");
      break;

    case SniffControlMsg:
      switch(pMsg->Options) {
        case MSG_OPT_EXIT_SNIFF:
          btstack_set_sniff_mode(0); // first transmission exits sniff anyway, so no need to do it here
          break;
        default:
          btstack_set_sniff_mode(1);
          break;
      }
      break;

    case ShippingModeMsg:
    case ReadRssiMsg:
    case ReadRssiResponseMsg:
    case PairingControlMsg:
    case CallerIdIndMsg:
    case MapMsg:
    case MapIndMsg:
    case ConnChangeMsg:
    case UpdWgtIndMsg:
    case ConnParamChgIndMsg:
    case SppAckMsg:
    case QueryMemoryMsg:
    case RadioPowerControlMsg:
    case EnableAdvMsg:
    case SetAdvDataMsg:
    case SetScanRespMsg:
      PrintS("BTS: not implemented");
      break; // not implemented (and also not used anymore)

    case ConnTimeoutMsg:
    case TunnelTimeoutMsg:
      PrintS("BTS: not used");
      break; // not used (stack has own timer mechanism)

    case UpdConnParamMsg:
      PrintS("BTS: no BLE support");
      break; // not implemented (BLE not supported yet)

    case HfpMsg:
      PrintS("BTS: no HFP support");
      break; // not implemented (hands free protocol not yet supported)

    default:
      //PrintS("BTS: queue packet start");
      btstack_queue_tx_packet(pMsg);
      //PrintS("BTS: queue packet end");
      return 0;

  }
  return 1;
}

/*! Function to implement the SPPTask loop
 *
 * \param pvParameters not used
 *
 */
static void SPPTask(void *pvParameters)
{
  // Message loop
  for(;;)
  {
    // Check for messages
    if (xQueueReceive(QueueHandles[WRAPPER_QINDEX], &SPPMsg, portMAX_DELAY))
    {
      niWrapperCurrentMsgType=SPPMsg.Type;
      PrintMessageType(&SPPMsg);
      if (SPPMessageHandler(&SPPMsg))
        SendToFreeQueue(&SPPMsg);
      CheckStackUsage(xSPPTaskHandle,"SPP Task");
      CheckQueueUsage(QueueHandles[WRAPPER_QINDEX]);
      niWrapperCurrentMsgType=0;
    }
  }
}

/*! Initialize the serial port profile task.  This should be called from main.
*
* This task opens the stack which in turn creates 3 more tasks that create and
* handle the bluetooth serial port connection.
*/
void CreateWrapperTask(void)
{
  // Init queue for communication with the firmware
  QueueHandles[WRAPPER_QINDEX] =
    xQueueCreate( SPP_MSG_QUEUE_LEN, MESSAGE_QUEUE_ITEM_SIZE );

  if (QueueHandles[WRAPPER_QINDEX] == 0) {
    PrintS("SPP queue creation failed");
    SoftwareReset();
  }

  // prams are: task function, task name, stack len , task params, priority, task handle
  xTaskCreate(SPPTask,
              (const signed char *)"SPP",
              SPP_STACK_SIZE,
              NULL,
              SPP_TASK_PRIORITY,
              &xSPPTaskHandle);
}

/*! Query the serial port profile task if it is okay to put the part into LPM3
*
* This function is called by the idle task.  The idle task puts the
* MSP430 into sleep mode.
*
* \return 0 if micro cannot go into LPM3; 1 if micro can go into LPM3
*/
unsigned char SerialPortReadyToSleep(void)
{
  return ReadyToSleep;
}

unsigned char PairedDeviceType(void)
{
  return DEVICE_TYPE_SPP;
}

/*! Determine if the bluetooth link is in the connected state.
* When the phone and watch are connected then data can be sent.
*
* \return 0 when not connected, 1 when connected
*/
unsigned char Connected(unsigned char Type)
{
  switch(Type) {
    case CONN_TYPE_MAIN:
      switch(BluetoothStateInt) {
        case Connect:
          return 1;
        default:
          return 0;
      }
    default:
      return 0; // BLE not yet supported
  }
}

/*! Check if been connected once
* \return 0 never been connected; 1: connected
*/
unsigned char OnceConnected()
{
  return (unsigned char)OnceConnectedInt;
}

/*! This function is used to determine if the radio is on and will return 1 when
* initialization has successfully completed, but the radio may or may not be
* paired or connected.
*
* \return 1 when stack is in the connected, paired, or radio on state, 0 otherwise
*/
unsigned char RadioOn(void)
{
  switch(BluetoothStateInt) {
    case Connect:
    case On:
      return 1;
    default:
      return 0;
  }
}

/*! This function is used to determine if the connection state of the
 * bluetooth serial port.
 *
 * \return eBluetoothState
 */
eBluetoothState BluetoothState(void)
{
  return BluetoothStateInt;
}

/*! Query Bluetooth Discoverability
 *
 * \return 0 when not discoverable, 1 when discoverable
 */
unsigned char QueryDiscoverable(void)
{
  return Discoverable;
}

/*! Query Bluetooth pairing information
 *
 * \return 0 when valid pairing does not exist, 1 when valid pairing information
 * exists
 */
unsigned char ValidAuthInfo(void)
{
  return 0;  // not yet adapted
}

void GetBDAddrStr(char *pAddr)
{
  strcpy(pAddr,BDAddr);
}

/*! If a function wishes to disable the flow of characters from the radio
 * during a long interrupt routine, it must first read the state of flow
 * pin. */
void EnableFlowControl(unsigned char Enable)
{
  // not yet supported (but also currently not used)
}
