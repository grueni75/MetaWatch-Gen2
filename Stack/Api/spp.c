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
/*! \file spp.c
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
#include "MessageInfo.h"
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
#include "Wrapper.h"

#include <string.h>
#include <btstack/hci_cmds.h>
#include <btstack/run_loop.h>
#include <btstack/sdp_util.h>
#include <btstack/memory_pool.h>

#include <run_loop_private.h>

#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"
#include "bt_control_cc256x.h"

#include <stdarg.h>
#include <stdio.h>

enum CONNECTION_MODE { ACTIVE=0, HOLD=1, SNIFF=2, PARK=3 } ;
enum STATE {INIT, W4_CONNECTION, W4_CHANNEL_COMPLETE, W4_SET_NOT_DISCOVERABLE, W4_ACTIVE} ;

extern const remote_device_db_t remote_device_db_metawatch;

//run_loop_t                  *the_run_loop;

static uint8_t              rfcomm_channel_nr = 1;
static uint16_t             rfcomm_mtu;
static uint16_t             rfcomm_connection_handle;
static uint16_t             rfcomm_channel_id;
static enum CONNECTION_MODE rfcomm_connection_mode = ACTIVE;
static uint8_t              spp_service_buffer[100];
static const uint16_t       sniff_max_interval = 1600;  // time unit: 0.625us
static const uint16_t       sniff_min_interval = 1344;
static const uint16_t       sniff_return_timeout = 3000; // time unit: ms
static const uint16_t       sniff_attempt = 10;
static const uint16_t       sniff_timeout = 10;
static timer_source_t       sniff_timer;
static enum STATE           state = INIT;

// Variables and functions used from Wrapper.c
extern char BDAddr[];
void SetDiscoverability(unsigned char Value);
void BluetoothStateChanged(eBluetoothState BS);

// Functions used from hal_uart_dma.c
void hal_uart_dma_deinit(void);

// Flag to enable packet dumps
int btstack_enable_dump_mode=0;

// Flag to indicate that bluetooth shall be switched off
int tx_power_off_after_delivery = 0;

// Flag that indicates a restart of the stack
int btstack_restart = 0;

// Next data to sent
#define MAX_TX_MSG_QUEUE_SIZE  ( 8 )
static tMessage         tx_msg_queue[MAX_TX_MSG_QUEUE_SIZE];
static uint8_t          tx_msg_queue_start = 0;
static uint8_t          tx_msg_queue_end = 0;
static uint8_t          tx_msg_queue_full = 0;
static uint8_t          tx_data[MSG_BUFFER_LENGTH];
static uint8_t          tx_data_len;
static uint8_t          tx_data_pos;
static uint8_t          tx_msg_handed_over_to_bluetooth_chip = 0;
static timer_source_t   tx_retry_timer;
static uint8_t          tx_retry_timer_active=0;
static const uint16_t   tx_retry_timeout = 1000;   // time unit: ms

// Packet that is currently received
static tMessage rx_msg;
static uint8_t rx_msg_type;
static uint8_t rx_msg_options;
static int8_t rx_msg_pos;
static uint8_t rx_msg_size;

// Checks if the tx queue is empty
static uint8_t tx_queue_empty() {
  return (tx_msg_queue_start==tx_msg_queue_end)&&(!tx_msg_queue_full) ? 1 : 0;
}

// Activates a new timer
static void run_loop_register_timer(timer_source_t *timer, uint16_t period) {
  run_loop_set_timer(timer, period);
  run_loop_remove_timer(timer);
  run_loop_add_timer(timer);
}

// Restarts the stack in case an error condition occurs
void btstack_error_handler() {
  SoftwareReset();
}

// Sets active mode
static void transit_to_active_mode()
{
  if (state!=W4_ACTIVE) return;

  // If sniff mode is on, request active mode
  if (rfcomm_connection_mode == SNIFF) {
    if (hci_can_send_packet_now(HCI_COMMAND_DATA_PACKET)) {
      //PrintS("BTS: exiting sniff mode");
      hci_send_cmd(&hci_exit_sniff_mode,rfcomm_connection_handle);
    }
  }
  // In active mode, ensure that return to sniff timeout is updated
  if (rfcomm_connection_mode == ACTIVE) {
    run_loop_register_timer(&sniff_timer, sniff_return_timeout);
  }
}

// Sets sniff mode
static void transit_to_sniff_mode(struct timer *t)
{
  if (state!=W4_ACTIVE) return;
  if (rfcomm_connection_mode == ACTIVE) {
    if (hci_can_send_packet_now(HCI_COMMAND_DATA_PACKET)) {
      //PrintS("BTS: entering sniff mode");
      hci_send_cmd(&hci_sniff_mode,rfcomm_connection_handle,sniff_max_interval,sniff_min_interval,sniff_attempt,sniff_timeout);
    }
    run_loop_register_timer(&sniff_timer,sniff_return_timeout);
  }
}

// Calculates the crc value
static void add_crc(uint8_t *data, uint8_t len)
{
  int8_t i,j;
  uint16_t crc = 0xFFFF;
  for (j = 0; j < len; j++) {
    uint8_t c = data[j];
    for (i = 7; i >= 0; i--) {
      uint8_t c15 = ((crc >> 15 & 1) == 1);
      uint8_t bit = ((c >> (7 - i) & 1) == 1);
      crc <<= 1;
      if (c15 ^ bit)
        crc ^= 0x1021; // 0001 0000 0010 0001 (0, 5, 12)
    }
  }
  uint32_t crc2 = crc - 0xffff0000;
  data[len] = (uint8_t) (crc2 % 256);
  data[len+1] = (uint8_t) (crc2 / 256);
}

// Send routine
static void try_to_send(void) {

  tMessage *msg;
  tMessage OutgoingMsg;
  uint8_t tx_data_fragment_len;

  if (state!=W4_ACTIVE) return;
  if ((tx_queue_empty())&&(tx_data_len==0)) {
    if (tx_power_off_after_delivery) {
      SetupMessage(&OutgoingMsg, TurnRadioOffMsg, MSG_OPT_NONE);
      RouteMsg(&OutgoingMsg);
      tx_power_off_after_delivery=0;
    }
    return;
  }
  //PrintS("BTS: trying to sent packet");

  // Set active mode
  transit_to_active_mode();

  // If we do not have tx data, get the next message from the queue
  if (tx_data_len==0) {
    if (tx_queue_empty())
      return; // queue empty
    msg=&tx_msg_queue[tx_msg_queue_start];
    /*PrintS("BTS: Message going to be sent");
    PrintF("0x01");
    PrintF("0x%02x",msg->Length+6);
    PrintF("0x%02x",msg->Type);
    PrintF("0x%02x",msg->Options);
    for (i=0;i<msg->Length;i++)
      PrintF("0x%02x",msg->pBuffer[i]);*/
    tx_data[0]=1;               // start byte
    tx_data[1]=6+msg->Length;   // length
    tx_data[2]=msg->Type;
    tx_data[3]=msg->Options;
    if (msg->Length) memcpy(&tx_data[4],msg->pBuffer,msg->Length);
    add_crc(tx_data,tx_data[1]-2);
    SendToFreeQueue(msg);
    tx_msg_queue_start++;
    if (tx_msg_queue_start>=MAX_TX_MSG_QUEUE_SIZE)
      tx_msg_queue_start=0;
    tx_msg_queue_full=0;
    tx_data_pos=0;
    tx_data_len=tx_data[1];
    //PrintF("BTS: tx_data_len=%d",tx_data_len);
  }

  // Prepare the next fragment for transmission
  tx_data_fragment_len=tx_data_len-tx_data_pos;
  if (tx_data_fragment_len>rfcomm_mtu) {
    tx_data_fragment_len=rfcomm_mtu;
  }

  // Try to transmit fragment
  int err = rfcomm_send_internal(rfcomm_channel_id, &tx_data[tx_data_pos], tx_data_fragment_len);
  switch (err){
    case 0:
      //PrintS("BTS: packet sent");
      tx_data_pos+=tx_data_fragment_len;
      if (tx_data_pos>=tx_data_len) {
        /*PrintS("BTS: Message handed over");
        for (i=0;i<tx_data_len;i++)
          PrintF("0x%02x",tx_data[i]);*/
        tx_data_len=0;
        tx_msg_handed_over_to_bluetooth_chip=1;
      }
      break;
    case BTSTACK_ACL_BUFFERS_FULL:
      break;
    default:
      break;
  }
}

// Tries to sent if the tx queue is still full
static void retry_transmission(struct timer *t)
{
  if (!tx_queue_empty()) {
    //PrintS("BTS: tx queue still contains packets, trying to sent them");
    try_to_send();
    run_loop_register_timer(&tx_retry_timer,tx_retry_timeout);
  } else {
    //PrintS("BTS: tx queue empty, stopping timer");
    tx_retry_timer_active=0;
  }
}

// Bluetooth packet handler
static void packet_handler(void * connection, uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
  bd_addr_t event_addr;
  uint8_t event = packet[0];
  uint16_t connection_handle;
  uint8_t current_mode;
  uint8_t status;
  uint16_t pos;
  uint16_t interval;
  uint16_t t;
  tMessage OutgoingMsg;

  // Handle data packets
  if (packet_type == RFCOMM_DATA_PACKET) {

    // Set active mode
    transit_to_active_mode();

    // Process data
    pos=0;
    while (pos<size) {

      switch(rx_msg_pos) {

        // Check start byte
        case 0:
          if (packet[pos]!=1) {
            PrintS("BTSERR: Corrupted packet received (start byte wrong)");
            rx_msg_pos=-1;
            pos=size; // skip the rest of the received data
            btstack_error_handler();
          }
          break;

        // Extract length
        case 1:
          rx_msg_size=packet[pos];
          if (rx_msg_size>MSG_BUFFER_LENGTH) {
            PrintS("BTSERR: Corrupted packet received (messsage too long)");
            rx_msg_pos=-1;
            pos=size; // skip the rest of the received data
            btstack_error_handler();
          }
          break;

        // Extract type
        case 2:
          rx_msg_type=packet[pos];
          break;

        // Extract options and allocate message
        case 3:
          rx_msg_options=packet[pos];
          if (rx_msg_size>6) {
            SetupMessageWithBuffer(&rx_msg,rx_msg_type,rx_msg_options);
            rx_msg.Length=rx_msg_size-6;
          } else {
            SetupMessage(&rx_msg,rx_msg_type,rx_msg_options);
          }
          break;

        // Copy payload
        default:

          // CRC reached?
          if (rx_msg_pos==rx_msg_size-2) {

            // Send message and skip crc
            /*PrintS("BTS: Message received");
            PrintF("0x01");
            PrintF("0x%02x",rx_msg.Length+6);
            PrintF("0x%02x",rx_msg.Type);
            PrintF("0x%02x",rx_msg.Options);
            for (status=0;status<rx_msg.Length;status++)
              PrintF("0x%02x",rx_msg.pBuffer[status]);*/
            RouteMsg(&rx_msg);
            rx_msg_pos=-1;
            pos++;

          } else {

            // Copy payload
            rx_msg.pBuffer[rx_msg_pos-4]=packet[pos];

          }
      }
      pos++;
      rx_msg_pos++;
    }
    rfcomm_grant_credits(rfcomm_channel_id, 1); // get the next packet
    return;
  }

  // handle events, ignore data
  if (packet_type != HCI_EVENT_PACKET) return;

  // Go to init state if stack is switched off
  if ((event == BTSTACK_EVENT_STATE) && (packet[2] == HCI_STATE_HALTING)) {
    state = INIT;
    return;
  }

  // Handle state
  switch(state) {
    case INIT:
      switch(event){
        case BTSTACK_EVENT_STATE:
          // bt stack activated, get started - set local name
          if (packet[2] == HCI_STATE_WORKING) {
            hci_send_cmd(&hci_write_local_name, "MetaWatch (BTStack)");
          }
          if (packet[2] == HCI_STATE_OFF) {
            hal_uart_dma_deinit();
            BluetoothStateChanged(Off);
            PrintS("BTS: stack is powered off");
            if (btstack_restart) {
              btstack_restart=0;
              SetupMessage(&OutgoingMsg, TurnRadioOnMsg, MSG_OPT_NONE);
              RouteMsg(&OutgoingMsg);
            }
          }
          break;
        case HCI_EVENT_COMMAND_COMPLETE:
          if (COMMAND_COMPLETE_EVENT(packet, hci_read_bd_addr)) {
            BDAddr[0]=0;
            for (pos=OFFSET_OF_DATA_IN_COMMAND_COMPLETE+5;pos>=OFFSET_OF_DATA_IN_COMMAND_COMPLETE+1;pos-=2) {
              t=READ_BT_16(packet,pos);
              if (BDAddr[0]==0)
                snprintf(BDAddr,15,"  %04x",t);
              else
                snprintf(BDAddr,15,"%s%04x",BDAddr,t);
            }
          }
          if (COMMAND_COMPLETE_EVENT(packet, hci_write_local_name)) {
            BluetoothStateChanged(On);
            PrintS("BTS: stack is powered on");
            state=W4_CONNECTION;
          }
          break;
        default:
          break;
      }
      break;

    case W4_CONNECTION:
      switch(event){
        case HCI_EVENT_PIN_CODE_REQUEST:
          // inform about pin code request
          //printf("Pin code request - using '0000'\n\r");
          bt_flip_addr(event_addr, &packet[2]);
          hci_send_cmd(&hci_pin_code_request_reply, &event_addr, 4, "0000");
          break;
        case RFCOMM_EVENT_INCOMING_CONNECTION:
          // data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
          bt_flip_addr(event_addr, &packet[2]);
          rfcomm_channel_nr = packet[8];
          rfcomm_channel_id = READ_BT_16(packet, 9);
          //printf("RFCOMM channel %u requested for %s\n\r", rfcomm_channel_nr, bd_addr_to_str(event_addr));
          rfcomm_accept_connection_internal(rfcomm_channel_id);
          state = W4_CHANNEL_COMPLETE;
          break;
        default:
          break;
      }
      break;

    case W4_CHANNEL_COMPLETE:

      if ( event != RFCOMM_EVENT_OPEN_CHANNEL_COMPLETE ) break;

      // data: event(8), len(8), status (8), address (48), handle (16), server channel(8), rfcomm_cid(16), max frame size(16)
      if (packet[2]) {
        //printf("RFCOMM channel open failed, status %u\n\r", packet[2]);
        break;
      }
      rfcomm_connection_handle = READ_BT_16(packet, 9);
      rfcomm_channel_id = READ_BT_16(packet, 12);
      rfcomm_mtu = READ_BT_16(packet, 14);

      // Update state
      //printf("\n\rRFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n\r", rfcomm_channel_id, rfcomm_mtu);
      PrintS("BTS: RFCOMM channel opened");
      state = W4_ACTIVE;
      rx_msg_pos = 0;
      tx_data_len = 0;
      tx_power_off_after_delivery = 0;
      //btstack_enable_dump_mode = 1;

      // Setup sniff mode
      rfcomm_connection_mode=ACTIVE;
      transit_to_sniff_mode(NULL);

      // Inform watch
      BluetoothStateChanged(Connect);
      break;

    case W4_ACTIVE:
      switch(event){
        case HCI_EVENT_MODE_CHANGE_EVENT:
          // data: event (8), len (8), status (8), handle (16), current_mode (8), interval (16)
          connection_handle = READ_BT_16(packet, 3);
          if (connection_handle==rfcomm_connection_handle) {
            status = packet[2];
            if (status!=0) {
              //PrintS("BTSERR: Mode change command failed!");
            }
            current_mode = packet[5];
            interval = READ_BT_16(packet,6);
            //printf("current_mode=%d interval=%d\n",current_mode,interval);
            switch (current_mode) {
              case 0:
                // Ensure that sniff mode is re-activated after timeout
                rfcomm_connection_mode = ACTIVE;
                run_loop_register_timer(&sniff_timer,sniff_return_timeout);
                break;
              case 2:
                rfcomm_connection_mode = SNIFF;
                break;
              default:
                PrintS("BTSERR: Unsupported connection mode!");
            }
          }
          break;
        case DAEMON_EVENT_HCI_PACKET_SENT:
        case RFCOMM_EVENT_CREDITS:
          if (QueryDiscoverable()) {
            SetDiscoverability(0);
          }
          else
            try_to_send();
          if (tx_msg_handed_over_to_bluetooth_chip) {
            //PrintS("BTS: Message sent");
            tx_msg_handed_over_to_bluetooth_chip=0;
          }
          break;
        case RFCOMM_EVENT_CHANNEL_CLOSED:
          state = W4_CONNECTION;
          BluetoothStateChanged(On);
          SetupMessage(&OutgoingMsg, LinkAlarmMsg, MSG_OPT_NONE);
          RouteMsg(&OutgoingMsg);
          PrintS("BTS: RFCOMM channel closed");
          break;
        case HCI_EVENT_COMMAND_STATUS:
        case HCI_EVENT_COMMAND_COMPLETE:
        case HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS:
        case HCI_EVENT_READ_REMOTE_VERSION_INFORMATION_COMPLETE:
        case 0x66:
        case 0x73:
          break;
        default:
          PrintF("BTS: unknown event 0x%02x received!",event);
          break;
      }
      break;

    default:
      break;
  }
}

// Initializes the bluetooth stack
void btstack_init() {

  /// GET STARTED with BTstack ///
  btstack_memory_init();
  run_loop_init(RUN_LOOP_EMBEDDED);

  // init HCI
  hci_transport_t    * transport = hci_transport_h4_dma_instance();
  bt_control_t       * control   = bt_control_cc256x_instance();
  hci_uart_config_t  * config    = hci_uart_config_cc256x_instance();
  remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_metawatch;
  hci_init(transport, config, control, remote_db);

  // use eHCILL
  bt_control_cc256x_enable_ehcill(1);

  // init L2CAP
  l2cap_init();
  l2cap_register_packet_handler(packet_handler);

  // init RFCOMM
  rfcomm_init();
  rfcomm_register_packet_handler(packet_handler);
  rfcomm_register_service_with_initial_credits_internal(NULL, rfcomm_channel_nr, 100, 1);  // reserved channel, mtu=100, 1 credit

  // init SDP, create record for SPP and register with SDP
  sdp_init();
  memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
  service_record_item_t * service_record_item = (service_record_item_t *) spp_service_buffer;
  sdp_create_spp_service( (uint8_t*) &service_record_item->service_record, 1, "MW-SPP");
  sdp_register_service_internal(NULL, service_record_item);

  // Init timers
  sniff_timer.process = &transit_to_sniff_mode;
  tx_retry_timer.process = &retry_transmission;
}


// Adds a message to the transmission queue
void btstack_queue_tx_packet(tMessage *tx_msg)
{
  if (state!=W4_ACTIVE) {
    SendToFreeQueue(tx_msg);
    return;
  }
  if (tx_msg_queue_full) {
    PrintS("BTSERR: TX message queue full");
    SendToFreeQueue(tx_msg);
    return;
  }
  uint8_t new_tx_msg_queue_end;
  new_tx_msg_queue_end=tx_msg_queue_end+1;
  if (new_tx_msg_queue_end>=MAX_TX_MSG_QUEUE_SIZE)
    new_tx_msg_queue_end=0;
  if (new_tx_msg_queue_end==tx_msg_queue_start) {
    tx_msg_queue_full=1;
  }
  memcpy(&tx_msg_queue[tx_msg_queue_end],tx_msg,sizeof(tMessage));
  tx_msg_queue_end=new_tx_msg_queue_end;
  switch (tx_msg->Type) {
    case LowBatteryBtOffMsgHost:
      tx_power_off_after_delivery=1;
      break;
  }
  //PrintS("BTS: Message queued");
  try_to_send();
  if (!tx_retry_timer_active) {
    tx_retry_timer_active=1;
    run_loop_register_timer(&tx_retry_timer, tx_retry_timeout);
  }
}

