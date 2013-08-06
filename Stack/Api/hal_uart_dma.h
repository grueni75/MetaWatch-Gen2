/**
 * @file  hal_uart_dma.h
 ***************************************************************************/
 
// RX error counter (lost bytes)
extern unsigned char rx_errors;

/**
 * @brief  Deinits the serial communications peripheral and GPIO ports
 *         to communicate with the PAN BT .. assuming 16 Mhz CPU
 *
 * @param  none
 *
 * @return none
 */
void hal_uart_dma_deinit(void);

/**
 * @brief  Stops the flow of characters from the bluetooth UART
 *
 * @param  disable: disable (1) or enable (0) the flow control
 *
 * @return none
 */
void hal_uart_disable_rx_flow(unsigned char disable);
