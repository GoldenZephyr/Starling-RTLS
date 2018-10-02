#include <stdio.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "mac_comms.h"
#include "spi_comms.h"

#define STDOUT_FD 1

int main() {
  struct spi_bus bus0;
  char sel[5];
  printf("Select device (0/1):\n");
  char i_name[20];
  char* ret = fgets(sel, sizeof(sel), stdin);
  if (ret == NULL) {
    printf("End of file reached before input");
  } else if (sel[0] == '0') {
    memcpy(i_name,"/dev/spidev0.0",sizeof(i_name));
  } else if (sel[0] == '1') {
    memcpy(i_name,"/dev/spidev1.0",sizeof(i_name));
  } else {
    return EXIT_FAILURE;
  }
  struct spi_ioc_transfer xfer0;

  struct tx_fctrl fctrl;
  struct tx_buffer tx_buff;
  struct system_conf sys_conf;
  struct system_control sys_ctrl;

  bus0.interface_name = i_name;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  //Check for proper SPI setup
  if (comms_check(&bus0) == 0) {
    printf("SPI Error: Cannot read device ID\n");
    exit(EXIT_FAILURE);
  }
  struct system_status sta;
  //Clear Status reg
  clear_status(&bus0, &sta);
  //Setup tx_fctrl
  frame_control_init(&fctrl);

  //Setup tx_buffer
  tx_buffer_init(&tx_buff);

  //Setup System Configure
  sys_conf_init(&sys_conf);
  unsigned char conf_rx[SYS_CONF_LEN] = {0x00};
  write_spi_msg(&bus0, conf_rx, &sys_conf, SYS_CONF_LEN);
  //Load Microcode
  unsigned char otp_ctrl_rx[4] = {0x00};
  unsigned char otp_ctrl_tx[4] = {0x00};
  otp_ctrl_tx[0] = 0x2D | 0xC0;
  otp_ctrl_tx[1] = 0x06;
  otp_ctrl_tx[3] = 0x80;
  write_spi_msg(&bus0, otp_ctrl_rx, otp_ctrl_tx, 0x04);

  //Get Payload - User Input

  //Initialize frame control / Device IDs
  const uint16_t pan_id = PAN_ID_LO | (PAN_ID_HI << 8);
  const uint16_t addr_id = ADDR_ID_LO | (ADDR_ID_HI << 8);
  if (decawave_comms_init(&bus0, pan_id, addr_id, &fctrl) == 1) {
    exit(EXIT_FAILURE);
  }
  //Confirm fctrl
  unsigned char tx_fc[TX_FCTRL_LEN] = {0x00};
  tx_fc[0] = TX_FCTRL_REG;
  unsigned char rx_fc[TX_FCTRL_LEN] = {0x00};
  write_spi_msg(&bus0, rx_fc, tx_fc, TX_FCTRL_LEN);
  printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
         rx_fc[1],rx_fc[2],rx_fc[3],rx_fc[4],rx_fc[5]);

  //Write in Payload
  printf("Type in payload (%lu chars max):\n", sizeof(tx_buff.payload));
  printf("No payload for receive mode\n");
  ret = fgets((char *) &tx_buff.payload, sizeof(tx_buff.payload), stdin);
  if (ret == NULL) {
    printf("Error reading tx_buff.payload");
    exit(EXIT_FAILURE);
  } else {
    printf("%s", tx_buff.payload);
  }

  sys_ctrl_init(&sys_ctrl);
  if (tx_buff.payload[0] == '\n') {
    printf("Waiting for msg...\n");
    wait_for_msg(&bus0, &sys_ctrl);
  } else { //Transmit Message
    unsigned char rx_payload_buf[TX_BUFFER_LEN] = {0x00};
    write_spi_msg(&bus0, rx_payload_buf, &tx_buff, TX_BUFFER_LEN);
    //Transmit Message
    send_message(&bus0, &sys_ctrl);
  }
}

