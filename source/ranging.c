#include <stdio.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#include <wiringPi.h> //TODO: Remove this when porting

#include "mac_comms.h"
#include "spi_comms.h"

#define STDOUT_FD 1

volatile sig_atomic_t interrupt_test = 0;

void test_interrupt(void) {
  interrupt_test = 1;
}

int main() {
  if (wiringPiSetupGpio() < 0) {
    perror("Error Initializeing wiringPi");
    exit(EXIT_FAILURE);
  }
  struct spi_bus bus0;
  char sel[5];
  printf("Select device (0/1):\n");
  char i_name[20];
  char* ret = fgets(sel, sizeof(sel), stdin);
  if (ret == NULL) {
    printf("End of file reached before input\n");
    exit(EXIT_FAILURE);
  } else if (sel[0] == '0') {
    memcpy(i_name,"/dev/spidev0.0",sizeof(i_name));
  } else if (sel[0] == '1') {
    memcpy(i_name,"/dev/spidev1.0",sizeof(i_name));
  } else {
    exit(EXIT_FAILURE);
  }

  struct spi_ioc_transfer xfer0;
//  struct system_conf sys_conf;
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
  sys_ctrl_init(&sys_ctrl);
  //Set Pll
  set_pll(&bus0);


  //Check System Status
  struct system_status sta;
  char tx_status[SYS_STATUS_LEN] = {0x00};
  char rx_status[SYS_STATUS_LEN] = {0x00};
  tx_status[0] = SYS_STATUS_REG;
  write_spi_msg(&bus0, &sta, tx_status, SYS_STATUS_LEN);
  memcpy(rx_status, &sta, sizeof(struct system_status));
  printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", rx_status[1],
    rx_status[2], rx_status[3], rx_status[4], rx_status[5]);
  clear_status(&bus0, &sta);
  write_spi_msg(&bus0, &sta, tx_status, SYS_STATUS_LEN);
  memcpy(rx_status, &sta, sizeof(struct system_status));
  printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", rx_status[1],
    rx_status[2], rx_status[3], rx_status[4], rx_status[5]);

  //Setup tx_fctrl  
  struct tx_fctrl fctrl;
  frame_control_init(&fctrl);
  //Setup tx_buffer 
  struct tx_buffer tx_buff;
  tx_buffer_init(&tx_buff);

  if (decawave_comms_init(&bus0, 0xFFFF, 0xFFFF, &fctrl) == 1) {
    exit(EXIT_FAILURE);
  }
  //Determine Interrupt Pin
  /*
  printf("Select Interrupt Pin (BCM)\n");
  ret = fgets(sel, sizeof(sel), stdin);
  printf("Ok, using interrupt pin %d\n", atoi(sel));
  int interrupt_pin = atoi(sel);
  */
  int interrupt_pin = 0;

  //Determine Mode
  printf("Mode: 1 for initial sender | 0 for receiver\n");
  ret = fgets((char *) &tx_buff.payload, sizeof(tx_buff.payload), stdin);
  if (ret == NULL) {
    printf("Error reading tx_buff.payload or no value entered\n");
    exit(EXIT_FAILURE);
  }
  //Interrupt Check
  /*if (wiringPiISR(3, INT_EDGE_RISING, test_interrupt) < 0){
    perror("Error setting up ISR");
  }
  while (interrupt_test == 0) {}
  printf("Got Interrupt\n");
  return 0; */
  
  //Load microcode
  load_microcode(&bus0);
  //Interrupt Mask
  struct system_mask mask;
  sys_mask_init(&bus0, &mask);
  
  //Check Mode
  if (tx_buff.payload[0] == '0') {
    printf("Waiting for msg...\n");
    ranging_recv(&bus0, &sys_ctrl, &sta, &tx_buff, interrupt_pin);
  } else if (tx_buff.payload[0] == '1') { //Transmit Message
    ranging_send(&bus0, &sys_ctrl, &sta, &tx_buff, interrupt_pin);
  }
  return 0;
}
