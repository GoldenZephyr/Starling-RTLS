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
  printf("Select Interrupt Pin (BCM)");
  ret = fgets(sel, sizeof(sel), stdin);
  int interrupt_pin = atoi(sel);


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
  struct system_status sta;
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
