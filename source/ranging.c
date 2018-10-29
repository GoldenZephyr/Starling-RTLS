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

  
  //Load microcode
  load_microcode(&bus0);
  //Interrupt Mask
  struct system_mask mask;
  sys_mask_init(&bus0, &mask);
 

  struct timespec slptime;
  slptime.tv_sec = 0;
  slptime.tv_nsec = 500000000; //0.5 sec
  
  while(1) {
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
   
  
   struct range_info info;
   memset(&info, 0x00, sizeof(struct range_info));
   //Check Mode
   if (tx_buff.payload[0] == '0') {
     slptime.tv_nsec = 100000000; //0.1 sec
     while (1) {
     printf("Waiting for msg...\n");
     ranging_recv(&bus0, &sys_ctrl, &sta, &tx_buff, &info, interrupt_pin);
     //Propogation Logic
     /*
     uint64_t t_round1 = time_sub(info.timestamp_rx_2, info.timestamp_tx_1);
     uint64_t t_round2 = time_sub(info.timestamp_rx_3, info.timestamp_tx_2);
     double t_prop = ((double)t_round1 * (double)t_round2 - (double)T_REPLY*(double)T_REPLY) / ((double)t_round1 + (double)t_round2 + (double)T_REPLY + (double)T_REPLY);
     printf("I compute a tprop of %f clock cycles\n", t_prop);
     double prop_in_secs = ((double) t_prop) / ((double) CLOCK_FREQ);
     printf("So the distance is %f meters\n", prop_in_secs * LIGHT_SPEED);
     */
     printf("%llu, %llu, %llu, %llu, %llu, %llu, %d, %d, %d, %d, %u, %u, %u %u %u %u %u %u %u %u %u %u %u %u\n",
     (uint64_t) info.timestamp_tx_1, (uint64_t) info.timestamp_rx_1,
     (uint64_t) info.timestamp_tx_2, (uint64_t) info.timestamp_rx_2,
     (uint64_t) info.timestamp_tx_3, (uint64_t) info.timestamp_rx_3,
     info.fp_ampl1_rx_1, info.fp_ampl1_rx_2, info.fp_ampl1_rx_3,
     info.rxpacc_rx_1, info.rxpacc_rx_2, info.rxpacc_rx_3, 
     info.rx_fqual_1.std_noise, info.rx_fqual_1.fp_ampl2, info.rx_fqual_1.pp_ampl3, info.rx_fqual_1.cir_pwr,
     info.rx_fqual_2.std_noise, info.rx_fqual_2.fp_ampl2, info.rx_fqual_2.pp_ampl3, info.rx_fqual_2.cir_pwr,
     info.rx_fqual_3.std_noise, info.rx_fqual_3.fp_ampl2, info.rx_fqual_3.pp_ampl3, info.rx_fqual_3.cir_pwr);
     nanosleep(&slptime, NULL);
     }
   } else if (tx_buff.payload[0] == '1') { //Transmit Message
     while (1) {
     ranging_send(&bus0, &sys_ctrl, &sta, &tx_buff, &info, interrupt_pin); 
     uint64_t t_round1 = time_sub(info.timestamp_rx_2, info.timestamp_tx_1);
     uint64_t t_prop = (t_round1 - T_REPLY) / 2;
     printf("I compute a tProp of %llu clock cycles\n", t_prop);
     double prop_in_secs = ((double) t_prop) / ((double) CLOCK_FREQ);
     printf("So the distance is %f meters\n", prop_in_secs * LIGHT_SPEED);
     nanosleep(&slptime, NULL);
     }
  }
 }
 return 0;
}
