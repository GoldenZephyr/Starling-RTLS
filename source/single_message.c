#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include "mac_comms.h"
#include "spi_comms.h"

#define STDOUT_FD 1

int main() {
  struct spi_bus bus0;
  const char *i_name = "/dev/spidev0.0"; 
  struct spi_ioc_transfer xfer0;

  struct tx_fctrl fctrl;
  struct tx_buffer tx_buff;

  bus0.interface_name = i_name;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  //Check for proper SPI setup
  if (comms_check(&bus0) == 0) {
    printf("SPI Error: Cannot read device ID\n");
    exit(EXIT_FAILURE);
  }

  //Setup tx_fctrl
  frame_control_init(&fctrl);
  
  //Setup tx_buffer



//Initialize Comms
  printf("%d %d\n", sizeof(struct mac_header), sizeof(struct frame_control));
  
 // decawave_comms_init(&bus0, 0xAAAA, 0xBBBB, &fctrl);
  return 0;
}

//Initializes tx_fctrl with our parameters
void frame_control_init(struct tx_fctrl *fctrl) {
  fctrl->reg = TX_FCTRL_REG | WRITE;
  fctrl->tflen    = 0x7F; //Frame Length - 127 Bytes
  fctrl->tfle     = 0x0;  //Extended Frame - No
  fctrl->res_1    = 0x0;  //Reserved Bits - Write 0
  fctrl->txbr     = 0x0;  //Transmit Bitrate - 110 kbps
  fctrl->tr       = 0x1;  //Ranging Frame - Yes (Unused)
  fctrl->txprf    = 0x2;  //Transmit Preamble Repitition Rate - 64Mhz
  fctrl->txpsr    = 0x3;
  fctrl->pe       = 0x0;  //Preamble Length Selection - 4096 Symbols
  fctrl->txbodds  = 0x0;  //Transmit Buffer Offset - 0 Bytes
  fctrl->ifsdelay = 0x0;  //Minimum Time Between Frame Sends - 0 Symbols
  fctrl->res_2    = 0x0;  //Reserved Bits - Write 0 */
}

void tx_buffer_init(struct tx_buffer *tx_buff) {
   
  tx_buff->reg = TX_BUFFER_REG;
  //Setup Frame_Contol
  struct frame_control *fcs = &tx_buff->mac_header.frame_control
  fcs->frame_type = 0x01; //Frame Type - Data
  fcs->security_enabled = 0x00; //Security Enabled - Nah man
  fcs->frame_pending = 0x00; //More Data Incoming - No
  fcs->ack_request = 0x00l //Request Acknowledge Message - No
  fcs->pan_id_compress = 
}

//Checks that we can read the device ID
//Returns 1 if we can, 0 if we cannot
int comms_check(struct spi_bus * const bus) {
  char rx_buf[DEV_ID_LEN] = {0x00};
  char tx_buf[DEV_ID_LEN] = {0x00};
  if (write_spi_msg(bus, rx_buf, tx_buf, DEV_ID_LEN) < 0)
    return 0;
  printf("0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
  return (rx_buf[1] == 0x30 && rx_buf[2] == 0x01 
           && rx_buf[3] == 0xCA && rx_buf[4] == 0xDE);

}

int spi_init(struct spi_bus *bus) {
  if ((bus->spi_fd = open(bus->interface_name, O_RDWR)) < 0) {
    perror("Failed to open spidev");
    return 1;
  }
  bus->xfer->speed_hz = 3000000; 
  bus->xfer->bits_per_word = 8;
  bus->xfer->delay_usecs = 0;
  bus->xfer->cs_change = 0;
  bus->xfer->pad = 0;
  bus->xfer->tx_nbits = 0;
  bus->xfer->rx_nbits = 0;
  return 0;
}

int write_spi_msg(
  struct spi_bus * bus, char * const rx, const void * const tx, int len) {
  bus->xfer->rx_buf = (unsigned long) rx;
  bus->xfer->tx_buf = (unsigned long) tx;
  bus->xfer->len = len;
  int ret = ioctl(bus->spi_fd, SPI_IOC_MESSAGE(1), bus->xfer);
  if (ret < 0) {
    perror("Error sending");
    return -1;
  }
  return ret;
}

int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id, const struct tx_fctrl *fctrl) {
  //Write the PAN ID and Addr ID
  struct panaddr pan_msg;
  pan_msg.reg = PANADDR_REG | WRITE;
  pan_msg.pan_id = pan_id;
  pan_msg.addr_id = addr_id;
  char rx_buf[PANADDR_LEN] = {0x00};
  write_spi_msg(bus, rx_buf, &pan_msg, PANADDR_LEN);
  
  /* Write Check | TODO: Make Unit Test 
  char tx_buf[PANADDR_LEN] = {PANADDR_REG, 0x00, 0x00, 0x00, 0x00};
  write_spi_msg(bus, rx_buf, tx_buf, PANADDR_LEN); 
  printf("0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]); */

  //Write the tx_fctrl register
  (void) fctrl;
  //char rx_buf[TX_FCTRL_LEN] = {0x00};
  //write_spi_msg(bus, rx_buf, fctrl, TX_FCTRL_LEN);
 return 0;
}


int write_payload(struct spi_bus * const bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp) {
  (void) bus;
  (void) mac;
  (void) payload;
  (void) len;
  (void) timestamp;
  //Write PAN and Addr IDs
  //Writes IDs for PAN and Addr
  
  //Write Payload to Data Buffer


  return 0;
}
