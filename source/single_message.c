#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include "mac_comms.h"
#include "spi_comms.h"

int main() {
  struct spi_bus bus0;
  const char *i_name = "/dev/spidev0.0";
  bus0.interface_name = i_name;
  struct spi_ioc_transfer xfer0;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  //Check for proper SPI setup
  if (comms_check(&bus0) == 0) {
    printf("SPI Error: Cannot read device ID\n");
    exit(EXIT_FAILURE);
  }
  //Initialize Comms
  decawave_comms_init(&bus0, 0xAAAA, 0xBBBB, NULL);
  return 0;
}

//Checks that we can read the device ID
//Returns 1 if we can, 0 if we cannot
int comms_check(struct spi_bus * const bus) {
  char rx_buf[DEV_ID_LEN] = {0x00};
  char tx_buf[DEV_ID_LEN] = {0x00};
  write_spi_msg(bus, rx_buf, tx_buf, DEV_ID_LEN);
  return (rx_buf[1] == 0x30 && rx_buf[2] == 0x01 
           && rx_buf[3] == 0xCA && rx_buf[4] == 0xDE);

}

int spi_init(struct spi_bus *bus) {
  if ((bus->spi_fd = open(bus->interface_name, O_RDWR)) < 0) {
    perror("Failed to open spidev");
    return 1;
  }
  bus->xfer->speed_hz = 3000000;
  return 0;
}

int write_spi_msg(
  struct spi_bus *bus, char * const rx, const void * const tx, int len) {
  bus->xfer->rx_buf = (unsigned long) rx;
  bus->xfer->tx_buf = (unsigned long) tx;
  bus->xfer->len = len;
  int ret = ioctl(bus->spi_fd, SPI_IOC_MESSAGE(1), bus->xfer);
  if (ret < 0) {
    perror("Error sending");
    return 1;
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
  char rx_buf[TX_FCTRL_LEN] = {0x00};
  write_spi_msg(bus, rx_buf, fctrl, TX_FCTRL_LEN);
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
