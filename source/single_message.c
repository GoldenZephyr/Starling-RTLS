#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include "mac_comms.h"
#include "spi_comms.h"
#define BUF_LEN 6

int main() {
  struct spi_bus bus0;
  const char *i_name = "/dev/spidev0.0";
  bus0.interface_name = i_name;
  struct spi_ioc_transfer xfer0;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  
  
  char rx_buf[BUF_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  char tx_buf[BUF_LEN] = {0x06, 0x00, 0x00, 0x00, 0x00};

  //Open file Descriptor
  //Send Message
  write_spi_msg(&bus0, rx_buf, tx_buf, BUF_LEN);
  printf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
    rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]); 
  return 0;
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
  struct spi_bus *bus, char * const rx, const char * const tx, int len) {
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
  const uint16_t addr_id) {
  char tx_buf[PANADDR_LEN] = {PANADDR_REG | WRITE,
                              (uint8_t) pan_id, (uint8_t) pan_id >> 8,
                              (uint8_t) addr_id, (uint8_t) addr_id >> 8};
  char rx_buf[PANADDR_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  write_spi_msg(bus, rx_buf, tx_buf, PANADDR_LEN);
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
