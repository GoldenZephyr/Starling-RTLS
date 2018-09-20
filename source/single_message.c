#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include "single_message.h"
#include <stdlib.h>
#define BUF_LEN 5

int main() {
  struct spi_bus bus0;
  const char *i_name = "/dev/spidev0.0";
  bus0.interface_name = i_name;
  struct spi_ioc_transfer xfer0;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  
  
  char rx_buf[BUF_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  char tx_buf[BUF_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};

  //Open file Descriptor
  //Send Message
  write_spi_msg(&bus0, rx_buf, tx_buf, BUF_LEN);
  printf("0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]); 
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

int write_spi_msg(struct spi_bus *bus, char *rx, char *tx, int len) {
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
