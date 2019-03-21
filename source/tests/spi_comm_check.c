#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#define BUF_LEN 5

int main() {
  char *filename = "/dev/spidev0.0";
  int spi_fd = 0;
  char rx_buf[BUF_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  char tx_buf[BUF_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00};
  struct spi_ioc_transfer xfer;
  xfer.tx_buf = (unsigned long) &tx_buf;
  xfer.rx_buf = (unsigned long) &rx_buf;
  xfer.len = BUF_LEN;
  xfer.speed_hz = 3000000;

  //Open file Descriptor
  if ((spi_fd = open(filename, O_RDWR)) < 0) {
    perror("Failed to open spidev");
    return 0;
  }
  //Send Message
  int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer);
  if (ret < 0) {
    perror("Error sending");
    return 1;
  }
  printf("0x%02X 0x%02X 0x%02X 0x%02X\n",
         rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
  return 0;
}

