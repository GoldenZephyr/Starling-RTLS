// spi_comms.h -- SPI Communication
#include <stdint.h>

const unsigned int SYS_TIME_REG = 0x06;
const int SYS_TIME_LEN = 6;

struct spi_bus {
  const char *interface_name;
  int spi_fd;
  struct spi_ioc_transfer *xfer;
};

int spi_init(struct spi_bus *bus);
int write_spi_msg(struct spi_bus *bus,
                  char * const rx,
                  const char * const tx,
                  int len);

int write_msg(struct spi_bus *bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp);


