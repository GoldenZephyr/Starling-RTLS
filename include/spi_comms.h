// spi_comms.h -- SPI Communication
#include <stdint.h>

#define WRITE 0x80

#define SYS_TIME_REG 0x06
#define SYS_TIME_LEN 6

#define PANADDR_REG 0x03
#define PANADDR_LEN 5

#define PAN_ID_LO 0x12
#define PAN_ID_HI 0x34
#define ADDR_ID_LO 0x13
#define ADDR_ID_HI 0x37

struct spi_bus {
  const char *interface_name;
  int spi_fd;
  struct spi_ioc_transfer *xfer;
};

int spi_init(struct spi_bus *bus);

int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id);

int write_spi_msg(struct spi_bus *bus,
                  char * const rx,
                  const char * const tx,
                  int len);

int write_payload(struct spi_bus *bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp);


