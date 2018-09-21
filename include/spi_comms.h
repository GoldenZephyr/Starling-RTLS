// spi_comms.h -- SPI Communication
#include <stdint.h>

#define WRITE 0x80

#define DEV_ID_REG 0x00
#define DEV_ID_LEN 5

#define SYS_TIME_REG 0x06
#define SYS_TIME_LEN 6

#define PANADDR_REG 0x03
#define PANADDR_LEN 5

#define PAN_ID_LO 0x12
#define PAN_ID_HI 0x34
#define ADDR_ID_LO 0x13
#define ADDR_ID_HI 0x37

#define TX_FCTRL_REG 0x08
#define TX_FCTRL_LEN 6

struct spi_bus {
  const char *interface_name;
  int spi_fd;
  struct spi_ioc_transfer *xfer;
};

struct __attribute__((__packed__)) panaddr {
  uint8_t reg;
  uint16_t pan_id;
  uint16_t addr_id;
};

struct __attribute__((__packed__)) tx_fctrl {
  uint8_t reg;
  unsigned int tflen : 7;
  unsigned int tfle : 3;
  unsigned int res_1 : 3;
  unsigned int txbr : 2;
  unsigned int tr : 1;
  unsigned int txprf : 2;
  unsigned int txpsr : 2;
  unsigned int pe : 2;
  unsigned int txbodds : 10;
  unsigned int ifsdelay : 8;
  unsigned int res_2 : 24;
};

int spi_init(struct spi_bus * const bus);

int comms_check(struct spi_bus * const bus);

int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id, const struct tx_fctrl * const fctrl);

int write_spi_msg(struct spi_bus *bus,
                  char * const rx,
                  const void * const tx,
                  int len);

int write_payload(struct spi_bus *bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp);


