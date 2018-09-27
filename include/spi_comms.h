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

#define TX_BUFFER_REG 0x09
#define TX_BUFFER_LEN 128

//TODO: Standardize this
#define PAYLOAD_LEN 114


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

//TX Fctrl - 9 Bytes
struct __attribute__((packed)) tx_fctrl {
  unsigned int reg : 8;
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

//Frame Control - 2 Bytes
struct __attribute__((__packed__)) frame_control {
 unsigned int frame_type : 3;
 unsigned int security_enabled : 1;
 unsigned int frame_pending : 1;
 unsigned int ack_request : 1;
 unsigned int pan_id_compress : 1;
 unsigned int reserved : 3;
 unsigned int dest_addr_mode : 2;
 unsigned int frame_version : 2;
 unsigned int source_address_mode : 2;
};

//MAC Header - 11 Bytes
struct __attribute__((__packed__)) mac_header {
  struct frame_control frame_control;
  uint8_t sequence_number;
  uint16_t dest_pan_id;
  uint16_t dest_addr;
  uint16_t source_pan_id;
  uint16_t source_addr;
};

//TX Buffer - 127 Octets
struct __attribute__((__packed__)) tx_buffer {
  uint8_t reg;
  struct mac_header mac_header;
  uint8_t payload[114]; //127 - 11 - 2 = 114 Bytes
};

int spi_init(struct spi_bus * const bus);

int comms_check(struct spi_bus * const bus);

int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id, const struct tx_fctrl * const fctrl);

void frame_control_init(struct tx_fctrl *fctrl);

void tx_buffer_init(struct tx_buffer *tx_buff);

int write_spi_msg(struct spi_bus *bus,
                  char * const rx,
                  const void * const tx,
                  int len);

int write_payload(struct spi_bus *bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp);




