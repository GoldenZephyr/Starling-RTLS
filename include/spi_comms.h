// spi_comms.h -- SPI Communication
#include <stdint.h>

#define WRITE 0x80

#define DEV_ID_REG 0x00
#define DEV_ID_LEN 5

#define SYS_TIME_REG 0x06
#define SYS_TIME_LEN 6

#define PANADDR_REG 0x03
#define PANADDR_LEN 5

#define SYS_CONF_REG 0x04
#define SYS_CONF_LEN 5

#define PAN_ID_LO 0x12
#define PAN_ID_HI 0x34
#define ADDR_ID_LO 0x13
#define ADDR_ID_HI 0x37

#define TX_FCTRL_REG 0x08
#define TX_FCTRL_LEN 6

#define TX_BUFFER_REG 0x09
#define TX_BUFFER_LEN 128

#define SYS_CTRL_REG 0x0D
#define SYS_CTRL_LEN 5

#define SYS_STATUS_REG 0x0F
#define SYS_STATUS_LEN 6

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

//System Control - 4 Bytes
struct __attribute__((__packed__)) system_control {
  unsigned int reg : 8;
  unsigned int sfcst : 1;
  unsigned int txstrt : 1;
  unsigned int txdlys : 1;
  unsigned int cansfcs : 1;
  unsigned int res : 2;
  unsigned int trxoff : 1;
  unsigned int waitresp : 1;
  unsigned int rxenab : 1;
  unsigned int rxdlye : 1;
  unsigned int res_2 : 14;
  unsigned int hrbpt : 1;
  unsigned int res_3 : 7;
};

//System Configure - 4 Bytes
struct __attribute__((__packed__)) system_conf {
  unsigned int reg : 8;
  unsigned int ffen : 1;
  unsigned int ffbc : 1;
  unsigned int ffab : 1;
  unsigned int ffad : 1;
  unsigned int ffaa : 1;
  unsigned int ffam : 1;
  unsigned int ffar : 1;
  unsigned int ffaf : 1;
  unsigned int ffav : 1;
  unsigned int hirq : 1;
  unsigned int spiedge : 1;
  unsigned int disfce : 1;
  unsigned int disdrxb : 1;
  unsigned int disphe : 1;
  unsigned int disrsde : 1;
  unsigned int fcsinit : 1;
  unsigned int phrmode : 2;
  unsigned int disstxp : 1;
  unsigned int res : 3;
  unsigned int rxmk : 1;
  unsigned int res_2 : 5;
  unsigned int rxwtoe : 1;
  unsigned int rxautr : 1;
  unsigned int autoack : 1;
  unsigned int aackpend : 1;
};

//System Status - 5 Bytes
struct __attribute__((__packed__)) system_status {
  unsigned int reg : 8;
  unsigned int irqs : 1;
  unsigned int cplock : 1;
  unsigned int esyncr : 1;
  unsigned int aat : 1;
  unsigned int txfrb : 1;
  unsigned int txprs : 1;
  unsigned int txphs : 1;
  unsigned int txfrs : 1;
  unsigned int rxprd : 1;
  unsigned int rxsfdd : 1;
  unsigned int ldedone : 1;
  unsigned int rxphd : 1;
  unsigned int rxphe : 1;
  unsigned int rxdfr : 1;
  unsigned int rxfcg : 1;
  unsigned int rxfce : 1;
  unsigned int rxrfsl : 1;
  unsigned int rxrfto : 1;
  unsigned int ldeerr : 1;
  unsigned int res : 1;
  unsigned int rxovrr : 1;
  unsigned int rxpto : 1;
  unsigned int gpioirq : 1;
  unsigned int slpinit : 1;
  unsigned int rfpll : 1;
  unsigned int clkpll : 1;
  unsigned int rxsfdto : 1;
  unsigned int hpdwarn : 1;
  unsigned int txberr : 1;
  unsigned int affrej : 1;
  unsigned int hsrbp : 1;
  unsigned int icrbp : 1;
  unsigned int rxrscs : 1;
  unsigned int rxprej : 1;
  unsigned int txpute : 1;
  unsigned int res_2 : 5;
};

int spi_init(struct spi_bus * const bus);

int comms_check(struct spi_bus * const bus);

int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id, const struct tx_fctrl * const fctrl);

void frame_control_init(struct tx_fctrl *fctrl);

void tx_buffer_init(struct tx_buffer *tx_buff);

void sys_conf_init(struct system_conf *conf);

void sys_ctrl_init(struct system_control *ctrl);

void send_message(struct spi_bus *bus, struct system_control *ctrl);

void wait_for_msg(struct spi_bus *bus, struct system_control *ctrl);

void clear_status(struct spi_bus *bus, struct system_status *sta);

int write_spi_msg(struct spi_bus *bus,
                  char * const rx,
                  const void * const tx,
                  int len);

int write_payload(struct spi_bus *bus,
              struct mac_header * const mac,
              const char * const payload,
              const int len,
              const uint64_t timestamp);




