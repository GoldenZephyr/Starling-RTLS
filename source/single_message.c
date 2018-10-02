#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "mac_comms.h"
#include "spi_comms.h"

#define STDOUT_FD 1

int main() {
  struct spi_bus bus0;
  char sel[5];
  printf("Select device (0/1):\n");
  char i_name[20];
  char* ret = fgets(sel, sizeof(sel), stdin);
  if (ret == NULL) {
    printf("End of file reached before input");
  } else if (sel[0] == '0') {
    memcpy(i_name,"/dev/spidev0.0",sizeof(i_name));
  } else if (sel[0] == '1') { 
    memcpy(i_name,"/dev/spidev1.0",sizeof(i_name));
  } else {
    return EXIT_FAILURE;
  }
  struct spi_ioc_transfer xfer0;

  struct tx_fctrl fctrl;
  struct tx_buffer tx_buff;
  struct system_conf sys_conf;
  struct system_control sys_ctrl;

  bus0.interface_name = i_name;
  bus0.xfer = &xfer0;
  if (spi_init(&bus0) != 0)
    exit(EXIT_FAILURE);
  //Check for proper SPI setup
  if (comms_check(&bus0) == 0) {
    printf("SPI Error: Cannot read device ID\n");
    exit(EXIT_FAILURE);
  }
  struct system_status sta;
  //Clear Status reg
  clear_status(&bus0, &sta);
  //Setup tx_fctrl
  frame_control_init(&fctrl);
  
  //Setup tx_buffer
  tx_buffer_init(&tx_buff);

  //Setup System Configure
  sys_conf_init(&sys_conf);
  unsigned char conf_rx[SYS_CONF_LEN] = {0x00};
  write_spi_msg(&bus0, conf_rx, &sys_conf, SYS_CONF_LEN);
  //Load Microcode
  unsigned char otp_ctrl_rx[4] = {0x00};
  unsigned char otp_ctrl_tx[4] = {0x00};
  otp_ctrl_tx[0] = 0x2D | 0xC0;
  otp_ctrl_tx[1] = 0x06;
  otp_ctrl_tx[3] = 0x80;
  write_spi_msg(&bus0, otp_ctrl_rx, otp_ctrl_tx, 0x04);

  //Get Payload - User Input

  //Initialize frame control / Device IDs
  const uint16_t pan_id = PAN_ID_LO | (PAN_ID_HI << 8);
  const uint16_t addr_id = ADDR_ID_LO | (ADDR_ID_HI << 8);
  if (decawave_comms_init(&bus0, pan_id, addr_id, &fctrl) == 1) {
    exit(EXIT_FAILURE);
  }
  //Confirm fctrl
  unsigned char tx_fc[TX_FCTRL_LEN] = {0x00};
  tx_fc[0] = TX_FCTRL_REG;
  unsigned char rx_fc[TX_FCTRL_LEN] = {0x00};
  write_spi_msg(&bus0, rx_fc, tx_fc, TX_FCTRL_LEN);
  printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_fc[1],rx_fc[2],rx_fc[3],rx_fc[4],rx_fc[5]);
  return 0;

  //Write in Payload
  printf("Type in payload (%lu chars max):\n", sizeof(tx_buff.payload));
  printf("No payload for receive mode\n");
  ret = fgets((char *) &tx_buff.payload, sizeof(tx_buff.payload), stdin);
  if (ret == NULL){
    printf("Error reading tx_buff.payload");
    exit(EXIT_FAILURE);
  } else {
    printf("%s", tx_buff.payload);
  }
   
  sys_ctrl_init(&sys_ctrl);
  if (tx_buff.payload[0] == '\n') {
    printf("Waiting for msg...\n");
    wait_for_msg(&bus0, &sys_ctrl);
  } else { //Transmit Message
    unsigned char rx_payload_buf[TX_BUFFER_LEN] = {0x00};
    write_spi_msg(&bus0, rx_payload_buf, &tx_buff, TX_BUFFER_LEN);
    //Transmit Message
    send_message(&bus0, &sys_ctrl);
  }
}

//Initializes tx_fctrl with our parameters
void frame_control_init(struct tx_fctrl *fctrl) {
  fctrl->reg      = TX_FCTRL_REG | WRITE;
  fctrl->tflen    = 0x7F; //Frame Length - 127 Bytes
  fctrl->tfle     = 0x00;  //Extended Frame - No
  fctrl->res_1    = 0x00;  //Reserved Bits - Write 0
  fctrl->txbr     = 0x02;  //Transmit Bitrate - 6.8Mbps
  fctrl->tr       = 0x01;  //Ranging Frame - Yes (Unused)
  fctrl->txprf    = 0x01;  //Transmit Preamble Repitition Rate - 16Mhz
  fctrl->txpsr    = 0x01;
  fctrl->pe       = 0x01;  //Preamble Length Selection - 128 Symbols
  fctrl->txbodds  = 0x00;  //Transmit Buffer Offset - 0 Bytes
  fctrl->ifsdelay = 0x00;  //Minimum Time Between Frame Sends - 0 Symbols
  fctrl->res_2    = 0x00;  //Reserved Bits - Write 0 

/*  fctrl->tflen    = 0x7F; //Frame Length - 127 Bytes
  fctrl->tfle     = 0x7;  //Extended Frame - No
  fctrl->res_1    = 0x7;  //Reserved Bits - Write 0
  fctrl->txbr     = 0x3;  //Transmit Bitrate - 110 kbps
  fctrl->tr       = 0x1;  //Ranging Frame - Yes (Unused)
  fctrl->txprf    = 0x3;  //Transmit Preamble Repitition Rate - 64Mhz
  fctrl->txpsr    = 0x3;
  fctrl->pe       = 0x3;  //Preamble Length Selection - 4096 Symbols
  fctrl->txbodds  = 0x3FF;  //Transmit Buffer Offset - 0 Bytes
  fctrl->ifsdelay = 0xFF;  //Minimum Time Between Frame Sends - 0 Symbols
  fctrl->res_2    = 0xFFFFFF;  //Reserved Bits - Write 0
*/
}

void tx_buffer_init(struct tx_buffer *tx_buff) {
  tx_buff->reg = TX_BUFFER_REG;
  struct mac_header *mac = &tx_buff->mac_header;
  struct frame_control *fcs = &mac->frame_control;

  //Setup Frame_Contol
  fcs->frame_type = 0x01; //Frame Type - Data
  fcs->security_enabled = 0x00; //Security Enabled - Nah man
  fcs->frame_pending = 0x00; //More Data Incoming - No
  fcs->ack_request = 0x00; //Request Acknowledge Message - No
  fcs->pan_id_compress = 0x01; //PAN ID Compression - No
  fcs->reserved = 0x00; //Reserved Bits - Write 0
  fcs->dest_addr_mode = 0x02; //Destination Address Mode - Short (16-bit)
  fcs->frame_version = 0x01; //Frame Version - IEEE 802.15.4 TODO: ???
  fcs->source_address_mode = 0x02; //Source Addressing Mode - Short (16-bit)
  
  //Setup MAC Header
  mac->sequence_number = 0x00; //Sequence # TODO: ???
  mac->dest_pan_id = 0x00; //Will be updated on send
  mac->dest_addr = 0x00; //Will be updated on send
  mac->source_pan_id = PAN_ID_LO | (PAN_ID_HI << 8);
  mac->source_addr = ADDR_ID_LO | (ADDR_ID_HI << 8);
}


void sys_ctrl_init(struct system_control *ctrl) {
  ctrl->reg = SYS_CTRL_REG | WRITE;
  ctrl->sfcst = 0; //Auto append FCS - Default No
  ctrl->txstrt = 0; //Start Transmittion - Default No
  ctrl->txdlys = 0; //Transmission Delay - Default No
  ctrl->cansfcs = 0; //High Speed Transmission - Default No
  ctrl->res = 0; //Reserved Bits - Write 0
  ctrl->trxoff = 0; //Disable Transeiver - Default No
  ctrl->waitresp = 0; //Enable Receiver after sending - Default No
  ctrl->rxenab = 0; //Enable Receiver - Default No
  ctrl->rxdlye = 0; //Receive Delay - Default No
  ctrl->res_2 = 0; //Reserved Bits - Write 0
  ctrl->hrbpt = 0; //Double Buffer Mode - Default 0
  ctrl->res_3 = 0; //Reserved Bits - Write 0
}

void sys_conf_init(struct system_conf *conf) {
  conf->reg = SYS_CONF_REG | WRITE;
  conf->ffen = 0; //Frame filtering enable - no
  conf->ffbc = 1; //FF as Coordinator - yes
  conf->ffab = 1; //FF Allow Beacon - yes
  conf->ffad = 1; //FF Allow Data - yes
  conf->ffaa = 1; //FF Allow Ack - yes
  conf->ffam = 1; //FF Allow MAC Command - yes
  conf->ffar = 1; //FF Allow Reserved - yes
  conf->ffaf = 1; //FF Allow Frames with type field 4 - Yes
  conf->ffav = 1; //FF Allow " " " " 5 - Yes
  conf->hirq = 1; //Use Interrupt GPIO - Yes - Active High
  conf->spiedge = 0; //SPI Edge mode - Default
  conf->disfce = 0; //Disable error handling - No
  conf->disdrxb = 1; //Disable Double Buffer - Yes
  conf->disphe = 0; //Disable on PHR error - Yes
  conf->disrsde = 0; //Disable receiver on RSD Error - yes
  conf->fcsinit = 0; //FCS Seed mode - Default
  conf->phrmode = 0x00; //PHR - Standard Frame
  conf->disstxp = 0; //Smart TX Power - Default
  conf->res = 0; //Reserved - Write 0
  conf->rxmk = 0; //110kbps mode - No
  conf->res_2 = 0; //Reserved - Write 0
  conf->rxwtoe = 0; //Timeout Receiver - No
  conf->rxautr = 1; //Auto-reenable receive - Yes
  conf->autoack = 0; //Auto-Ackknowledge - No
  conf->aackpend = 0; //Auto-Append - No
}

void send_message(struct spi_bus *bus, struct system_control *ctrl) {
  //Setup System Control to send
  ctrl->txstrt = 0x01;
  unsigned char rx_sys_ctrl[SYS_CTRL_LEN];
  write_spi_msg(bus, rx_sys_ctrl, &ctrl, SYS_CTRL_LEN); //IT IS SENT

  while(1) {
    unsigned char tx_status[SYS_STATUS_LEN] = {0x00};
    tx_status[0] = SYS_STATUS_REG;
    unsigned char rx_status[SYS_STATUS_LEN] = {0x00};
    write_spi_msg(bus, rx_status, tx_status, SYS_STATUS_LEN);
    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_status[1], rx_status[2], rx_status[3], rx_status[4], rx_status[5]);
    struct timespec t;
    t.tv_sec = 1;
    t.tv_nsec = 0;
    nanosleep(&t, NULL);
    }
}

void wait_for_msg(struct spi_bus * bus, struct system_control *ctrl) {
  //Turn on receiver
  ctrl->rxenab = 0x01;
  unsigned char rx_sys_ctrl[SYS_CTRL_LEN];
  write_spi_msg(bus, rx_sys_ctrl, &ctrl, SYS_CTRL_LEN); //RECEIVER ON
  //Wait for msg
  
  //TODO: Interrupts, for now, we will poll
  //Clear Status Register 
  struct system_status sta;
  clear_status(bus, &sta);
  //Poll Status Event Register
  while(1) {
    unsigned char tx_status[SYS_STATUS_LEN] = {0x00};
    tx_status[0] = SYS_STATUS_REG;
    unsigned char rx_status[SYS_STATUS_LEN] = {0x00};
    write_spi_msg(bus, rx_status, tx_status, SYS_STATUS_LEN);
    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_status[1], rx_status[2], rx_status[3], rx_status[4], rx_status[5]);
    struct timespec t;
    t.tv_sec = 1;
    t.tv_nsec = 0;
    nanosleep(&t, NULL);
  }
}


//Checks that we can read the device ID
//Returns 1 if we can, 0 if we cannot
int comms_check(struct spi_bus * const bus) {
  unsigned char rx_buf[DEV_ID_LEN] = {0x00};
  unsigned char tx_buf[DEV_ID_LEN] = {0x00};
  if (write_spi_msg(bus, rx_buf, tx_buf, DEV_ID_LEN) < 0) {
    return 0;
  }
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
  struct spi_bus * bus, unsigned char * const rx, const void * const tx, int len) {
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


//Initialize Communications
int decawave_comms_init(struct spi_bus * const bus, const uint16_t pan_id,
  const uint16_t addr_id, const struct tx_fctrl *fctrl) {
  //Write the PAN ID and Addr ID
  struct panaddr pan_msg;
  pan_msg.reg = PANADDR_REG | WRITE;
  pan_msg.pan_id = pan_id;
  pan_msg.addr_id = addr_id;
  unsigned char rx_buf[PANADDR_LEN] = {0x00};
  write_spi_msg(bus, rx_buf, &pan_msg, PANADDR_LEN);
 (void) pan_msg; 
  // Write Check - Unit Test
  char tx_buf[PANADDR_LEN] = {PANADDR_REG, 0x00, 0x00, 0x00, 0x00};
  write_spi_msg(bus, rx_buf, tx_buf, PANADDR_LEN);
  (void) tx_buf;
  printf("0x%02X 0x%02X 0x%02X 0x%02X\n",
    rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
  if ((rx_buf[1] | (rx_buf[2] << 8)) != pan_id) {
    printf("Did not set PAN_ID (reading 0x%02X 0x%02X)\n", rx_buf[1], rx_buf[2]);
    return 1;
  }
  if ((rx_buf[3] | (rx_buf[4] << 8)) != addr_id) {
    printf("Did not set ADDR_ID (reading 0x%02X 0x%02X)\n", rx_buf[1], rx_buf[2]);
//    return 1;
  }


  //Write the tx_fctrl register
  unsigned char rx_fctrl_buf[TX_FCTRL_LEN] = {0x00};
  write_spi_msg(bus, rx_fctrl_buf, fctrl, TX_FCTRL_LEN);
/*
  char fctrl_buf[sizeof(struct tx_fctrl)];
  memcpy(fctrl_buf, fctrl, sizeof(struct tx_fctrl));
  printf("%02X\n", fctrl->tfle);

  printf("0x");
  for (unsigned int i = 0; i < sizeof(struct tx_fctrl); i++) {
    printf("%02X ", (unsigned)fctrl_buf[i]);
  }
  printf("\n");
*/

 return 0;
}


int write_payload(struct spi_bus * const bus,
              struct mac_header * const mac,
              const unsigned char * const payload,
              const int len,
              const uint64_t timestamp) {
  (void) bus;
  (void) mac;
  (void) payload;
  (void) len;
  (void) timestamp;
  return 0;
}


void clear_status(struct spi_bus *bus, struct system_status *sta) {
  sta->reg = SYS_STATUS_REG | WRITE;
  sta->irqs = 0;
  sta->cplock = 1;
  sta->esyncr = 1;
  sta->aat = 1;
  sta->txfrb = 1;
  sta->txprs = 1;
  sta->txphs = 1;
  sta->txfrs = 1;
  sta->rxprd = 1;
  sta->rxsfdd = 1;
  sta->ldedone = 1;
  sta->rxphd = 1;
  sta->rxphe = 1;
  sta->rxdfr = 1;
  sta->rxfcg = 1;
  sta->rxfce = 1;
  sta->rxrfsl = 1;
  sta->rxrfto = 1;
  sta->ldeerr = 1;
  sta->res = 0;
  sta->rxovrr = 0;
  sta->rxpto = 1;
  sta->gpioirq = 1;
  sta->slpinit = 0;
  sta->rfpll = 1;
  sta->clkpll = 1;
  sta->rxsfdto = 1;
  sta->hpdwarn = 0;
  sta->txberr = 1;
  sta->affrej = 1;
  sta->hsrbp = 0;
  sta->icrbp = 0;
  sta->rxrscs = 1;
  sta->rxprej = 1;
  sta->txpute = 0;
  sta->res_2 = 0;
  unsigned char rx_buf[SYS_STATUS_LEN];
  write_spi_msg(bus, rx_buf, sta, SYS_STATUS_LEN);
}
