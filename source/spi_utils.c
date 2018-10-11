#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <string.h>
#include <wiringPi.h>
#include <signal.h>

#include "spi_comms.h"
#define STDOUT_FD 1

#ifdef DEBUG
#define debug_print(...) printf(...)
#else
#define debug_print(...)
#endif

  //Interrupt Flag
  volatile sig_atomic_t interrupt_flag = 0;

  void gpio_interrupt(void){
    interrupt_flag = 1;
  }

  //Initializes tx_fctrl with our parameters
  void frame_control_init(struct tx_fctrl *fctrl) {
    fctrl->reg      = TX_FCTRL_REG | WRITE;
    fctrl->tflen    = 0x0B + PAYLOAD_LEN; //Frame Length - MAC Header + Payload
    fctrl->tfle     = 0x00;  //Extended Frame - No
    fctrl->res_1    = 0x00;  //Reserved Bits - Write 0
    fctrl->txbr     = 0x02;  //Transmit Bitrate - 6.8Mbps
    fctrl->tr       = 0x00;  //Ranging Frame - No (Unused)
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
    tx_buff->reg = TX_BUFFER_REG | WRITE;
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
    mac->sequence_number = 0x01; //Sequence # TODO: ???
    mac->dest_pan_id = PAN_ID_LO | (PAN_ID_HI << 8);
    mac->dest_addr = PAN_ID_LO | (PAN_ID_HI << 8);
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

  //Interrupt GPIO Mask (Masked = Will show)
  void sys_mask_init(struct spi_bus *bus, struct system_mask *mask) {
    memset(mask, 0x00, sizeof(struct system_mask));
    mask->reg = SYS_MASK_REG | WRITE;
    mask->mrxdfr = 0x01; //Mask Data Frame Ready
    char mask_rx[SYS_MASK_LEN];
    write_spi_msg(bus, mask_rx, mask, SYS_MASK_LEN);
  }

  void send_message(struct spi_bus *bus, struct system_control *ctrl) {
    //Setup System Control to send 
    sys_ctrl_init(ctrl); 
    ctrl->txstrt = 0x01;
    unsigned char rx_sys_ctrl[SYS_CTRL_LEN];
    write_spi_msg(bus, rx_sys_ctrl, ctrl, SYS_CTRL_LEN); //IT IS SENT
    return;
  }

  void send_message_delay(struct spi_bus *bus, struct system_control *ctrl,
    struct dx_time *delay_time, int reenb_transmitter) {
    //Write in delay time
    char rx_dx_time[DX_TIME_LEN] = {0x00};
    delay_time->reg = DX_TIME_REG | WRITE;
    write_spi_msg(bus, rx_dx_time, delay_time, DX_TIME_LEN);
    //Setup System Control to send 
    sys_ctrl_init(ctrl); 
    ctrl->txstrt = 0x01; //Set sending flag
    ctrl->txdlys = 0x01; //Set delayed send
    ctrl->waitresp = reenb_transmitter; //Automatically wait for next msg
    unsigned char rx_sys_ctrl[SYS_CTRL_LEN];
    write_spi_msg(bus, rx_sys_ctrl, ctrl, SYS_CTRL_LEN); //IT WILL BE SENT
    return;
  }

  void set_pll(struct spi_bus *bus) {
    struct ec_ctrl pll_ctrl;
    pll_ctrl.reg = EC_CTRL_REG | WRITE;
    pll_ctrl.ostsm = 0x00;
    pll_ctrl.osrsm = 0x00;
    pll_ctrl.pllldt = 0x01; //Set
    pll_ctrl.wait = 0x00;
    pll_ctrl.ostrm = 0x00;
    pll_ctrl.res = 0x00;
    char ec_ctrl_rx[EC_CTRL_LEN];
    write_spi_msg(bus, ec_ctrl_rx, &pll_ctrl, EC_CTRL_LEN);
  }

  void load_microcode(struct spi_bus *bus) {
    struct timespec slptime;
    slptime.tv_sec = 0;
    slptime.tv_nsec = 200000;
    
    unsigned char tx_temp[4] = {0x00};
    unsigned char rx_temp[4] = {0x00};
    tx_temp[0] = 0x36 | WRITE | SUB_INDEX;
    tx_temp[1] = 0x00; //Sub Index 0x00
    tx_temp[2] = 0x01;
    tx_temp[3] = 0x03;
    write_spi_msg(bus, rx_temp, tx_temp, 4);
    tx_temp[0] = 0x2D | WRITE | SUB_INDEX;
    tx_temp[1] = 0x06;
    tx_temp[2] = 0x00;
    tx_temp[3] = 0x80;
    write_spi_msg(bus, rx_temp, tx_temp, 4);
    nanosleep(&slptime, NULL);
    tx_temp[0] = 0x36 | WRITE | SUB_INDEX;
    tx_temp[1] = 0x00; //Sub Index 0x00
    tx_temp[2] = 0x00;
    tx_temp[3] = 0x02;
    write_spi_msg(bus, rx_temp, tx_temp, 4);
  }

  void wait_for_msg_int(struct spi_bus *bus, struct system_control *ctrl,
    struct system_status *sta, int interrupt_pin) {
    //Clear Status Reg  
    clear_status(bus, sta);

    struct timespec slptime;
    slptime.tv_sec = 0;
    slptime.tv_nsec = 10000;
    
    //Turn on receiver
    sys_ctrl_init(ctrl); 
    unsigned char rx_sys_ctrl[SYS_CTRL_LEN] = {0x00};
    ctrl->rxenab = 0x01;
    write_spi_msg(bus, rx_sys_ctrl, ctrl, SYS_CTRL_LEN); //RECEIVER ON
    
    //Wait for GPIO Interrpt
    //TODO: Interrupt, for now we will poll
    unsigned char tx_status[SYS_STATUS_LEN] = {0x00};
  //unsigned char rx_status[SYS_STATUS_LEN] = {0x00};
    tx_status[0] = SYS_STATUS_REG;
    while (1) {
      write_spi_msg(bus, sta, tx_status, SYS_STATUS_LEN); 
  //    memcpy(rx_status, sta, SYS_STATUS_LEN);
  //    debug_print("0x%02X 0x%02X 0x%02X 0x%02X\n", rx_status[1]
  //    ,rx_status[2], rx_status[3], rx_status[4]);
      if (sta->rxdfr) {  //Check Data Ready Status 
        //Clear Status Pin
        clear_status(bus, sta);
        break;
      }
      nanosleep(&slptime, NULL);
    }

    /*
    wiringPiISR(interrupt_pin, INT_EDGE_RISING, gpio_interrupt);
    //Wait for Interrupt
    while(interrupt_flag == 0) {};
    //Clear Flag
    interrupt_flag = 0;
    */
    (void) interrupt_pin;
  }

  void wait_for_msg(struct spi_bus * bus, struct system_control *ctrl) {
    
    struct timespec slptime;
    slptime.tv_sec = 1;
    slptime.tv_nsec = 0;

    //Turn on receiver
    sys_ctrl_init(ctrl); 
    unsigned char rx_sys_ctrl[SYS_CTRL_LEN] = {0x00};
    ctrl->rxenab = 0x01;
    write_spi_msg(bus, rx_sys_ctrl, ctrl, SYS_CTRL_LEN); //RECEIVER ON
    nanosleep(&slptime, NULL);
    
    //Wait for msg
    //TODO: Interrupts, for now, we will poll
    //Clear Status Register 
    //Poll Status Event Register

    struct system_status sta;
    unsigned char tx_status[SYS_STATUS_LEN] = {0x00};
    unsigned char rx_status[SYS_STATUS_LEN] = {0x00};

    unsigned char rx_buffer_tx[RX_BUFFER_LEN] = {0x00};
    rx_buffer_tx[0] = RX_BUFFER_REG;
    
    while (1) {
      tx_status[0] = SYS_STATUS_REG;
      write_spi_msg(bus, rx_status, tx_status, SYS_STATUS_LEN);
      //Check if there is data 
      //debug_print("0x%02X 0x%02X 0x%02X 0x%02X\n", rx_status[1]
      //  ,rx_status[2], rx_status[3], rx_status[4]);
      if (rx_status[2] & 0x20) {  //Check Data Ready Status
        debug_print("GOT MESSAGE!\n");
        //Clear Status Pin
        clear_status(bus, &sta);
        //Get Message
        struct rx_buffer recv_buff;
        write_spi_msg(bus, &recv_buff, rx_buffer_tx, RX_BUFFER_LEN);
        //Write Message to stdout
        write(STDOUT_FD, recv_buff.payload, PAYLOAD_LEN);
        break;
      }
      nanosleep(&slptime, NULL);
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
    debug_print("0x%02X 0x%02X 0x%02X 0x%02X\n",
           rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
    return rx_buf[1] == 0x30 && rx_buf[2] == 0x01
            && rx_buf[3] == 0xCA && rx_buf[4] == 0xDE;

  }

  int spi_init(struct spi_bus *bus) {
    if ((bus->spi_fd = open(bus->interface_name, O_RDWR)) < 0) {
      perror("Failed to open spidev");
      return 1;
    }
    bus->xfer->speed_hz = 3000000; //3Mhz
    bus->xfer->bits_per_word = 8;
    bus->xfer->delay_usecs = 0;
    bus->xfer->cs_change = 0;
    bus->xfer->pad = 0;
    bus->xfer->tx_nbits = 0;
    bus->xfer->rx_nbits = 0;
    return 0;
  }

  int write_spi_msg(
    struct spi_bus * bus, void * const rx, const void * const tx, int len) {
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
    // Write Check - Unit Test
    char tx_buf[PANADDR_LEN] = {PANADDR_REG, 0x00, 0x00, 0x00, 0x00};
    write_spi_msg(bus, rx_buf, tx_buf, PANADDR_LEN);
    debug_print("0x%02X 0x%02X 0x%02X 0x%02X\n",
           rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
    if ((rx_buf[1] | (rx_buf[2] << 8)) != pan_id) {
      debug_print("Did not set PAN_ID (reading 0x%02X 0x%02X)\n", rx_buf[1], rx_buf[2]);
      return 1;
    }
    if ((rx_buf[3] | (rx_buf[4] << 8)) != addr_id) {
      debug_print("Did not set ADDR_ID (reading 0x%02X 0x%02X)\n", rx_buf[1], rx_buf[2]);
      return 1;
    }

    //Write the tx_fctrl register
    unsigned char rx_fctrl_buf[TX_FCTRL_LEN] = {0x00};
    write_spi_msg(bus, rx_fctrl_buf, fctrl, TX_FCTRL_LEN);
  /*
     char fctrl_buf[sizeof(struct tx_fctrl)];
     memcpy(fctrl_buf, fctrl, sizeof(struct tx_fctrl));
     debug_print("%02X\n", fctrl->tfle);

     debug_print("0x");
     for (unsigned int i = 0; i < sizeof(struct tx_fctrl); i++) {
      debug_print("%02X ", (unsigned)fctrl_buf[i]);
     }
     debug_print("\n");
   */

    return 0;
  }

  void ranging_send(struct spi_bus * bus, struct system_control *ctrl,
    struct system_status *sta, struct tx_buffer *tx_buff, int interrupt_pin) {
    //Get current time, determine initial sending time
    //struct timespec slptime;
    //slptime.tv_sec = 1;
    //slptime.tv_nsec = 0; 
    struct sys_time curr_time = {0x00};
    unsigned char tx_sys_time[SYS_TIME_LEN] = {0x00};
    tx_sys_time[0] = SYS_TIME_REG;
    write_spi_msg(bus, &curr_time, tx_sys_time, SYS_TIME_LEN);
    //uint64_t time = curr_time.curr_time;
    //debug_print("%llu\n", time);
    //nanosleep(&slptime, NULL);
    //}
    uint64_t timestamp_tx_1 = time_add(curr_time.curr_time, T_REPLY);
    //Put timestamp into buffer
    struct dx_time delay_time;  
    delay_time.reg = DX_TIME_REG | WRITE;
    delay_time.delay_time = timestamp_tx_1;
    tx_buff->timestamp_tx = timestamp_tx_1;
    //Write in new buffer
    char buff_rx[TX_BUFFER_LEN];
    write_spi_msg(bus, buff_rx, tx_buff, TX_BUFFER_LEN);
    debug_print("Ok, I will intialize the ranging at time %llu\n", timestamp_tx_1);
    //Send Message
    send_message_delay(bus, ctrl, &delay_time, 1);
    //Go into recieve mode - Wait for Message
    debug_print("Sent message, waiting for reply...\n");
    wait_for_msg_int(bus, ctrl, sta, interrupt_pin);
    //Grab Info
    debug_print("Got message, grabbing data...\n");
    struct rx_data data;
    get_rx_data(bus, &data);
    //Get Timestamp
    uint64_t timestamp_rx_2 = data.timestamp.rx_stamp; //Lower 40 Bits
    //Get TX Timestamp (When it sent it's message)
    uint64_t timestamp_tx_2 = data.buffer.timestamp_tx;
    (void) timestamp_tx_2;
    //Get RX Timestamp (When it got our message)

    //Compute Next Send Time
    uint64_t timestamp_tx_3 = time_add(timestamp_rx_2, T_REPLY);
    //Compose Response
    debug_print("I got the response at %llu, which says the other device"
      " sent its message at %llu, I will reply at %llu\n",
      timestamp_rx_2,
      timestamp_tx_2,
      timestamp_tx_3);
    delay_time.delay_time = timestamp_tx_3;
    tx_buff->timestamp_tx = timestamp_tx_3;
    //Write in new buffer
    write_spi_msg(bus, buff_rx, tx_buff, TX_BUFFER_LEN);
    //Send Message
    send_message_delay(bus, ctrl, &delay_time, 0);
    debug_print("Message will be sent, I am done\n");
    //TODO: Propogation Logic
    uint64_t t_round1 = time_sub(timestamp_rx_2, timestamp_tx_1);
    uint64_t t_prop = (t_round1 - T_REPLY) / 2;
    printf("I compute a tProp of %llu clock cycles\n", t_prop);
    double prop_in_secs = ((double) t_prop) / ((double) CLOCK_FREQ);
    printf("So the distance is %f meters", prop_in_secs * LIGHT_SPEED);
  }

  void ranging_recv(struct spi_bus * bus, struct system_control *ctrl,
    struct system_status *sta, struct tx_buffer *tx_buff, int interrupt_pin) {
   //Go into recieve moce - Wait for Message
   debug_print("Ok, waiting for first message...\n");
   wait_for_msg_int(bus, ctrl, sta, interrupt_pin);
   debug_print("Got First Message, grabbing data...\n");
   //Grab Info
   struct rx_data data;
   get_rx_data(bus, &data);
   //Get Timestamp
   uint64_t timestamp_rx_1 = data.timestamp.rx_stamp; //Lower 40 Bits
   //Get TX Timestamp
   uint64_t timestamp_tx_1 = data.buffer.timestamp_tx;
 //Compute Next Send Time
 uint64_t timestamp_tx_2 = time_add(timestamp_rx_1, T_REPLY);
 debug_print("Ok, got message at timestamp %llu, which was sent at"
 " %llu, will send at %llu\n",
  timestamp_rx_1, timestamp_tx_1, timestamp_rx_2);
 //Compose Response
 struct dx_time delay_time; 
 delay_time.reg = DX_TIME_REG | WRITE;
 delay_time.delay_time = timestamp_tx_2;
 tx_buff->timestamp_tx = timestamp_tx_2;
 //Write in new buffer
 char buff_rx[TX_BUFFER_LEN];
 write_spi_msg(bus, buff_rx, tx_buff, TX_BUFFER_LEN);
 //Send Message
 send_message_delay(bus, ctrl, &delay_time, 1);
 debug_print("Sending packet containing send time %llu"
  " at time %llu, now waiting for response\n",
  timestamp_tx_2,
  timestamp_tx_2);
 //Wait for Response
 wait_for_msg_int(bus, ctrl, sta, interrupt_pin);
 //Grab Info
 debug_print("Got third message, grabbing data...\n");
 struct rx_data data2;
 get_rx_data(bus, &data2);
 //Get Timestamp
 uint64_t timestamp_rx_3 = data2.timestamp.rx_stamp; //Lower 40 Bits
 //Get TX Timestamp
 uint64_t timestamp_tx_3 = data2.buffer.timestamp_tx;
 debug_print("The other device sent it's message at %llu,"
  " I received its message at %llu\n",
  timestamp_tx_3,
  timestamp_rx_3);
 debug_print("I am done\n");
 //TODO: Propogation Logic
 uint64_t timestamp_rx_2 = time_sub(timestamp_tx_3,T_REPLY);
 uint64_t t_round1 = time_sub(timestamp_rx_2, timestamp_tx_1);
 uint64_t t_round2 = time_sub(timestamp_rx_3, timestamp_tx_2);
 printf("The other device will compute tprop of %llu\n", (t_round1 - T_REPLY) / 2);
 uint64_t t_prop = (t_round2 - T_REPLY) / 2;
 printf("I compute a tprop of %llu clock cycles\n",t_prop);
 double prop_in_secs = ((double) t_prop) / ((double) CLOCK_FREQ);
 printf("So the distance is %f meters", prop_in_secs * LIGHT_SPEED);
}

uint64_t time_add(uint64_t curr_time, uint64_t delay_time) {
  //debug_print("The Current time is %llu\n", curr_time);
  //debug_print("We want to delay by %llu\n", delay_time);
  uint64_t sum_time = curr_time + delay_time;
  //Wrap around if we overflow
  if (sum_time > 0xFFFFFFFFFF) {
    uint64_t wrap_time = delay_time - (0xFFFFFFFFFF - sum_time);
    debug_print("OVERFLOW, %llu + %llu wraps to %llu\n", curr_time, delay_time,
      wrap_time);
    return wrap_time;
  } else {
   //debug_print("No overflow, using time as is\n");
   return sum_time;
  }
}
//Computes t_larger - t_smaller
uint64_t time_sub(uint64_t t_larger, uint64_t t_smaller) {
  if (t_smaller > t_larger) //If we wrapped around
    return (0xFFFFFFFFFF - t_smaller) + t_larger;
  else
    return t_larger - t_smaller;
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

int get_rx_data(struct spi_bus *bus, struct rx_data *data) {
  //Initialize structs
  data->finfo.reg = RX_FINFO_REG;
  data->timestamp.reg = RX_TIME_REG;
  data->buffer.reg = RX_BUFFER_REG;
  //Initalize TX Messages
  char tx_finfo[RX_FINFO_LEN] = {0x00};
  tx_finfo[0] = RX_FINFO_REG;
  char tx_timestamp[RX_TIME_LEN] = {0x00};
  tx_timestamp[0] = RX_TIME_REG;
  char tx_buffer[TX_BUFFER_LEN] = {0x00};
  tx_buffer[0] = RX_BUFFER_REG;
  //Write Data Into Structs
  write_spi_msg(bus, &(data->finfo), tx_finfo, RX_FINFO_LEN);
  write_spi_msg(bus, &(data->timestamp), tx_timestamp, RX_TIME_LEN);
  write_spi_msg(bus, &(data->buffer), tx_buffer, RX_BUFFER_LEN);
  return 0;
}

void clear_status(struct spi_bus *bus, struct system_status *sta) {
//Clears all RX/TX System Status Flags
  sta->reg = SYS_STATUS_REG | WRITE;
  sta->irqs = 0;
  sta->cplock = 0;
  sta->esyncr = 0;
  sta->aat = 0;
  sta->txfrb = 1;
  sta->txprs = 1;
  sta->txphs = 1;
  sta->txfrs = 1;
  sta->rxprd = 1;
  sta->rxsfdd = 1;
  sta->ldedone = 0;
  sta->rxphd = 1;
  sta->rxphe = 1;
  sta->rxdfr = 1;
  sta->rxfcg = 1;
  sta->rxfce = 1;
  sta->rxrfsl = 1;
  sta->rxrfto = 1;
  sta->ldeerr = 0;
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
