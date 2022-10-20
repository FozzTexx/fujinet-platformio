#ifdef BUILD_APPLE
#include "iwm.h"
#include "fnSystem.h"
#include "fnHardwareTimer.h"
// #include "fnFsTNFS.h" // do i need this?
#include <string.h>
// #include "driver/timer.h" // contains the hardware timer register data structure
#include "../../include/debug.h"
#include "utils.h"
#include "led.h"

#include "../device/iwm/disk.h"
#include "../device/iwm/fuji.h"
#include "../device/iwm/cpm.h"

/******************************************************************************
Based on:
Apple //c Smartport Compact Flash adapter
Written by Robert Justice  email: rjustice(at)internode.on.net
Ported to Arduino UNO with SD Card adapter by Andrea Ottaviani email: andrea.ottaviani.69(at)gmail.com
SD FAT support added by Katherine Stark at https://gitlab.com/nyankat/smartportsd/
 *****************************************************************************
 * Written for FujiNet ESP32 by @jeffpiep 
 * search for "todo" to find things to work on
*/

/* pin assignments for Arduino UNO 
from  http://www.users.on.net/~rjustice/SmartportCFA/SmartportSD.htm
IDC20 Disk II 20-pin pins based on
https://www.bigmessowires.com/2015/04/09/more-fun-with-apple-iigs-disks/
*/

// only used when looking for tricky things - will crash after first packet most likely
#undef VERBOSE_IWM
// #define VERBOSE_IWM


//------------------------------------------------------------------------------

//*****************************************************************************
// Function: print_packet
// Parameters: pointer to data, number of bytes to be printed
// Returns: none
//
// Description: prints packet data for debug purposes to the serial port
//*****************************************************************************
void print_packet (uint8_t* data, int bytes)
{
  int row;
  char tbs[8];
  char xx;

  Debug_printf(("\r\n"));
  for (int count = 0; count < bytes; count = count + 16) 
  {
    sprintf(tbs, ("%04X: "), count);
    Debug_print(tbs);
    for (row = 0; row < 16; row++) {
      if (count + row >= bytes)
        Debug_print(("   "));
      else {
        Debug_printf("%02x ",data[count + row]);
      }
    }
    Debug_print(("-"));
    for (row = 0; row < 16; row++) {
      if ((data[count + row] > 31) && (count + row < bytes) && (data[count + row] < 128))
      {
        xx = data[count + row];
        Debug_print(xx);
      }
      else
      {
        Debug_print(("."));
      }
    }
    Debug_printf(("\r\n"));
  }
}

void print_packet(uint8_t* data)
{
  Debug_printf("\r\n");
  for (int i = 0; i < 40; i++)
  {
    if (data[i]!=0 || i==0)
      Debug_printf("%02x ", data[i]);
    else
      break;
  }
  // Debug_printf("\r\n");
}

void print_packet_wave(uint8_t* data, int bytes)
{
  int row;
  char tbs[8];

  Debug_printf(("\r\n"));
  for (int count = 0; count < bytes; count = count + 12) 
  {
    sprintf(tbs, ("%04X: "), count);
    Debug_print(tbs);
    for (row = 0; row < 12; row++) {
      if (count + row >= bytes)
        Debug_print(("         "));
      else {
        uint8_t b = data[count + row];
        for (int bnum=0; bnum<8; bnum++)
        {
          if (b & 0x80)
          {
            Debug_print("#");
          }
          else
          {
            Debug_print("_");
          }
          b <<= 1;
        }
        Debug_print(".");
         }
    }
    Debug_printf(("\r\n"));
  }
}

//------------------------------------------------------------------------------

uint8_t iwmDevice::packet_buffer[BLOCK_PACKET_LEN] = { 0 };
uint16_t iwmDevice::packet_len = 0;
uint16_t iwmDevice::num_decoded = 0;



void iwmBus::iwm_ack_deassert()
{
  smartport.iwm_ack_set(); // go hi-z
}

void iwmBus::iwm_ack_assert()
{
  smartport.iwm_ack_clr();
  smartport.spi_end();
}

bool iwmBus::iwm_phase_val(uint8_t p)
{
  uint8_t phases = smartport.iwm_phase_vector();
  if (p < 4)
    return (phases >> p) & 0x01;
  Debug_printf("\r\nphase number out of range");
  return false;
}

iwmBus::iwm_phases_t iwmBus::iwm_phases()
{ 
  iwm_phases_t phasestate = iwm_phases_t::idle;
  // phase lines for smartport bus reset
  // ph3=0 ph2=1 ph1=0 ph0=1
  // phase lines for smartport bus enable
  // ph3=1 ph2=x ph1=1 ph0=x
  uint8_t phases = smartport.iwm_phase_vector();
  if (phases == 0b1010)
    phasestate = iwm_phases_t::enable;
  else if (phases == 0b0101)
    phasestate = iwm_phases_t::reset;

#ifdef VERBOSE_IWM
  if (phasestate != oldphase)
  {
    //Debug_printf("\r\n%d%d%d%d",iwm_phase_val(0),iwm_phase_val(1),iwm_phase_val(2),iwm_phase_val(3));
    switch (phasestate)
    {
    case iwm_phases_t::idle:
      Debug_printf("\r\nidle");
      break;
    case iwm_phases_t::reset:
      Debug_printf("\r\nreset");
      break;
    case iwm_phases_t::enable:
      Debug_printf("\r\nenable");
    }
    oldphase=phasestate;
  }
#endif

  return phasestate;
}

//------------------------------------------------------

int iwmBus::iwm_send_packet(uint8_t *a)
{
  return smartport.iwm_send_packet_spi(a);
}

int iwmBus::iwm_read_packet_timeout(int attempts, uint8_t *a, int n)
{
  portDISABLE_INTERRUPTS();
  iwm_ack_deassert();
  for (int i = 0; i < attempts; i++)
  {
    if (!smartport.iwm_read_packet_spi(a, n))
    {
      iwm_ack_assert();
      portENABLE_INTERRUPTS();
#ifdef DEBUG
      print_packet(a);
#endif
      return 0;
    } // if
  }
#ifdef DEBUG
  Debug_printf("\r\nERROR: Read Packet tries exceeds %d attempts", attempts);
  print_packet(a);
#endif
  portENABLE_INTERRUPTS();
  return 1;
}


void iwmBus::setup(void)
{
  Debug_printf(("\r\nIWM FujiNet based on SmartportSD v1.15\r\n"));

  fnTimer.config();
  Debug_printf("\r\nIWM timer started");

  smartport.setup();
  Debug_printf("\r\nSPI configured for smartport I/O");

  fnSystem.set_pin_mode(SP_ACK, gpio_mode_t::GPIO_MODE_OUTPUT);
  fnSystem.digital_write(SP_ACK, DIGI_LOW); // set up ACK ahead of time to go LOW when enabled
  //set ack to input to avoid clashing with other devices when sp bus is not enabled
  fnSystem.set_pin_mode(SP_ACK, gpio_mode_t::GPIO_MODE_INPUT);
  
  fnSystem.set_pin_mode(SP_PHI0, gpio_mode_t::GPIO_MODE_INPUT); // REQ line
  fnSystem.set_pin_mode(SP_PHI1, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_PHI2, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_PHI3, gpio_mode_t::GPIO_MODE_INPUT);

  // fnSystem.set_pin_mode(SP_WRDATA, gpio_mode_t::GPIO_MODE_INPUT); // not needed cause set in SPI?

  fnSystem.set_pin_mode(SP_WREQ, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_DRIVE1, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_DRIVE2, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_EN35, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_HDSEL, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_RDDATA, gpio_mode_t::GPIO_MODE_OUTPUT); // tri-state buffer control
  fnSystem.digital_write(SP_RDDATA, DIGI_HIGH); // Turn tristate buffer off by default

  Debug_printf("\r\nIWM GPIO configured");
}

//*****************************************************************************
// Function: encode_data_packet
// Parameters: source id
// Returns: none
//
// Description: encode 512 byte data packet for read block command from host
// requires the data to be in the packet buffer, and builds the smartport
// packet IN PLACE in the packet buffer
//*****************************************************************************
void iwmDevice::encode_data_packet(uint16_t num) 
{
  int grpbyte, grpcount;
  uint8_t checksum = 0, grpmsb;
  uint8_t group_buffer[7];

  // Calculate checksum of sector bytes before we destroy them
  for (int count = 0; count < num; count++) // xor all the data bytes
    checksum = checksum ^ packet_buffer[count];

  // Start assembling the packet at the rear and work 
  // your way to the front so we don't overwrite data
  // we haven't encoded yet

  // how many groups of 7?
  uint8_t numgrps = num / 7;
  uint8_t numodds = num % 7;

  //grps of 7
  for (grpcount = numgrps; grpcount >= 0; grpcount--) //73
  {
    memcpy(group_buffer, packet_buffer + numodds + (grpcount * 7), 7);
    // add group msb byte
    grpmsb = 0;
    for (grpbyte = 0; grpbyte < 7; grpbyte++)
      grpmsb = grpmsb | ((group_buffer[grpbyte] >> (grpbyte + 1)) & (0x80 >> (grpbyte + 1)));
    // groups start after odd bytes, which is at 13 + numodds + (numodds != 0) + 1
    int grpstart = 13 + numodds + (numodds != 0) + 1;
    packet_buffer[grpstart + (grpcount * 8)] = grpmsb | 0x80; // set msb to one

    // now add the group data bytes bits 6-0
    for (grpbyte = 0; grpbyte < 7; grpbyte++)
      packet_buffer[grpstart + 1 + (grpcount * 8) + grpbyte] = group_buffer[grpbyte] | 0x80;

  }
  
  // oddbytes
  packet_buffer[14] = 0x80; // init the oddmsb
  for (int oddcnt = 0; oddcnt < numodds; oddcnt++)
  {
    packet_buffer[14] |= (packet_buffer[oddcnt] & 0x80) >> (1 + oddcnt);
    packet_buffer[15 + oddcnt] = packet_buffer[oddcnt] | 0x80;
  }

  // header
  packet_buffer[0] = 0xff;  //sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = id(); //SRC - source id - us
  packet_buffer[9] = 0x82;  //TYPE - 0x82 = data
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = 0x80; //STAT
  packet_buffer[12] = numodds | 0x80; //ODDCNT  - 1 odd byte for 512 byte packet
  packet_buffer[13] = numgrps | 0x80; //GRP7CNT - 73 groups of 7 bytes for 512 byte packet

  for (int count = 7; count < 14; count++) // now xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];
  int lastidx = 14 + numodds + (numodds != 0) + numgrps * 8;
  packet_buffer[lastidx++] = checksum | 0xaa;      // 1 c6 1 c4 1 c2 1 c0
  packet_buffer[lastidx++] = (checksum >> 1) | 0xaa; // 1 c7 1 c5 1 c3 1 c1

  //end bytes
  packet_buffer[lastidx++] = 0xc8;  //pkt end
  packet_buffer[lastidx] = 0x00;  //mark the end of the packet_buffer
}

// void iwmDevice::encode_data_packet() // to do overload with packet size for read?
// {
//   encode_data_packet(512);
// }

//*****************************************************************************
// Function: encode_extended_data_packet
// Parameters: source id
// Returns: none
//
// Description: encode 512 byte data packet for read block command from host
// requires the data to be in the packet buffer, and builds the smartport
// packet IN PLACE in the packet buffer
//*****************************************************************************
void iwmDevice::encode_extended_data_packet (uint8_t source)
{
  int grpbyte, grpcount;
  uint8_t checksum = 0, grpmsb;
  uint8_t group_buffer[7];

  // Calculate checksum of sector bytes before we destroy them
  for (int count = 0; count < 512; count++) // xor all the data bytes
    checksum = checksum ^ packet_buffer[count];

  // Start assembling the packet at the rear and work 
  // your way to the front so we don't overwrite data
  // we haven't encoded yet

  //grps of 7
  for (grpcount = 72; grpcount >= 0; grpcount--) //73
  {
    memcpy(group_buffer, packet_buffer + 1 + (grpcount * 7), 7);
    // add group msb byte
    grpmsb = 0;
    for (grpbyte = 0; grpbyte < 7; grpbyte++)
      grpmsb = grpmsb | ((group_buffer[grpbyte] >> (grpbyte + 1)) & (0x80 >> (grpbyte + 1)));
    packet_buffer[16 + (grpcount * 8)] = grpmsb | 0x80; // set msb to one

    // now add the group data bytes bits 6-0
    for (grpbyte = 0; grpbyte < 7; grpbyte++)
      packet_buffer[17 + (grpcount * 8) + grpbyte] = group_buffer[grpbyte] | 0x80;

  }
  
  //total number of packet data bytes for 512 data bytes is 584
  //odd byte
  packet_buffer[14] = ((packet_buffer[0] >> 1) & 0x40) | 0x80;
  packet_buffer[15] = packet_buffer[0] | 0x80;

  packet_buffer[0] = 0xff;  //sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = source; //SRC - source id - us
  packet_buffer[9] = 0xC2;  //TYPE - 0xC2 = extended data
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = 0x80; //STAT
  packet_buffer[12] = 0x81; //ODDCNT  - 1 odd byte for 512 byte packet
  packet_buffer[13] = 0xC9; //GRP7CNT - 73 groups of 7 bytes for 512 byte packet

  for (int count = 7; count < 14; count++) // now xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];

  packet_buffer[600] = checksum | 0xaa;      // 1 c6 1 c4 1 c2 1 c0
  packet_buffer[601] = checksum >> 1 | 0xaa; // 1 c7 1 c5 1 c3 1 c1

  //end bytes
  packet_buffer[602] = 0xc8;  //pkt end
  packet_buffer[603] = 0x00;  //mark the end of the packet_buffer

}



//*****************************************************************************
// Function: decode_data_packet
// Parameters: none
// Returns: error code, >0 = error encountered
//
// Description: decode 512 (arbitrary now) byte data packet for write block command from host
// decodes the data from the packet_buffer IN-PLACE!
//*****************************************************************************
bool iwmDevice::decode_data_packet(void)
{
  int grpbyte, grpcount;
  uint8_t numgrps, numodd;
  uint16_t numdata;
  uint8_t checksum = 0, bit0to6, bit7, oddbits, evenbits;
  uint8_t group_buffer[8];

  //Handle arbitrary length packets :) 
  numodd = packet_buffer[11] & 0x7f;
  numgrps = packet_buffer[12] & 0x7f;
  numdata = numodd + numgrps * 7;
  Debug_printf("\r\nDecoding %d bytes",numdata);
  // if (numdata==512)
  // {
  //   // print out packets
  //   print_packet(packet_buffer,BLOCK_PACKET_LEN);
  // }
  // First, checksum  packet header, because we're about to destroy it
  for (int count = 6; count < 13; count++) // now xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];

  int chkidx = 13 + numodd + (numodd != 0) + numgrps * 8;
  evenbits = packet_buffer[chkidx] & 0x55;
  oddbits = (packet_buffer[chkidx + 1] & 0x55) << 1;

  //add oddbyte(s), 1 in a 512 data packet
  for(int i = 0; i < numodd; i++){
    packet_buffer[i] = ((packet_buffer[13] << (i+1)) & 0x80) | (packet_buffer[14+i] & 0x7f);
  }

  // 73 grps of 7 in a 512 byte packet
  int grpstart = 12 + numodd + (numodd != 0) + 1;
  for (grpcount = 0; grpcount < numgrps; grpcount++)
  {
    memcpy(group_buffer, packet_buffer + grpstart + (grpcount * 8), 8);
    for (grpbyte = 0; grpbyte < 7; grpbyte++) {
      bit7 = (group_buffer[0] << (grpbyte + 1)) & 0x80;
      bit0to6 = (group_buffer[grpbyte + 1]) & 0x7f;
      packet_buffer[numodd + (grpcount * 7) + grpbyte] = bit7 | bit0to6;
    }
  }

  //verify checksum
  for (int count = 0; count < numdata; count++) // xor all the data bytes
    checksum = checksum ^ packet_buffer[count];

  Debug_printf("\r\ndecode data packet checksum calc %02x, packet %02x", checksum, (oddbits | evenbits));

  if (checksum != (oddbits | evenbits))
  {
    Debug_printf("\r\nCHECKSUM ERROR!");
    return true; // error!
  }
  
  num_decoded = numdata;
  return false;
}

//*****************************************************************************
// Function: encode_write_status_packet
// Parameters: source,status
// Returns: none
//
// Description: this is the reply to the write block data packet. The reply
// indicates the status of the write block cmd.
//*****************************************************************************
void iwmDevice::encode_write_status_packet(uint8_t source, uint8_t status)
{
  uint8_t checksum = 0;

  packet_buffer[0] = 0xff;  //sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  //  int i;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = source; //SRC - source id - us
  packet_buffer[9] = PACKET_TYPE_STATUS;  //TYPE
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = status | 0x80; //STAT
  packet_buffer[12] = 0x80; //ODDCNT
  packet_buffer[13] = 0x80; //GRP7CNT

  for (int count = 7; count < 14; count++) // xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];
  packet_buffer[14] = checksum | 0xaa;      // 1 c6 1 c4 1 c2 1 c0
  packet_buffer[15] = checksum >> 1 | 0xaa; // 1 c7 1 c5 1 c3 1 c1

  packet_buffer[16] = 0xc8;  //pkt end
  packet_buffer[17] = 0x00;  //mark the end of the packet_buffer

}

//*****************************************************************************
// Function: encode_init_reply_packet
// Parameters: source
// Returns: none
//
// Description: this is the reply to the init command packet. A reply indicates
// the original dest id has a device on the bus. If the STAT byte is 0, (0x80)
// then this is not the last device in the chain. This is written to support up
// to 4 partions, i.e. devices, so we need to specify when we are doing the last
// init reply.
//*****************************************************************************
void iwmDevice::encode_init_reply_packet (uint8_t source, uint8_t status)
{
  uint8_t checksum = 0;

  packet_buffer[0] = 0xff;  //sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = source; //SRC - source id - us
  packet_buffer[9] = 0x80;  //TYPE
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = status | 0x80; //STAT - data status

  packet_buffer[12] = 0x80; //ODDCNT
  packet_buffer[13] = 0x80; //GRP7CNT

  for (int count = 7; count < 14; count++) // xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];
  packet_buffer[14] = checksum | 0xaa;      // 1 c6 1 c4 1 c2 1 c0
  packet_buffer[15] = checksum >> 1 | 0xaa; // 1 c7 1 c5 1 c3 1 c1

  packet_buffer[16] = 0xc8; //PEND
  packet_buffer[17] = 0x00; //end of packet in buffer

}

void iwmDevice::encode_error_reply_packet (uint8_t stat)
{
  uint8_t checksum = 0;

  packet_buffer[0] = 0xff;  //sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = id(); //SRC - source id - us
  packet_buffer[9] = PACKET_TYPE_STATUS;  //TYPE -status
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = stat | 0x80; //STAT - data status - error
  packet_buffer[12] = 0x80; //ODDCNT - 0 data bytes
  packet_buffer[13] = 0x80; //GRP7CNT

  for (int count = 7; count < 14; count++) // xor the packet header bytes
    checksum = checksum ^ packet_buffer[count];
  packet_buffer[14] = checksum | 0xaa;      // 1 c6 1 c4 1 c2 1 c0
  packet_buffer[15] = checksum >> 1 | 0xaa; // 1 c7 1 c5 1 c3 1 c1

  packet_buffer[16] = 0xc8; //PEND
  packet_buffer[17] = 0x00; //end of packet in buffer
}

void iwmDevice::iwm_return_badcmd(cmdPacket_t cmd)
{
  Debug_printf("\r\nUnit %02x Bad Command %02x", id(), cmd.command);
  encode_error_reply_packet(SP_ERR_BADCMD);
  IWM.iwm_send_packet((unsigned char *)packet_buffer);
}

void iwmDevice::iwm_return_ioerror(cmdPacket_t cmd)
{
  Debug_printf("\r\nUnit %02x Bad Command %02x", id(), cmd.command);
  encode_error_reply_packet(SP_ERR_IOERROR);
  IWM.iwm_send_packet((unsigned char *)packet_buffer);
}

//*****************************************************************************
// Function: verify_cmdpkt_checksum
// Parameters: none
// Returns: 0 = ok, 1 = error
//
// Description: verify the checksum for command packets
//
// &&&&&&&&not used at the moment, no error checking for checksum for cmd packet
//*****************************************************************************
bool iwmBus::verify_cmdpkt_checksum(void)
{
  //int length;
  uint8_t evenbits, oddbits, bit7, bit0to6, grpbyte;
  uint8_t calc_checksum = 0; //initial value is 0
  uint8_t pkt_checksum;

  //length = get_packet_length();
  //Debug_printf("\r\npacket length = %d", length);
  //2 oddbytes in cmd packet
  // calc_checksum ^= ((packet_buffer[13] << 1) & 0x80) | (packet_buffer[14] & 0x7f);
  // calc_checksum ^= ((packet_buffer[13] << 2) & 0x80) | (packet_buffer[15] & 0x7f);
  calc_checksum ^= ((command_packet.oddmsb << 1) & 0x80) | (command_packet.command & 0x7f);
  calc_checksum ^= ((command_packet.oddmsb << 2) & 0x80) | (command_packet.parmcnt & 0x7f);

  // 1 group of 7 in a cmd packet
  for (grpbyte = 0; grpbyte < 7; grpbyte++) {
    bit7 = (command_packet.grp7msb << (grpbyte + 1)) & 0x80;
    bit0to6 = (command_packet.data[17 + grpbyte]) & 0x7f;
    calc_checksum ^= bit7 | bit0to6;
  }

  // calculate checksum for overhead bytes
  for (int count = 6; count < 13; count++) // start from first id byte
    calc_checksum ^= command_packet.data[count];

  // int chkidx = 13 + numodd + (numodd != 0) + numgrps * 8;
  // evenbits = packet_buffer[chkidx] & 0x55;
  // oddbits = (packet_buffer[chkidx + 1] & 0x55) << 1;
  oddbits = (command_packet.chksum2 << 1) | 0x01;
  evenbits = command_packet.chksum1;
  pkt_checksum = oddbits & evenbits; // oddbits | evenbits;
  // every other bit is ==1 in checksum, so need to AND to get data back

  //  Debug_print(("Pkt Chksum Byte:\r\n"));
  //  Debug_print(pkt_checksum,DEC);
  //  Debug_print(("Calc Chksum Byte:\r\n"));
  //  Debug_print(calc_checksum,DEC);
  //  Debug_printf("\r\nChecksum - pkt,calc: %02x %02x", pkt_checksum, calc_checksum);
  // if ( pkt_checksum == calc_checksum )
  //   return false;
  // else
  //   return true;
  return (pkt_checksum != calc_checksum);  
}

void iwmDevice::iwm_status(cmdPacket_t cmd) // override;
{
  uint8_t status_code = cmd.g7byte3 & 0x7f; // (packet_buffer[19] & 0x7f); // | (((unsigned short)packet_buffer[16] << 3) & 0x80);
  Debug_printf("\r\nTarget Device: %02x", cmd.dest);
  // add a switch case statement for ALL THE STATUSESESESESS
  if (status_code == 0x03)
  { // if statcode=3, then status with device info block
    Debug_printf("\r\n******** Sending DIB! ********");
    encode_status_dib_reply_packet();
    // print_packet ((unsigned char*) packet_buffer,get_packet_length());
    fnSystem.delay(50);
    }
    else
    { // else just return device status
      Debug_printf("\r\nSending Status");
      encode_status_reply_packet();
    }
  print_packet(&packet_buffer[14]);
  IWM.iwm_send_packet((unsigned char *)packet_buffer);
}

//*****************************************************************************
// Function: packet_length
// Parameters: none
// Returns: length
//
// Description: Calculates the length of the packet in the packet_buffer.
// A zero marks the end of the packet data.
//*****************************************************************************
int iwmDevice::get_packet_length (void)
{
  int x = 5; // start at the 0xc3 beginning of packet
  while (packet_buffer[x++]);
  return x - 1; // point to last packet byte = C8
}


//*****************************************************************************
// Function: main loop
/*
 * notes:
 * with individual devices, like disk.cpp,
 * we need to hand off control to the device to service
 * the command packet. 
 * 
 * Disk II/3.5 selection is determined by the ENABLE lines
 * from BMOW - https://www.bigmessowires.com/2015/04/09/more-fun-with-apple-iigs-disks/
 * On an Apple II, things are more complicated. The Apple 5.25 controller card was the first to use a DB19 connector, and it supported two daisy-chained 5.25 inch drives. Pin 17 is /DRIVE1 enable, and pin 9 (unconnected on the Macintosh) is /DRIVE2 enable. Within each drive, internal circuitry routes the signal from input pin 9 to output pin 17 on the daisy-chain connector. Drive #2 doesn’t actually know that it’s drive #2 – it enables itself by observing /DRIVE1 on pin 17, just like the first drive – only the first drive has sneakily rerouted /DRIVE2 to /DRIVE1. This allows for two drives to be daisy chained.
 * On an Apple IIgs, it’s even more complicated. Its DB19 connector supports daisy-chaining two 3.5 inch drives, and two 5.25 inch drives – as well as even more SmartPort drives, which I won’t discuss now. Pin 4 (GND on the Macintosh) is /EN3.5, a new signal that enables the 3.5 inch drives when it’s low, or the 5.25 inch drives when it’s high. The 3.5 inch drives must appear before any 5.25 inch drives in the daisy chain. When /EN3.5 is low, the 3.5 inch drives use pins 17 and 9 to enable themselves, and when /EN3.5 is high, the 3.5 inch drives pass through the signals on pins 17 and 9 unmodified to the 5.25 drives behind them.
 * This is getting complicated, but there’s one final kick in the nuts: when the first 3.5 drive is enabled, by the IIgs setting /EN3.5 and /DRIVE1 both low, you would think the drive would disable the next 3.5 drive behind it by setting both /DRIVE1 and /DRIVE2 high at the daisy-chain connector. But no, the first 3.5 drive disables the second 3.5 drive by setting both /DRIVE1 and /DRIVE2 low! This looks like both are enabled at the same time, which would be a definite no-no, but the Apple 3.5 Drive contains circuitry that recognizes this “double enable” as being equivalent to a disable. Why it’s done this way, I don’t know, but I’m sure it has some purpose.
 * 
 * So for starters FN will look at the /DRIVEx line (not sure which one because IIc has internal floppy drive connected right now)
 * If floppy is enabled, the motor is spinning and FN needs to track the phases and spit out data (unless writereq is activated)
 * If floppy is disabled, smartport should be in control instead.
 * 
 * The smartport algorithm is something like:
 * check for 0x85 init and do a bus initialization:
 * BUS INIT
 * after a reset, all devices no longer have an address
 * and they are gating some signal (REQ?) so devices
 * down the chain cannot respond to commands. So the
 * first device responds to INIT. During this, it checks
 * the sense line (still not sure which pin this is) to see
 * if it is low (grounded) or high (floating or pulled up?).
 * It will be low if there's another device in the chain
 * after it. If it is the last device it will be high.
 * It sends this state in the response to INIT. It also
 * ungates whatever the magic line is so the next device
 * in the chain can receive the INIT command that is
 * coming next. This repeats until the last device in the
 * chain says it's so and the A2 will stop sending INITs.
 *
 * Every other command:
 * The bus class checks the target device and should pass
 * on the command packet to the device service routine.
 * Then the device can respond accordingly.
 *
 * When device ID is not FujiNet's:
 * If the device ID does not belong to any of the FujiNet
 * devices (disks, printers, modem, network, etc) then FN
 * should not respond. The SmartPortSD code runs through
 * the states for the packets that should come next. I'm
 * not sure this is the best because what happens in case
 * of a malfunction. I suppose there could be a time out
 * that takes us back to idle. This will take more
 * investigation.
 */
//*****************************************************************************
void iwmBus::service()
{
  iwm_ack_deassert(); // go hi-Z

  // if (iwm_drive_enables())
  // {
  //   //Debug_printf("\r\nFloppy Drive ENabled!");
  //   iwm_rddata_clr();
  // }
  // else
  // {
  //   //Debug_printf("\r\nFloppy Drive DISabled!"); // debug msg latency here screws up SP timing.
  //    iwm_rddata_set(); // make rddata hi-z
  // }

  // read phase lines to check for smartport reset or enable
  switch (iwm_phases())
  {
  case iwm_phases_t::idle:
    break;
  case iwm_phases_t::reset:
    // instead of the code in this section, we should call a reset handler
    // the handler should reset every device
    // and wait for reset to clear (probably with a timeout)
    Debug_printf(("\r\nReset"));
    // hard coding 1 partition - will use disk class instances instead
    // smort->_devnum = 0;
    for (auto devicep : _daisyChain)
      devicep->_devnum = 0;

    while (iwm_phases() == iwm_phases_t::reset)
      ; // no timeout needed because the IWM must eventually clear reset.
    // even if it doesn't, we would just come back to here, so might as
    // well wait until reset clears.

    Debug_printf(("\r\nReset Cleared"));
    break;
  case iwm_phases_t::enable:
    // expect a command packet
    portDISABLE_INTERRUPTS();
    if(smartport.iwm_read_packet_spi(command_packet.data, COMMAND_PACKET_LEN))
    {
      portENABLE_INTERRUPTS();
      return;
    }
    // should not ACK unless we know this is our Command
    if (command_packet.command == 0x85)
    {
      iwm_ack_assert(); // includes waiting for spi read transaction to finish
      portENABLE_INTERRUPTS();

      // wait for REQ to go low
      if (smartport.req_wait_for_falling_timeout(50000))
        return;


#ifdef DEBUG
      print_packet(command_packet.data);
      Debug_printf("\r\nhandling init command");
#endif
      if (verify_cmdpkt_checksum())
      {
        Debug_printf("\r\nBAD CHECKSUM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Debug_printf("\r\ndo init anyway");
      }      // to do - checksum verification? How to respond?
      handle_init();
    }
    else
    {
      // smort->process(command_packet);
      for (auto devicep : _daisyChain)
      {
        if (command_packet.dest == devicep->_devnum)
        {
          iwm_ack_assert(); // includes waiting for spi read transaction to finish
          portENABLE_INTERRUPTS();
          // wait for REQ to go low
          if (smartport.req_wait_for_falling_timeout(50000))
            return;

          // need to take time here to service other ESP processes so they can catch up
          taskYIELD(); // Allow other tasks to run
          print_packet(command_packet.data);
          
          _activeDev = devicep;
          // handle command
          if (verify_cmdpkt_checksum())
          {
            Debug_printf("\r\nBAD CHECKSUM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            _activeDev->iwm_return_ioerror(command_packet);
          }
          else
          {
            _activeDev->process(command_packet);
          }
        }
      }
    }
  } // switch (phasestate)
}

bool iwmBus::iwm_drive_enables()
{
  return false; // ignore floppy drives for now
  //return !iwm_enable_val();
}

void iwmBus::handle_init()
{
  uint8_t status = 0;
  iwmDevice* pDevice = nullptr;

  fnLedManager.set(LED_BUS, true);

  // iwm_rddata_clr();
  
  // to do - get the next device in the daisy chain and assign ID
  for (auto it = _daisyChain.begin(); it != _daisyChain.end(); ++it)
  {
    // tell the Fuji it's device no.
    if (it == _daisyChain.begin())
    {
      theFuji._devnum = command_packet.dest;
    }
    // assign dev numbers
    pDevice = (*it);
    if (pDevice->id() == 0)
    {
      pDevice->_devnum = command_packet.dest; // assign address
      if (++it == _daisyChain.end())
        status = 0xff; // end of the line, so status=non zero - to do: check GPIO for another device in the physical daisy chain
      pDevice->encode_init_reply_packet(command_packet.dest, status);
      Debug_printf("\r\nSending INIT Response Packet...");
      smartport.iwm_send_packet_spi((uint8_t *)pDevice->packet_buffer); // timeout error return is not handled here (yet?)

      // print_packet ((uint8_t*) packet_buffer,get_packet_length());

      Debug_printf(("\r\nDrive: %02x\r\n"), pDevice->id());
      fnLedManager.set(LED_BUS, false);
      return;
    }
  }

  fnLedManager.set(LED_BUS, false);

}

// Add device to SIO bus
void iwmBus::addDevice(iwmDevice *pDevice, iwm_fujinet_type_t deviceType)
{
  // SmartPort interface assigns device numbers to the devices in the daisy chain one at a time
  // as opposed to using standard or fixed device ID's like Atari SIO. Therefore, an emulated
  // device cannot rely on knowing its device number until it is assigned.
  // Instead of using device_id's to know what kind a specific device is, smartport 
  // uses a Device Information Block (DIB) that is returned in a status call for DIB. The 
  // DIB includes a 16-character string, Device type byte, and Device subtype byte.
  // In the IIgs firmware reference, the following device types are defined:
  // 0 - memory cards (internal to the machine)
  // 1 - Apple and Uni 3.5 drives
  // 2 - harddisk
  // 3 - SCSI disk
  // The subtype uses the 3 msb's to indicate the following:
  // 0x80 == 1 -> support extended smartport
  // 0x40 == 1 -> supprts disk-switched errors
  // 0x20 == 0 -> removable media (1 means non removable)

  // todo: work out how to use addDevice
  // we can add devices and indicate they are not initialized and have no device ID - call it a value of 0
  // when the SP bus goes into RESET, we would rip through the list setting initialized to false and
  // setting device id's to 0. Then on each INIT command, we iterate through the list, setting 
  // initialized to true and assigning device numbers as assigned by the smartport controller in the A2.
  // so I need "reset()" and "initialize()" functions.

  // todo: I need a way to internally keep track of what kind of device each one is. I'm thinking an
  // enumerated class type might work well here. It can be expanded as needed and an extra case added 
  // below. I can also make this a switch case structure to ensure each case of the class is handled.

    // assign dedicated pointers to certain devices
    switch (deviceType)
    {
    case iwm_fujinet_type_t::BlockDisk:
      break;
    case iwm_fujinet_type_t::FujiNet:
      _fujiDev = (iwmFuji *)pDevice;
      break;
    case iwm_fujinet_type_t::Modem:
      _modemDev = (iwmModem *)pDevice;
      break;
    case iwm_fujinet_type_t::Network:
      // todo: work out how to assign different network devices - idea:
      // include a number in the DIB name, e.g., "NETWORK 1"
      // and extract that number from the DIB and use it as the index
      //_netDev[device_id - SIO_DEVICEID_FN_NETWORK] = (iwmNetwork *)pDevice;
      break;
    case iwm_fujinet_type_t::CPM:
       _cpmDev = (iwmCPM *)pDevice;
       break;
    case iwm_fujinet_type_t::Printer:
      _printerdev = (iwmPrinter *)pDevice;
      break;
    case iwm_fujinet_type_t::Voice:
    // not yet implemented: todo - take SAM and implement as a special block device. Also then available for disk rotate annunciation. 
      break;
    case iwm_fujinet_type_t::Other:
      break;
    }

    pDevice->_devnum = 0;
    pDevice->_initialized = false;

    _daisyChain.push_front(pDevice);
}

// Removes device from the SIO bus.
// Note that the destructor is called on the device!
void iwmBus::remDevice(iwmDevice *p)
{
    _daisyChain.remove(p);
}

// Should avoid using this as it requires counting through the list
int iwmBus::numDevices()
{
    int i = 0;
    __BEGIN_IGNORE_UNUSEDVARS
    for (auto devicep : _daisyChain)
        i++;
    return i;
    __END_IGNORE_UNUSEDVARS
}

void iwmBus::changeDeviceId(iwmDevice *p, int device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep == p)
            devicep->_devnum = device_id;
    }
}

iwmDevice *iwmBus::deviceById(int device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep->_devnum == device_id)
            return devicep;
    }
    return nullptr;
}

void iwmBus::enableDevice(uint8_t device_id)
{
    iwmDevice *p = deviceById(device_id);
    p->device_active = true;
}

void iwmBus::disableDevice(uint8_t device_id)
{
    iwmDevice *p = deviceById(device_id);
    p->device_active = false;
}

// Give devices an opportunity to clean up before a reboot
void iwmBus::shutdown()
{
    for (auto devicep : _daisyChain)
    {
        Debug_printf("Shutting down device %02x\n",devicep->id());
        devicep->shutdown();
    }
    Debug_printf("All devices shut down.\n");
}

iwmBus IWM; // global smartport bus variable

#endif /* BUILD_APPLE */
