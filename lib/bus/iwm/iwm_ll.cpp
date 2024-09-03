#ifdef BUILD_APPLE

#ifndef DEV_RELAY_SLIP

#include <string.h>

#include <esp_rom_gpio.h>
#include <soc/spi_periph.h>

#include "iwm_ll.h"
#include "iwm.h"
#include "../device/iwm/disk2.h"
#include "../device/iwm/fuji.h"
#include "fnSystem.h"
#include "fnHardwareTimer.h"
#include "../../include/debug.h"
#include "led.h"

volatile uint8_t _phases = 0;
volatile sp_cmd_state_t sp_command_mode = sp_cmd_state_t::standby;
volatile int isrctr = 0;

#define IWM_NUMBYTES_FOR_BITS(bitcount, buffer) ({			\
      size_t bufbits = sizeof(*(buffer)) * 8;				\
      size_t blen = (bitcount) *					\
	IWM_SAMPLES_PER_CELL(smartport.f_spirx);			\
      blen = (blen + bufbits - 1) / bufbits;				\
      blen;								\
    })

void IRAM_ATTR phi_isr_handler(void *arg)
{
  // handle SP Command Packet or Disk ][ track changes
  // maintain the diskii process:
  // update the head position based on phases
  // put the right track in the SPI buffer

  uint32_t int_gpio_num = (uint32_t) arg; // gpio that triggered the interrupt
  int error; // checksum error return
  uint8_t c;

  _phases = IWM_PHASE_COMBINE();

  //if ((sp_command_mode == sp_cmd_state_t::standby) && (_phases == 0b1011))
  if (_phases == 0b1011)
  {
    switch (sp_command_mode)
    {
    case sp_cmd_state_t::standby:
      error = smartport.iwm_read_packet_spi(IWM.command_packet.data, COMMAND_PACKET_LEN);
      c = IWM.command_packet.command & 0x0f;
      if (!error) // packet received ok and checksum good
      {
        if (c == 0x05)
        {
          smartport.iwm_ack_clr();
          sp_command_mode = sp_cmd_state_t::command;
        }
        else
        {
          for (auto devicep : IWM._daisyChain)
          {
            if (IWM.command_packet.dest == devicep->id())
            {
              smartport.iwm_ack_clr();
              // look for CTRL command
              //  Debug_printf("\nhello from ISR - looking for control command!");

              if ((c == 0x02) ||
                  (c == 0x04) ||
                  (c == 0x09) ||
                  (c == 0x0a) ||
                  (c == 0x0b))
              {
                // Debug_printf("\nhello from ISR - control command!");
                if (smartport.req_wait_for_falling_timeout(5500))
                {
                  Debug_printf("\nWRITE/CTRL received\nREQ timeout in ISR");
                  return;
                }
                memset(smartport.packet_buffer, 0, sizeof(smartport.packet_buffer));
                smartport.iwm_ack_set();
                sp_command_mode = sp_cmd_state_t::rxdata;
                // Debug_printf("\nWRITE/CTRL received\nACK set in ISR!");
              }
              else
              {
                sp_command_mode = sp_cmd_state_t::command;
              }
            }
          }
        }
      }
      else if (error == 2) // checksum error
      {
        Debug_printf("\r\nISR Cmd Chksum error, calc %02x, pkt %02x", smartport.calc_checksum, smartport.pkt_checksum);
      }
      // initial Req timeout (error==1) and checksum (error==2) just fall through here and we try again next time
      smartport.spi_end();
      break;
    case sp_cmd_state_t::rxdata:
      error = smartport.iwm_read_packet_spi(smartport.packet_buffer, BLOCK_PACKET_LEN);
      if (!error) // packet received ok and checksum good
      {
          smartport.iwm_ack_clr();
          sp_command_mode = sp_cmd_state_t::command;
      }
      else if (error == 2) // checksum error
      {
        Debug_printf("\r\nISR Data Packet Chksum error, calc %02x, pkt %02x command = %02x", smartport.calc_checksum, smartport.pkt_checksum,IWM.command_packet.command & 0x0f);
        /*We sometimes get garbage data packets with control code 0 commands, accept them as-is and go on*/
        if((IWM.command_packet.command == 0x84) && (IWM.command_packet.data[19] == 0x80)) {
          Debug_printf("\r\nIgnoring bad data packet");
          smartport.iwm_ack_clr();
          sp_command_mode = sp_cmd_state_t::command;
        }
        // reset sp_command_mode to standy or leave to retry?
      }
      // initial Req timeout (error==1) and checksum (error==2) just fall through here and we try again next time
      smartport.spi_end();
      break;
    case sp_cmd_state_t::command:
      break;
    }
  }
  // add extra condition here to stop edge case where on softsp, the disk is stepping inadvertantly when SP bus is
  // disabled. PH1 gets set low first, then PH3 follows a very short time after. We look for the interrupt on PH1 (33)
  // and then PH1 = 0 (going low) and PH3 = 1 (still high)
  else if ((diskii_xface.iwm_enable_states() & 0b11) && !((int_gpio_num == SP_PHI1 && _phases == 0b1000)))
  {
    if (IWM_ACTIVE_DISK2->move_head())
    {
      isrctr = isrctr + 1;
      IWM_ACTIVE_DISK2->change_track(isrctr);
    }
  }
}

inline void iwm_ll::iwm_extra_set()
{
#ifdef EXTRA
  IWM_BIT_SET(SP_EXTRA);
#endif
}

inline void iwm_ll::iwm_extra_clr()
{
#ifdef EXTRA
  IWM_BIT_CLEAR(SP_EXTRA);
#endif
}

void IRAM_ATTR iwm_sp_ll::encode_spi_packet()
{
  // clear out spi buffer
  memset(spi_buffer, 0, SPI_SP_LEN);
  // loop through "l" bytes of the buffer "packet_buffer"
  uint16_t i=0,j=0;
  while(packet_buffer[i])
  {
    // Debug_printf("\nByte %02X: ",packet_buffer[i]);
    // for each byte, loop through 4 x 2-bit pairs
    uint8_t mask = 0x80;
    for (int k = 0; k < 4; k++)
    {
      if (packet_buffer[i] & mask)
      {
        spi_buffer[j] |= 0x40;
      }
      mask >>= 1;
      if (packet_buffer[i] & mask)
      {
        spi_buffer[j] |= 0x04;
      }
      mask >>= 1;
      // Debug_printf("%02x",spi_buffer[j]);
      j++;
    }
    i++;
  }
  spi_len = --j;
}


int IRAM_ATTR iwm_sp_ll::iwm_send_packet_spi()
{
  //*****************************************************************************
  // Function: iwm_send_packet_spi
  // Parameters: packet_buffer pointer
  // Returns: status (not used yet, always returns 0)
  //
  // Description: This handles the ACK and REQ lines and sends the packet from the
  // pointer passed to it. (packet_buffer)
  //
  //*****************************************************************************

  portDISABLE_INTERRUPTS();
  set_output_to_spi();
  encode_spi_packet();

  // send data stream using SPI
  esp_err_t ret;
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.tx_buffer = spi_buffer; // finally send the line data
  trans.length = spi_len * 8;   // Data length, in bits
  trans.flags = 0;              // undo SPI_TRANS_USE_TXDATA flag

  iwm_ack_set(); // signal ready to send data

  // wait for req line to go high
  if (req_wait_for_rising_timeout(300000))
    {
      // timeout!
      portENABLE_INTERRUPTS(); // takes 7 us to execute
      Debug_printf("\nSendPacket timeout waiting for REQ");
      return 1;
    }

  // send the data
  // TODO - enable / disable output using hasbuffer - no external tristate
  enable_output(); // enable the tri-state buffer
  ret = spi_device_polling_transmit(spi, &trans);
  disable_output(); // make rddata hi-z
  iwm_ack_clr();
  assert(ret == ESP_OK);

  // wait for REQ to go low
  if (req_wait_for_falling_timeout(5000)) // if we don't get REQ low within 500us, then the host didn't like the packet
  {
    portENABLE_INTERRUPTS(); // takes 7 us to execute
    Debug_printf("\nSend REQ timeout");
    req_wait_for_falling_timeout(100000); //wait until host eventually sets REQ low (~1ms), then we can retry send
    return 1;
  }
  portENABLE_INTERRUPTS();
  return 0;
}

#define IWM_NEXT_BIT() ({bool _v = ((src[offset / 8] << (offset % 8)) & 0x80) == 0x80; \
      offset++; _v;})
uint8_t iwm_ll::iwm_decode_byte(uint8_t *src, size_t src_size, unsigned int sample_frequency,
				int timeout, size_t *bit_offset, bool *more_avail)
{
  unsigned int numbits, idx;
  uint8_t byte;
  bool bit, current_level;
  const int spi_samples_per_cell = (CELL_US * sample_frequency) / MHZ;
  const int half_samples = spi_samples_per_cell / 2;
  size_t offset = *bit_offset;
  static bool prev_level = true;

  // ((f_nyquist * f_over) * 18) / (1000 * 1000);
  int timeout_ctr = (sample_frequency * timeout) / MHZ;


  *more_avail = true;
  for (numbits = 8, byte = 0; numbits; numbits--) {
    for (idx = bit = 0; idx < spi_samples_per_cell; idx++) {
      if (offset / 8 >= src_size) {
	// out of spi data, abort
	numbits = 1;
	*more_avail = false;
	break;
      }

      current_level = IWM_NEXT_BIT();

      // loop through 4 usec worth of samples looking for an edge
      // if found, jump forward 2 usec and set bit = 1;
      // otherwise, bit = 0;
      if (prev_level != current_level) {
	bit = true;
	// resync the receiver - must be halfway through 4-us period at an edge
	idx = half_samples;
      }

      prev_level = current_level;
    }

    byte <<= 1;
    byte |= bit;
  }

  // See if there are more 1 bits
  for (; timeout_ctr; timeout_ctr--) {
    if (offset / 8 >= src_size) {
      *more_avail = false;
      break;
    }
    current_level = IWM_NEXT_BIT();
    if (prev_level != current_level)
      break;
  }
  if (!timeout_ctr)
    *more_avail = false;

  *bit_offset = offset;
  return byte;
}

size_t iwm_ll::iwm_decode_buffer(uint8_t *src, size_t src_size, unsigned int sample_frequency,
				 int timeout, uint8_t *dest, size_t *used)
{
  bool more_data = true;
  size_t offset = 0;
  uint8_t *output;


  for (output = dest, offset = 0; more_data; output++)
    *output = iwm_decode_byte(src, src_size, sample_frequency, timeout, &offset, &more_data);

  *used = (offset + 7) / 8;
  return output - dest;
}

int IRAM_ATTR iwm_sp_ll::iwm_read_packet_spi(int n)
{
  return iwm_read_packet_spi(packet_buffer, n);
}

int IRAM_ATTR iwm_sp_ll::iwm_read_packet_spi(uint8_t* buffer, int n)
{ // read data stream using SPI

  // these are for the on the fly checksum decode
  uint8_t checksum = 0;
  uint8_t numodd = 0;
  uint8_t numgrps = 0;
  uint8_t oddbits = 0, evenbits = 0;
  uint16_t grpstart = 14;
  uint16_t group = 0;

  fnTimer.reset();

  // signal the logic analyzer
  iwm_extra_set();

  /* calculations for determining array sizes
  int numsamples = pulsewidth * (n + 2) * 8;
  command packet on DIY SP is 872 us long
  2051282 * 872e-6 = 1798 samples = 224 byes
  nominal command length is 27 bytes * 8 * 8 = 1728 samples
  1798/1728 = 1.04

  command packet on YellowStone (YS) is 919 us
  2.052 * 919 = 1886 samples
  1886 / 1728 = 1.0914    --    this one says we need 10% extra array length

  write block packet on DIY 29 is 20.1 ms long
  2052kHz * 20.1ms =  41245 samples = 5156 bytes
  nominal 604 bytes for block packet = 38656 samples
  41245/38656 = 1.067

  write block packet on YS is 18.95 ms so should fit within DIY
  IIgs take 18.88 ms for a write block

  IIgs GSOS 4 during shutdown sequence is sending something with 20.8 ms duration
  2052 kHz * 20.8 ms = 42681 samples = 5335 bytes

  */

  spi_len = n * pulsewidth * 11 / 10 ; //add 10% for overhead to accomodate YS command packet

  // comment this out, trying to minimise the time from REQ interrupt to start the SPI polling
  // helps the IIgs get in sync to the bitstream quicker
  //memset(spi_buffer, 0xff, SPI_SP_LEN);

  memset(&rxtrans, 0, sizeof(spi_transaction_t));
  rxtrans.flags = 0;
  rxtrans.length = 0; //spi_len * 8;   // Data length, in bits
  rxtrans.rxlength = spi_len * 8;   // Data length, in bits
  rxtrans.tx_buffer = nullptr;
  rxtrans.rx_buffer = spi_buffer; // finally send the line data

  // setup a timeout counter to wait for REQ response
  if (req_wait_for_rising_timeout(10000))
  { // timeout!
#ifdef VERBOSE_IWM
    // timeout
    Debug_print("t");
#endif
    iwm_extra_clr();
    return 1; // timeout waiting for REQ
  }

  esp_err_t ret = spi_device_polling_start(spirx, &rxtrans, portMAX_DELAY);
  assert(ret == ESP_OK);
  iwm_extra_clr();

  // decode the packet here
  size_t spirx_bit_ctr = 0; // initialize the SPI buffer sampler
  bool more_data, have_data = true;
  bool synced = false;
  int idx = 0;             // index into *buffer

  uint8_t rxbyte = 0;      // r23 received byte being built bit by bit

  //for tracking the number of samples
  int last_bit_pos = 0;
  int samples;
  bool start_packet = true;

  fnTimer.latch();               // latch highspeed timer value
  fnTimer.read();                //  grab timer low word
  // sync byte is 10 * 8 * (10*1000*1000/2051282) = 39.0 us long
  fnTimer.alarm_set(390 * 3 / 2);          // wait for first 1.5 sync bytes

  do // have_data
  {
    iwm_extra_set(); // signal to LA we're in the nested loop

    samples = spirx_bit_ctr - last_bit_pos; // difference since last time
    last_bit_pos = spirx_bit_ctr;

    // calc checksum as we go
    // note: idx is pointing to the next byte to be read at this point
    if(idx > 6 && idx < 12) // first part of header
    {
      checksum ^= buffer[idx-1];
    }
    else if (idx == 12) // odd byte count
    {
      numodd = buffer[idx-1] & 0x7f;
      checksum ^= buffer[idx-1];
    }
    else if (idx == 13) //group of 7 bytes count
    {
      numgrps = buffer[idx-1] & 0x7f;
      checksum ^= buffer[idx-1];
    }
    else if ((numodd != 0) && (idx == 14 + numodd)) // calc checksum for odd bytes
    {
      for(int i = 0; i < numodd; i++)
      {
        checksum ^= (((buffer[idx - 1 - numodd]<< (i + 1)) & 0x80) | (buffer[idx - numodd + i] & 0x7f));
      }
      grpstart = 14 + numodd + 1; // update grpstart
    }
    else if ((numgrps != 0) && (group < numgrps) && (idx == grpstart + group * 8 + 7)) // calc checksum for group of 7 bytes
    {
      checksum ^= ((buffer[idx - 8] << (1)) & 0x80) | ((buffer[idx - 7]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (2)) & 0x80) | ((buffer[idx - 6]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (3)) & 0x80) | ((buffer[idx - 5]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (4)) & 0x80) | ((buffer[idx - 4]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (5)) & 0x80) | ((buffer[idx - 3]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (6)) & 0x80) | ((buffer[idx - 2]) & 0x7f);
      checksum ^= ((buffer[idx - 8] << (7)) & 0x80) | ((buffer[idx - 1]) & 0x7f);
      group++;
    }
    else if (idx == 14 + numodd + (numodd != 0) + numgrps * 8 + 1) // decode checksum sent in packet
    {
      evenbits = buffer[idx-2] & 0x55;
      oddbits = (buffer[idx-1] & 0x55) << 1;
    }

    fnTimer.wait();

    if (start_packet) // is at the start, assume sync byte, 39.2 us for 10-bit sync bytes
    {
      fnTimer.alarm_set( 390 ); // latch and read already done in fnTimer.wait()
      start_packet = false;
    }
    else
    {
      fnTimer.alarm_snooze( (samples * 10 * 1000 * 1000) / f_spirx); // samples * 10 /2 ); // snooze the timer based on the previous number of samples
    }

    rxbyte = iwm_decode_byte(spi_buffer, SPI_SP_LEN, f_spirx, 19, &spirx_bit_ctr, &more_data);

    if ((rxbyte == 0xc3) && (!synced))
    {
      synced = true;
      idx = 5;
    }
    buffer[idx++] = rxbyte;
    if (idx > n || !more_data)
      have_data = false;
  } while (have_data); // while have_data

  // keep this so we can print them later for debug
  smartport.calc_checksum = checksum;
  smartport.pkt_checksum = (oddbits | evenbits);

  if (checksum == (oddbits | evenbits))
  {
    return 0; // all good
  }
  else if (((numodd + numgrps * 7) > 0xff && (numodd + numgrps * 7) < 0x200) && (checksum == smartport.last_checksum)) // Liron bug workaround
  {
    return 0; // just assume its ok due to last checksum being the same as this one for the Liron affected size data packets
  }
  else
  {
    smartport.last_checksum = checksum;
    return 2; // checksum error
  }
}

void IRAM_ATTR iwm_sp_ll::spi_end() { spi_device_polling_end(spirx, portMAX_DELAY); };

bool iwm_sp_ll::req_wait_for_falling_timeout(int t)
{
  fnTimer.reset();
  fnTimer.latch();      // latch highspeed timer value
  fnTimer.read();       //  grab timer low word
  fnTimer.alarm_set(t); // t in 100ns units
  while (iwm_req_val())
  {
    fnTimer.latch();       // latch highspeed timer value
    fnTimer.read();        // grab timer low word
    if (fnTimer.timeout()) // test for timeout
    {                      // timeout!
      return true;
    }
  }
  return false;
}

bool iwm_sp_ll::req_wait_for_rising_timeout(int t)
{
  fnTimer.reset();
  fnTimer.latch();      // latch highspeed timer value
  fnTimer.read();       //  grab timer low word
  fnTimer.alarm_set(t); // t in 100ns units
  while (!iwm_req_val())
  {
    fnTimer.latch();       // latch highspeed timer value
    fnTimer.read();        // grab timer low word
    if (fnTimer.timeout()) // test for timeout
    {                      // timeout!
      return true;
    }
  }
  return false;
}

void iwm_sp_ll::setup_spi()
{
  int spirx_mosi_pin = -1;
  esp_err_t ret; // used for calling SPI library functions below

  spi_buffer = (uint8_t *)heap_caps_malloc(SPI_SP_LEN, MALLOC_CAP_DMA);

  if (!fnSystem.spishared())
    spirx_mosi_pin = SP_RDDATA;

  // SPI for receiving packets - sprirx
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = spirx_mosi_pin,
    .miso_io_num = SP_WRDATA,
    .sclk_io_num = -1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .data4_io_num = -1,
    .data5_io_num = -1,
    .data6_io_num = -1,
    .data7_io_num = -1,
    .max_transfer_sz = TRACK_LEN, // SPI_II_LEN,
    .flags = SPICOMMON_BUSFLAG_MASTER,
    .intr_flags = 0
  };

  ret = spi_bus_initialize(VSPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);

  spi_device_interface_config_t rxcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,                      // SPI mode 0
    .duty_cycle_pos = 0,            ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = f_spirx,      // f_over * f_nyquist, // Clock at 500 kHz x oversampling factor
    .input_delay_ns = 0,
    .spics_io_num = -1,             // CS pin
    .flags = SPI_DEVICE_HALFDUPLEX,
    .queue_size = 1,                // We want to be able to queue 7 transactions at a time
    .pre_cb = 0,
    .post_cb = 0,
  };

  ret = spi_bus_add_device(VSPI_HOST, &rxcfg, &spirx);
  assert(ret == ESP_OK);


  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,                   // SPI mode 0
    .duty_cycle_pos = 0,         ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    .cs_ena_pretrans = 0,        ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
    .cs_ena_posttrans = 0,       ///< Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
    .clock_speed_hz = 1 * MHZ, // Clock out at 1 MHz
    .input_delay_ns = 0,
    .spics_io_num = -1,                // CS pin
    .flags = 0,
    .queue_size = 2,                   // We want to be able to queue 7 transactions at a time
    .pre_cb = 0,
    .post_cb = 0,
  };

  if (!fnSystem.spishared())
  {
    // use different SPI than SDCARD
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);
    // connect peripheral to GPIO because RMT screwed it up
    esp_rom_gpio_connect_out_signal(SP_RDDATA, spi_periph_signal[VSPI_HOST].spid_out, false, false);
  }
  else
  {
    // use same SPI as SDCARD
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);
    // connect peripheral to GPIO because RMT screwed it up
    esp_rom_gpio_connect_out_signal(PIN_SD_HOST_MOSI, spi_periph_signal[HSPI_HOST].spid_out, false, false);
  }

  if (smartport.spiMutex == NULL)
  {
    smartport.spiMutex = xSemaphoreCreateMutex();
  }

}

void iwm_ll::setup_gpio()
{
  fnSystem.set_pin_mode(SP_ACK, gpio_mode_t::GPIO_MODE_OUTPUT);
  fnSystem.digital_write(SP_ACK, DIGI_LOW); // set up ACK ahead of time to go LOW when enabled
  //set ack to input to avoid clashing with other devices when sp bus is not enabled
  fnSystem.set_pin_mode(SP_ACK, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_UP);

  if (fnSystem.no3state())
  {
    // set up the output pin as IO (like SPI does) and set to low, then to hi-z
    fnSystem.set_pin_mode(SP_RDDATA, gpio_mode_t::GPIO_MODE_INPUT_OUTPUT);
    fnSystem.digital_write(SP_RDDATA, DIGI_LOW);
    disable_output();
  }

  fnSystem.set_pin_mode(SP_PHI0, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_NONE, gpio_int_type_t::GPIO_INTR_ANYEDGE); // REQ line
  fnSystem.set_pin_mode(SP_PHI1, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_NONE, gpio_int_type_t::GPIO_INTR_ANYEDGE);
  fnSystem.set_pin_mode(SP_PHI2, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_NONE, gpio_int_type_t::GPIO_INTR_ANYEDGE);
  fnSystem.set_pin_mode(SP_PHI3, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_NONE, gpio_int_type_t::GPIO_INTR_ANYEDGE);

  fnSystem.set_pin_mode(SP_WREQ, gpio_mode_t::GPIO_MODE_INPUT,
			SystemManager::pull_updown_t::PULL_UP,
			gpio_int_type_t::GPIO_INTR_ANYEDGE);

  fnSystem.set_pin_mode(SP_DRIVE1, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_DRIVE2, gpio_mode_t::GPIO_MODE_INPUT, SystemManager::pull_updown_t::PULL_UP);
  fnSystem.set_pin_mode(SP_EN35, gpio_mode_t::GPIO_MODE_INPUT);
  fnSystem.set_pin_mode(SP_HDSEL, gpio_mode_t::GPIO_MODE_INPUT);

#ifdef SP_RD_BUFFER
  if (!fnSystem.no3state())
  {
    fnSystem.set_pin_mode(SP_RD_BUFFER, gpio_mode_t::GPIO_MODE_OUTPUT); // tri-state buffer control
    fnSystem.digital_write(SP_RD_BUFFER, DIGI_HIGH); // Turn tristate buffer off by default
  }
#endif

#ifdef EXTRA
  fnSystem.set_pin_mode(SP_EXTRA, gpio_mode_t::GPIO_MODE_OUTPUT);
  fnSystem.digital_write(SP_EXTRA, DIGI_LOW);
  fnSystem.digital_write(SP_EXTRA, DIGI_HIGH); // ID extra for logic analyzer
  fnSystem.digital_write(SP_EXTRA, DIGI_LOW);
  fnSystem.digital_write(SP_EXTRA, DIGI_HIGH); // ID extra for logic analyzer
  fnSystem.digital_write(SP_EXTRA, DIGI_LOW);
  Debug_printf("\nEXTRA signaling line configured");
#endif

#ifdef SP_DEBUG
  fnSystem.set_pin_mode(SP_DEBUG, gpio_mode_t::GPIO_MODE_OUTPUT);
  fnSystem.digital_write(SP_DEBUG, DIGI_LOW);
#endif

  // attach the interrupt service routine
  gpio_isr_handler_add((gpio_num_t)SP_PHI0, phi_isr_handler, (void*) (gpio_num_t)SP_PHI0);
  gpio_isr_handler_add((gpio_num_t)SP_PHI1, phi_isr_handler, (void*) (gpio_num_t)SP_PHI1);
  gpio_isr_handler_add((gpio_num_t)SP_PHI2, phi_isr_handler, (void*) (gpio_num_t)SP_PHI2);
  gpio_isr_handler_add((gpio_num_t)SP_PHI3, phi_isr_handler, (void*) (gpio_num_t)SP_PHI3);
}

void iwm_sp_ll::encode_packet(uint8_t source, iwm_packet_type_t packet_type, uint8_t status, const uint8_t* data, uint16_t num)
{
  // generic version would need:
  // source id
  // packet type
  // status code
  // pointer to the data
  // number of bytes to encode

  uint8_t checksum = 0;
  int numgrps = 0;
  int numodds = 0;

  if ((data != nullptr) && (num != 0))
  {
  int grpbyte, grpcount;
  uint8_t grpmsb;
  uint8_t group_buffer[7];
                                    // Calculate checksum of sector bytes before we destroy them
    for (int count = 0; count < num; count++) // xor all the data bytes
      checksum = checksum ^ data[count];

    // Start assembling the packet at the rear and work
    // your way to the front so we don't overwrite data
    // we haven't encoded yet

    // how many groups of 7?
    numgrps = num / 7;
    numodds = num % 7;

    // grps of 7
    for (grpcount = numgrps - 1; grpcount >= 0; grpcount--) // 73
    {
      memcpy(group_buffer, data + numodds + (grpcount * 7), 7);
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
    if(numodds)
    {
      packet_buffer[14] = 0x80; // init the oddmsb
      for (int oddcnt = 0; oddcnt < numodds; oddcnt++)
      {
        packet_buffer[14] |= (data[oddcnt] & 0x80) >> (1 + oddcnt);
        packet_buffer[15 + oddcnt] = data[oddcnt] | 0x80;
      }
    }
  }

  // header
  packet_buffer[0] = 0xff; // sync bytes
  packet_buffer[1] = 0x3f;
  packet_buffer[2] = 0xcf;
  packet_buffer[3] = 0xf3;
  packet_buffer[4] = 0xfc;
  packet_buffer[5] = 0xff;

  packet_buffer[6] = 0xc3;  //PBEGIN - start byte
  packet_buffer[7] = 0x80;  //DEST - dest id - host
  packet_buffer[8] = source; //SRC - source id - us
  packet_buffer[9] = static_cast<uint8_t>(packet_type);  //TYPE - 0x82 = data
  packet_buffer[10] = 0x80; //AUX
  packet_buffer[11] = status | 0x80; //STAT
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

//*****************************************************************************
// Function: decode_data_packet
// Parameters: none
// Returns: error code, >0 = error encountered
//
// Description: decode arbitrary sized data packet for ctrl/write/write block command from host
// Note: checksum has already been calculated/verified in read packet
//*****************************************************************************
size_t iwm_sp_ll::decode_data_packet(uint8_t* output_data)
{
  return decode_data_packet(packet_buffer, output_data);
}

size_t iwm_sp_ll::decode_data_packet(uint8_t* input_data, uint8_t* output_data)
{
  int grpbyte, grpcount;
  uint8_t numgrps, numodd;
  size_t numdata;
  uint8_t bit0to6, bit7;
  uint8_t group_buffer[8];

  //Handle arbitrary length packets :)
  numodd = input_data[11] & 0x7f;
  numgrps = input_data[12] & 0x7f;
  numdata = numodd + numgrps * 7;
  Debug_printf("\nDecoding %d bytes",numdata);

  // decode oddbyte(s), 1 in a 512 data packet
  for(int i = 0; i < numodd; i++){
    output_data[i] = ((input_data[13] << (i+1)) & 0x80) | (input_data[14+i] & 0x7f);
  }

  // decode groups of 7, 73 grps of 7 in a 512 byte packet
  int grpstart = 12 + numodd + (numodd != 0) + 1;
  for (grpcount = 0; grpcount < numgrps; grpcount++)
  {
    memcpy(group_buffer, input_data + grpstart + (grpcount * 8), 8);
    for (grpbyte = 0; grpbyte < 7; grpbyte++)
    {
      bit7 = (group_buffer[0] << (grpbyte + 1)) & 0x80;
      bit0to6 = (group_buffer[grpbyte + 1]) & 0x7f;
      output_data[numodd + (grpcount * 7) + grpbyte] = bit7 | bit0to6;
    }
  }

  return numdata;
}

void iwm_sp_ll::set_output_to_spi()
{
  if (!fnSystem.spishared())
  {
    esp_rom_gpio_connect_out_signal(SP_RDDATA, spi_periph_signal[VSPI_HOST].spid_out, false, false);
  }
  else
  {
    esp_rom_gpio_connect_out_signal(PIN_SD_HOST_MOSI, spi_periph_signal[HSPI_HOST].spid_out, false, false);
  }
}

void iwm_diskii_ll::set_output_to_low()
{
  if(fnSystem.hasbuffer())
  {
    fnSystem.digital_write(SP_RDDATA, DIGI_LOW);
    esp_rom_gpio_connect_out_signal(SP_RDDATA, SIG_GPIO_OUT_IDX, false, false);
    enable_output();
  }
}


// =========================================================================================
// ========================== DISK II below ======== SP above ==============================
// =========================================================================================

// https://docs.espressif.com/projects/esp-idf/en/v3.3.5/api-reference/peripherals/rmt.html
#define RMT_TX_CHANNEL rmt_channel_t::RMT_CHANNEL_0
#define RMT_USEC (APB_CLK_FREQ / MHZ)

// enable/disable capturing write signal from Disk II
void IRAM_ATTR diskii_write_handler_forwarder(void *arg)
{
  iwm_diskii_ll *d2i = (iwm_diskii_ll *) arg;
  d2i->diskii_write_handler();
  return;
}

size_t iwm_diskii_ll::cspi_current_pos()
{
  uint8_t *spi_rxbuf;
  lldesc_t *current_desc = (lldesc_t *) SPI3.dma_inlink_dscr;


  // Access the current descriptor being used by DMA
  spi_rxbuf = (decltype(spi_rxbuf)) current_desc->buf;

#if 0
  Debug_printf("\r\nDisk II SPI buffer: %x %x offset: %x %x",
	       ptr1, ptr2, d2w_spiaddr - d2w_buffer, d2w_begin);
  strcpy((char *) d2w_spiaddr, "DISKIIMARKER");
#endif
  return spi_rxbuf - d2w_buffer;
}

void IRAM_ATTR iwm_diskii_ll::diskii_write_handler()
{
  bool doCapture = !IWM_BIT(SP_WREQ);


  //Debug_printf("\r\nDisk II write state: %i", doCapture);

  if (doCapture) {
    d2w_begin = track_location;
    d2w_position = cspi_current_pos();
    rx_enabled = true;
    IWM_BIT_SET(SP_DEBUG);

    //IWM_BIT_CLEAR(SP_ACK);
  }
  else if (rx_enabled) {
    IWM_BIT_CLEAR(SP_DEBUG);
    BaseType_t woken;
    iwm_write_data item = {
      .quarter_track = d2w_tracknum,
      .track_begin = d2w_begin,
      .track_end = track_location,
      .track_numbits = track_numbits,
      .buffer = NULL,
      .length = 0,
    };

    {
      size_t offset;

      // Rewind pointer to make sure to get sync bytes
      d2w_position = (d2w_position + d2w_buflen - SPI_CHUNK_SIZE * 2) % d2w_buflen;

      offset = cspi_current_pos();
      item.length = (offset + d2w_buflen - d2w_position) % d2w_buflen;
      //item.length = d2w_buflen;
    }

    item.buffer = (decltype(item.buffer)) heap_caps_malloc(item.length, MALLOC_CAP_8BIT);
    if (!item.buffer)
      Debug_printf("\r\nDisk II unable to allocate buffer! %u %u %u",
		   item.length, item.track_begin, item.track_end);
    else {
      size_t end1, end2;


      end1 = d2w_position + item.length;
      end2 = 0;
      if (end1 >= d2w_buflen) {
	end2 = end1 - d2w_buflen;
	end1 = d2w_buflen;
      }

      end1 -= d2w_position;
      memcpy(item.buffer, &d2w_buffer[d2w_position], end1);
      if (end2)
	memcpy(&item.buffer[end1], d2w_buffer, end2);
      xQueueSendFromISR(iwm_write_queue, &item, &woken);
    }
    rx_enabled = false;
  }

  return;
}

void iwm_diskii_ll::start(uint8_t drive, bool write_protect)
{
  if (write_protect)
    smartport.iwm_ack_set();
  else {
    smartport.iwm_ack_clr();
    gpio_isr_handler_add(SP_WREQ, diskii_write_handler_forwarder, (void *) this);

    SPI3.dma_in_link.addr = (uint32_t) d2w_desc;
    SPI3.dma_inlink_dscr = SPI3.dma_in_link.addr;
    SPI3.dma_conf.dma_continue = 1;
    SPI3.dma_in_link.start = 1;

    // Start SPI receive
    SPI3.cmd.usr = 1;
  }

  diskii_xface.set_output_to_rmt();
  diskii_xface.enable_output();
  ESP_ERROR_CHECK(fnRMT.rmt_write_bitstream(RMT_TX_CHANNEL, track_buffer, track_numbits, track_bit_period));
  fnLedManager.set(LED_BUS, true);
  Debug_printf("\nstart diskII d%d",drive+1);
}

void iwm_diskii_ll::stop()
{
  fnRMT.rmt_tx_stop(RMT_TX_CHANNEL);
  diskii_xface.disable_output();
  SPI3.cmd.usr = 0;
  SPI3.dma_conf.dma_continue = 0;
  //SPI3.dma_in_link.stop = 1;
  SPI3.dma_in_link.start = 0;
  smartport.iwm_ack_set();
  gpio_isr_handler_remove(SP_WREQ);
  fnLedManager.set(LED_BUS, false);
  Debug_printf("\nstop diskII");
}

void iwm_diskii_ll::set_output_to_rmt()
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  if (!fnSystem.spishared())
    esp_rom_gpio_connect_out_signal(SP_RDDATA, rmt_periph_signals.groups[0].channels[0].tx_sig, false, false);
  else
    esp_rom_gpio_connect_out_signal(PIN_SD_HOST_MOSI, rmt_periph_signals.groups[0].channels[0].tx_sig, false, false);
#else
  if (!fnSystem.spishared())
    esp_rom_gpio_connect_out_signal(SP_RDDATA, rmt_periph_signals.channels[0].tx_sig, false, false);
  else
    esp_rom_gpio_connect_out_signal(PIN_SD_HOST_MOSI, rmt_periph_signals.channels[0].tx_sig, false, false);
#endif
}

void iwm_ll::enable_output()
{
  if(fnSystem.no3state())
    IWM_BIT_OUTPUT(SP_RDDATA); // enable output
#ifdef SP_RD_BUFFER
  else
    IWM_BIT_CLEAR(SP_RD_BUFFER); //  enable the tri-state buffer
#endif
}

void iwm_ll::disable_output()
{
  if(fnSystem.no3state())
  {
    GPIO.func_out_sel_cfg[SP_RDDATA].oen_sel = 1;     // let me control the enable register
    IWM_BIT_INPUT(SP_RDDATA); // go hi-z with disabled output
  }
#ifdef SP_RD_BUFFER
  else
    IWM_BIT_SET(SP_RD_BUFFER); // disable the tri-state buffer
#endif
}

void IRAM_ATTR encode_rmt_bitstream_forwarder(const void* src, rmt_item32_t* dest,
					      size_t src_size, size_t wanted_num,
					      size_t* translated_size, size_t* item_num,
					      int bit_period)
{
  diskii_xface.encode_rmt_bitstream(src, dest, src_size, wanted_num, translated_size,
				    item_num, bit_period);
  return;
}

//Convert track data to rmt format data.
void IRAM_ATTR iwm_diskii_ll::encode_rmt_bitstream(const void* src, rmt_item32_t* dest,
						   size_t src_size, size_t wanted_num,
						   size_t* translated_size, size_t* item_num,
						   int bit_period)
{
    // *src is equal to *track_buffer
    // src_size is equal to numbits
    // translated_size is not used
    // item_num will equal wanted_num at end

    if (src == NULL || dest == NULL)
    {
      *translated_size = 0;
      *item_num = 0;
      return;
    }

    // TODO: allow adjustment of bit timing per WOZ optimal bit timing
    //
    uint32_t bit_ticks = RMT_USEC; // ticks per microsecond (1000 ns)
    bit_ticks *= bit_period; // now units are ticks * ns /us
    bit_ticks /= 1000; // now units are ticks

#define BIT_TICK_34 ((uint16_t) ((3 * bit_ticks) / 4))
#define BIT_TICK_14 ((uint16_t) (bit_ticks / 4))

    const rmt_symbol_word_t bits[] = {
      {{ BIT_TICK_34, 0, BIT_TICK_14, 0 }},
      {{ BIT_TICK_34, 0, BIT_TICK_14, 1 }},
    };
    static uint8_t window = 0;
    uint8_t outbit = 0;
    size_t num = 0;
    uint8_t *trk_buf = (uint8_t *) src;

    for (num = 0; num < wanted_num; num++, dest++)
      {
	// MC34780 behavior for random bit insertion
	// https://applesaucefdc.com/woz/reference2/

	int track_byte_ctr = track_location / 8;
	int track_bit_ctr = track_location % 8;

	// bits go MSB first
	outbit = (trk_buf[track_byte_ctr] & (0x80 >> track_bit_ctr)) != 0;
	track_location = (track_location + 1) % src_size;

	window <<= 1;
	window |= outbit;
	window &= 0x0f;
	if (window != 0)
	  outbit = window & 0x02;
	else
	  {
	    const uint8_t MC3470[] = {0b01010000, 0b10110011, 0b01000010, 0b00000000, 0b10101101, 0b00000010, 0b01101000, 0b01000110, 0b00000001, 0b10010000, 0b00001000, 0b00111000, 0b00001000, 0b00100101, 0b10000100, 0b00001000, 0b10001000, 0b01100010, 0b10101000, 0b01101000, 0b10010000, 0b00100100, 0b00001011, 0b00110010, 0b11100000, 0b01000001, 0b10001010, 0b00000000, 0b11000001, 0b10001000, 0b10001000, 0b00000000};

	    static int MC3470_byte_ctr;
	    static int MC3470_bit_ctr;

	    ++MC3470_bit_ctr %= 8;
	    if (MC3470_bit_ctr == 0)
	      ++MC3470_byte_ctr %= sizeof(MC3470);

	    outbit = (MC3470[MC3470_byte_ctr] & (0x01 << MC3470_bit_ctr)) != 0;
	  }

	dest->val = bits[!!outbit].val;
      }
    *translated_size = wanted_num;
    *item_num = wanted_num;
}

/*
 * Initialize the RMT Tx channel and SPI Rx channel
 */

void iwm_diskii_ll::setup_rmt()
{
  // SPI continuous
  {
    uint32_t num_chunks, idx;
    lldesc_t *desc_ptr;


    d2w_buflen = IWM_NUMBYTES_FOR_BITS(TRACK_LEN * 8, d2w_buffer);
    /* Make sure it's an even multiple of SPI_CHUNK_SIZE */
    num_chunks = (d2w_buflen + SPI_CHUNK_SIZE - 1) / SPI_CHUNK_SIZE;
    d2w_buflen = num_chunks * SPI_CHUNK_SIZE;
    d2w_buffer = (decltype(d2w_buffer)) heap_caps_malloc(d2w_buflen, MALLOC_CAP_DMA);
    iwm_write_queue = xQueueCreate(10, sizeof(iwm_write_data));
    memset(d2w_buffer, 0xff, d2w_buflen);

    d2w_desc = (lldesc_t *) heap_caps_malloc(sizeof(lldesc_t) * num_chunks, MALLOC_CAP_DMA);
    if (!d2w_desc) {
      ESP_LOGE("SPI_DMA", "Failed to allocate DMA descriptor");
      return;
    }

    memset(d2w_desc, 0, sizeof(lldesc_t) * num_chunks);
    for (idx = 0; idx < num_chunks; idx++) {
      desc_ptr = &d2w_desc[idx];
      desc_ptr->size = SPI_CHUNK_SIZE;
      desc_ptr->owner = 1; // Owned by DMA hardware
      desc_ptr->buf = &d2w_buffer[idx * SPI_CHUNK_SIZE];
      desc_ptr->qe.stqe_next = &d2w_desc[(idx + 1) % num_chunks];
    }
  }

  track_buffer = (uint8_t *)heap_caps_malloc(TRACK_LEN, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (track_buffer == NULL)
    Debug_println("could not allocate track buffer");

  config.rmt_mode = rmt_mode_t::RMT_MODE_TX;
  config.channel = RMT_TX_CHANNEL;
#ifdef RMTTEST
  config.gpio_num = (gpio_num_t)SP_EXTRA;
#else
  if (!fnSystem.spishared())
    config.gpio_num = (gpio_num_t)SP_RDDATA; //SP_WRDATA; // SP_RDDATA ; //PIN_SD_HOST_MOSI;
  else
    config.gpio_num = (gpio_num_t)PIN_SD_HOST_MOSI; //SP_WRDATA; // SP_RDDATA ; //PIN_SD_HOST_MOSI;
#endif
  config.mem_block_num = 8;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW;
  config.clk_div = 1; // use full 80 MHz resolution of APB clock

  ESP_ERROR_CHECK(fnRMT.rmt_config(&config));
  ESP_ERROR_CHECK(fnRMT.rmt_driver_install(config.channel, 0, ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(fnRMT.rmt_translator_init(config.channel, encode_rmt_bitstream_forwarder));
}

void IRAM_ATTR iwm_diskii_ll::copy_track(uint8_t *track, size_t tracklen, size_t trackbits,
					 int bitperiod, int tracknum)
{
  d2w_tracknum = tracknum;
  // copy track from SPIRAM to INTERNAL RAM
  if (track != nullptr)
    memcpy(track_buffer, track, tracklen);
  else
    memset(track_buffer, 0, tracklen);

  // new_position = current_position * new_track_length / current_track_length
  track_location *= trackbits;
  track_location /= track_numbits;
  // update track info
  track_numbytes = tracklen;
  track_numbits = trackbits;
  track_bit_period = bitperiod;
}

uint8_t IRAM_ATTR iwm_diskii_ll::iwm_enable_states()
{
  uint8_t states = 0;

  // only enable diskII if we are either not on an en35 capable host, or we are on an en35host and /EN35=high
  if (!IWM.en35Host || (IWM.en35Host && IWM_BIT(SP_EN35)))
  {
    if (!(states |= IWM_BIT(SP_DRIVE1) ? 0b00 : 0b01))
    {
      states |= IWM_BIT(SP_DRIVE2) ? 0b00 : 0b10;
    }
  }

  // Check if Drive 2 is being accessed but disabled
  if ((states == 0x02) && !isDrive2Enabled()) {
    // Drive is disabled, return 0
    return 0;
  }

  return states;
}

iwm_sp_ll smartport;
iwm_diskii_ll diskii_xface;

#endif // !DEV_RELAY_SLIP

#endif // BUILD_APPLE
