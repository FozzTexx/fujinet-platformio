#ifdef BUILD_IEC

#include "iec_ll.h"
#include "fnSystem.h"
#include "../../../include/pinmap.h"

#include "../../../include/debug.h"

#include <soc/gpio_struct.h>
#include <string.h>

// To make it easier to port
#define IEC_ATN		PIN_IEC_ATN
#define IEC_CLK		PIN_IEC_CLK_IN
#define IEC_DATA	PIN_IEC_DATA_IN

#define DATA_EOI	0x100
#define DATA_ATN	0x200

enum {
  IECWaitState = 1,
  IECAttentionState,
  IECAttentionIgnoreState,
  IECListenState,
  IECInputState,
  IECTalkState,
  IECOutputState,
};

#define IEC_ASSERTED  true
#define IEC_RELEASED  false

#define IEC_RELEASE(pin) ({			\
      uint32_t _pin = pin;			\
      uint32_t _mask = 1 << (_pin % 32);	\
      if (_pin >= 32)				\
	GPIO.enable1_w1tc.val = _mask;		\
      else					\
	GPIO.enable_w1tc = _mask;		\
    })
#define IEC_ASSERT(pin) ({			\
      uint32_t _pin = pin;			\
      uint32_t _mask = 1 << (_pin % 32);	\
      if (_pin >= 32)				\
	GPIO.enable1_w1ts.val = _mask;		\
      else					\
	GPIO.enable_w1ts = _mask;		\
    })

#define IEC_IS_SET(pin) ({						\
      uint32_t _pin = pin;						\
      (_pin >= 32 ? GPIO.in1.val : GPIO.in) & (1 << (_pin % 32));	\
    })

#ifndef IEC_INVERTED_LINES
#define IEC_IS_ASSERTED(pin) (!IEC_IS_SET(pin))
#else
#define IEC_IS_ASSERTED(pin) (IEC_IS_SET(pin) ? IEC_ASSERTED : IEC_RELEASED)
#endif /* !IEC_INVERTED_LINES */

void IECLowLevel::handleATN()
{
  int atn;


  atn = IEC_IS_ASSERTED(IEC_ATN);
  if (atn) {
    iec_atnState = IECAttentionState;
    iec_state = IECWaitState;
    IEC_RELEASE(IEC_CLK);
    IEC_ASSERT(IEC_DATA);
  }
  else
    iec_atnState = IECWaitState;

  return;
}

void morse_code(int val)
{
  int idx;


  for (idx = 0; idx < 8; idx++, val <<= 1) {
    usleep(1);
    IEC_ASSERT(PIN_IEC_SRQ);
    usleep(2 + 4 * !!(val & 0x80));
    IEC_RELEASE(PIN_IEC_SRQ);
  }
  usleep(10);

  return;
}

void IECLowLevel::handleCLK()
{
  int atn, val;
  int cmd, dev;


  gpio_intr_disable(IEC_CLK);
  atn = IEC_IS_ASSERTED(IEC_ATN);
  if (!atn)
    iec_atnState = IECWaitState;

  if (iec_atnState != IECAttentionState &&
      (iec_state == IECWaitState || iec_state == IECOutputState || iec_state == IECTalkState)) {
    morse_code('S');
    morse_code(iec_state);
    goto done;
  }

  if (atn && iec_atnState == IECAttentionIgnoreState) {
    morse_code('A');
    morse_code(iec_atnState);
    goto done;
  }

  /* FIXME - if buffer is full waiting for userspace to read, we need to block! */
  /* FIXME - if iec_readAvail is set then buffer is being sent */

  IEC_ASSERT(PIN_IEC_SRQ);//Debug
  val = readByte();
  if (val >= 0) {
    if (atn) {
      val |= DATA_ATN;

      /* Partially processing commands that may need to release bus
	 here because of timing issues. */
      cmd = val & 0xe0;
      dev = val & 0x1f;
      if ((cmd == IECListenCommand || cmd == IECTalkCommand) &&
	  (dev == IEC_ALLDEV || !iec_openDevices[dev])) {
	iec_atnState = IECAttentionIgnoreState;
	if (dev == IEC_ALLDEV)
	  iec_state = IECWaitState;
	usleep(c64slowdown);
	releaseBus();
      }
    }

    iec_buffer[iec_inpos] = val;
    iec_inpos = (iec_inpos + 1) % IEC_BUFSIZE;
  }

 done:
  gpio_intr_enable(IEC_CLK);
  IEC_RELEASE(PIN_IEC_SRQ);//Debug
  return;
}

int IECLowLevel::waitForSignals(int pin1, int state1, int pin2, int state2, int delay)
{
  uint64_t start, now;
  int elapsed, abort = 0;


  start = esp_timer_get_time();
  for (;;) {
    if (IEC_IS_ASSERTED(pin1) == state1)
      break;
    if (pin2 && IEC_IS_ASSERTED(pin2) == state2)
      break;

    now = esp_timer_get_time();
    elapsed = now - start;
    if (elapsed >= delay) {
      abort = 1;
      break;
    }
  }

  return abort;
}

int IECLowLevel::readByte()
{
  int eoi, abort;
  int len, bits;
  uint64_t start, now;
  int elapsed = 0;


  IEC_RELEASE(IEC_DATA);

  start = esp_timer_get_time();
  for (abort = eoi = 0; !abort && !IEC_IS_ASSERTED(IEC_CLK); ) {
    now = esp_timer_get_time();
    elapsed = now - start;

    if (!eoi && elapsed >= 200) {
      IEC_ASSERT(IEC_DATA);
      usleep(c64slowdown*2);
      IEC_RELEASE(IEC_DATA);
      eoi = DATA_EOI;
    }

    if (elapsed > 100000) {
      abort = 1;
      break;
    }
  }

  if (elapsed < 60) {
    len = elapsed - c64slowdown;
    if (len > 0 && c64slowdown / 10 < len && len > 5) {
      c64slowdown = elapsed;
    }
  }

  for (len = 0, bits = eoi; !abort && len < 8; len++) {
    if ((abort = waitForSignals(IEC_CLK, IEC_RELEASED, 0, 0, 150))) {
      break;
    }

    if (IEC_IS_SET(IEC_DATA))
      bits |= 1 << len;

    if (waitForSignals(IEC_CLK, IEC_ASSERTED, 0, 0, 150)) {
      if (len < 7)
	abort = 1;
    }
  }

  IEC_ASSERT(IEC_DATA);

  usleep(c64slowdown);

  if (abort)
    return -1;

  return bits;
}

int IECLowLevel::writeByte(int bits)
{
  int len;
  int abort = 0;


  if (IEC_IS_ASSERTED(IEC_ATN) || iec_state != IECOutputState) {
    return 1;
  }

  IEC_RELEASE(IEC_CLK);

  if ((abort = waitForSignals(IEC_DATA, IEC_RELEASED, IEC_ATN, IEC_ASSERTED, 100000))) {
    //printk(KERN_NOTICE "IEC: Timeout waiting to send\n");
  }

  /* Because interrupts are disabled it's possible to miss the ATN pause signal */
  if (IEC_IS_ASSERTED(IEC_ATN)) {
    iec_state = IECWaitState;
    iec_atnState = IECAttentionState;
    abort = 1;
  }

  if (!abort && (bits & DATA_EOI)) {
    if ((abort = waitForSignals(IEC_DATA, IEC_ASSERTED, IEC_ATN, IEC_ASSERTED, 100000))) {
      //printk(KERN_NOTICE "IEC: Timeout waiting for EOI ack\n");
    }

    if (!abort &&
	(abort = waitForSignals(IEC_DATA, IEC_RELEASED, IEC_ATN, IEC_ASSERTED, 100000))) {
      //printk(KERN_NOTICE "IEC: Timeout waiting for EOI ack finish\n");
    }
  }

  IEC_ASSERT(IEC_CLK);
  usleep(c64slowdown);

  for (len = 0; !abort && len < 8; len++, bits >>= 1) {
    if (IEC_IS_ASSERTED(IEC_ATN) || iec_state != IECOutputState) {
      //printk(KERN_NOTICE "IEC: attention during write\n");
      abort = 1;
      break;
    }
    if (bits & 1)
      IEC_RELEASE(IEC_DATA);
    else
      IEC_ASSERT(IEC_DATA);

    usleep(c64slowdown*2);
    IEC_RELEASE(IEC_CLK);
    usleep(c64slowdown*2);
    IEC_RELEASE(IEC_DATA);
    IEC_ASSERT(IEC_CLK);
  }

  if (!abort &&
      (abort = waitForSignals(IEC_DATA, IEC_ASSERTED, IEC_ATN, IEC_ASSERTED, 10000))) {
    if (!IEC_IS_ASSERTED(IEC_ATN)) {
      abort = 0;
    }
    else {
      iec_state = IECWaitState;
      iec_atnState = IECAttentionState;
    }
  }

  usleep(c64slowdown);

  if (abort && IEC_IS_ASSERTED(IEC_ATN))
    IEC_RELEASE(IEC_CLK);

  return abort;
}

void IECLowLevel::releaseBus()
{
  IEC_RELEASE(IEC_CLK);
  IEC_RELEASE(IEC_DATA);
  usleep(c64slowdown);
  return;
}

void IECLowLevel::sendInput(void)
{
  iec_device *device;
  iec_chain *chain;


  if (!(device = iec_openDevices[iec_curDevice]))
    return;

  /* FIXME - if we are holding the bus because of this device, release the bus */

  chain = &device->in;
  if (!chain->cur)
    return;

  chain->cur = NULL;
  iec_readAvail = 1;

  // Send things off to high level CBM DOS emulator
  BaseType_t woken;
  xSemaphoreGiveFromISR(iec_canRead, &woken);

  return;
}

void IECLowLevel::newIO(int val)
{
  int dev;
  iec_device *device;
  iec_chain *chain;
  iec_io *io;
  static unsigned char serial = 0;


  dev = val & 0x1f;
  device = iec_openDevices[dev];
  if (!device)
    return;

  chain = &device->in;

#if 0
  /* FIXME - this shouldn't happen */
  if (chain->cur)
    printk(KERN_NOTICE "IEC: new IO with IO still open\n");
#endif

  io = (iec_io *) malloc(sizeof(iec_io));
  io->header.command = val & 0xff;
  io->header.channel = 0;
  io->header.len = 0;
  io->header.eoi = 0;
  io->header.serial = serial++;
  io->next = NULL;
  io->outpos = 0;

  if (!chain->head)
    chain->head = chain->tail = io;
  else {
    chain->tail->next = io;
    chain->tail = io;
  }
  chain->cur = io;

  return;
}

void IECLowLevel::channelIO(int val)
{
  int chan;
  iec_device *device;
  iec_chain *chain;


  chan = val & 0x0f;

  if (!(device = iec_openDevices[iec_curDevice]))
    return;
  iec_curChannel = chan;
  chain = &device->in;
  if (!chain->cur)
    return;

  chain->cur->header.channel = val & 0xff;

  return;
}

int IECLowLevel::unlinkIO(iec_device *device)
{
  iec_chain *chain;
  iec_io *io;


  if (!xSemaphoreTake(device->lock, portMAX_DELAY))
    return 0;

  chain = &device->in;
  io = chain->head;
  chain->head = io->next;
  free(io);
  xSemaphoreGive(device->lock);
  return 1;
}

int IECLowLevel::setupTalker(void)
{
  int abort = 0;


  abort = waitForSignals(IEC_ATN, IEC_RELEASED, 0, 0, 1000000);
  if (!abort)
    abort = waitForSignals(IEC_CLK, IEC_RELEASED, IEC_ATN, IEC_ASSERTED, 100000);

  if (!abort) {
    IEC_RELEASE(IEC_DATA);
    IEC_ASSERT(IEC_CLK);
    usleep(c64slowdown);
    iec_state = IECOutputState;
  }

  return abort;
}

void IECLowLevel::appendByte(int val)
{
  iec_device *device;
  iec_chain *chain;
  iec_io *io;


  if (!(device = iec_openDevices[iec_curDevice]))
    return;

  chain = &device->in;
  if (!(io = chain->cur))
    return;

  val = val & 0xff;
  io->data[io->header.len] = val;
  io->header.len++;

  return;
}

void IECLowLevel::processData()
{
  int avail;
  int val, atn;
  int cmd, dev;


  for (;;) {
    avail = (IEC_BUFSIZE + iec_inpos - iec_outpos) % IEC_BUFSIZE;
    if (!avail)
      return;

    val = iec_buffer[iec_outpos];
    atn = val & DATA_ATN;
    iec_outpos = (iec_outpos + 1) % IEC_BUFSIZE;

    if (atn) {
      cmd = val & 0xe0;
      dev = val & 0x1f;

      switch (cmd) {
      case IECListenCommand:
	if (dev == IEC_ALLDEV || !iec_openDevices[dev])
	  sendInput();
	else {
	  iec_state = IECListenState;
	  newIO(val);
	  iec_curDevice = dev;
	}
	break;

      case IECTalkCommand:
	if (dev == IEC_ALLDEV || !iec_openDevices[dev])
	  sendInput();
	else {
	  iec_state = IECTalkState;
	  newIO(val);
	  iec_curDevice = dev;
	}
	break;

      case IECReopenCommand:
	channelIO(val);
	/* Take a break driver 8. We can reach our destination, but we're still a ways away */
	if (iec_state == IECTalkState) {
	  setupTalker();
	  sendInput();
	}
	break;

      case IECFileCommand:
	channelIO(val);
	if (dev == 0x00)
	  sendInput();
	break;

      default:
	break;
      }
    }
    else {
      iec_device *device;


      appendByte(val);
      device = iec_openDevices[iec_curDevice];
      if (!device || !device->in.cur)
	return;
      if (val & DATA_EOI)
	device->in.cur->header.eoi = 1;
      if (device->in.cur->header.len == sizeof(device->in.cur->data))
	sendInput();
    }
  }

  return;
}

void IECLowLevel::configPin(gpio_num_t pin, gpio_int_type_t intr_type, isr_handler *handler)
{
  gpio_config_t conf = {
    .pin_bit_mask = 1ULL << pin,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = intr_type,
  };

  gpio_config(&conf);
  gpio_set_level(pin, 0);

  if (intr_type != GPIO_INTR_DISABLE)
    gpio_isr_handler_add(pin, isr_forwarder, handler);

  return;
}

/* Public methods */

bool IECLowLevel::init()
{
  iec_canRead = xSemaphoreCreateBinary();

  iec_inpos = iec_outpos = 0;
  atn_isr.handler = this;
  atn_isr.method = &IECLowLevel::handleATN;
  clk_isr.handler = this;
  clk_isr.method = &IECLowLevel::handleCLK;

  configPin(PIN_IEC_ATN, GPIO_INTR_NEGEDGE, &atn_isr);
  configPin(PIN_IEC_CLK_IN, GPIO_INTR_POSEDGE, &clk_isr);
  configPin(PIN_IEC_DATA_IN, GPIO_INTR_DISABLE, nullptr);
  configPin(PIN_IEC_SRQ, GPIO_INTR_DISABLE, nullptr);
#ifdef IEC_HAS_RESET
  configPin(PIN_IEC_RESET, GPIO_INTR_DISABLE, nullptr);
#endif

  if (!(iec_buffer = (uint16_t *) malloc(IEC_BUFSIZE * sizeof(uint16_t))))
    return false;

  return true;
}

int IECLowLevel::open(int devnum)
{
  iec_device *device;


  device = iec_openDevices[devnum];

  /* FIXME - allow more than one process to open at once */
  if (device)
    return -1;

  device = iec_openDevices[devnum] = (iec_device *) malloc(sizeof(iec_device));
  device->in.head = device->in.tail = device->in.cur = NULL;
  device->out.outpos = 0;
  device->lock = xSemaphoreCreateBinary();
  xSemaphoreGive(device->lock);
  iec_deviceCount++;

  return 0;
}

int IECLowLevel::close(int devnum)
{
  cleanupDevice(devnum);
  iec_deviceCount--;
  return 0;
}

void IECLowLevel::cleanupDevice(int devnum)
{
  iec_device *device;
  iec_io *io, *ioNext;


  if ((device = iec_openDevices[devnum])) {
    if (!xSemaphoreTake(iec_canRead, portMAX_DELAY)) {
      Debug_printv("IEC: unable to cleanup device %i\n", devnum);
      return;
    }

    iec_openDevices[devnum] = NULL;
    xSemaphoreGive(device->lock);
    vSemaphoreDelete(device->lock);
    device->lock = NULL;
    io = device->in.head;
    while (io) {
      ioNext = io->next;
      free(io);
      io = ioNext;
    }

    free(device);
  }

  return;
}

size_t IECLowLevel::hasData(int devnum)
{
  int avail = 0;
  iec_device *device = iec_openDevices[devnum];
  iec_io *io;


  processData();

  io = device->in.head;
  if (io && io != device->in.cur)
    avail = (io->header.len + sizeof(io->header)) - io->outpos;

  return avail;
}

size_t IECLowLevel::read(int devnum, uint8_t *buf, size_t count)
{
  int avail = 0;
  iec_device *device = iec_openDevices[devnum];
  iec_io *io;
  unsigned char *wbuf;


  processData();

  do {
    io = device->in.head;
    if (io && io != device->in.cur)
      avail = (io->header.len + sizeof(io->header)) - io->outpos;
    if (!avail && !xSemaphoreTake(iec_canRead, portMAX_DELAY)) {
      Debug_printv("Failed to get iec_canRead");
      return 0;
    }
    iec_readAvail = 0;
  } while (!avail);

  if (!xSemaphoreTake(device->lock, portMAX_DELAY)) {
    Debug_printv("IEC: unable to lock IO");
    return 0;
  }

  if (io->outpos < sizeof(io->header)) {
    wbuf = (unsigned char *) &io->header;
    wbuf += io->outpos;
    if (count > sizeof(io->header) - io->outpos)
      count = sizeof(io->header) - io->outpos;
  }
  else {
    int pos;


    pos = io->outpos - sizeof(io->header);
    if (count > io->header.len - pos)
      count = io->header.len - pos;
    wbuf = &io->data[pos];
  }

  memcpy(buf, wbuf, count);
  xSemaphoreGive(device->lock);

  io->outpos += count;

  if (io->outpos == io->header.len + sizeof(io->header) && !unlinkIO(device))
    return -1;

  return count;
}

size_t IECLowLevel::write(int devnum, const uint8_t *buf, size_t count)
{
  iec_device *device = iec_openDevices[devnum];
  iec_io *io;
  unsigned char *wbuf;
  size_t len, offset;
  int abort, val;
  char data[2];


  /* FIXME - really only care about EOI unless minor is 0 and acting as master */
  /* FIXME - how to detect EOI? */

  offset = 0;
  abort = 0;
  while (!abort && count > 0) {
    len = count;
    io = &device->out;

    if (io->outpos < sizeof(io->header)) {
      wbuf = (unsigned char *) &io->header;
      wbuf += io->outpos;
      if (len > sizeof(io->header) - io->outpos)
	len = sizeof(io->header) - io->outpos;

      memcpy(wbuf, buf+offset, len);
      io->outpos += len;

      if (io->outpos == sizeof(io->header) && !io->header.len) {
	iec_state = IECWaitState;
	releaseBus();
      }
    }
    else {
      if (iec_state != IECOutputState) {
	len = 0;
	abort = 1;
      }
      else {
	len = 1;
	memcpy(data, buf+offset, len);
	if (len) {
	  val = data[0];
	  if (io->header.eoi && io->outpos + 1 == io->header.len + sizeof(io->header))
	    val |= DATA_EOI;
	  abort = writeByte(val);
	  if (abort)
	    len = 0;
	  else
	    io->outpos += len;
	}
      }
    }

    if (abort || io->outpos == io->header.len + sizeof(io->header))
      io->outpos = 0;

    count -= len;
    offset += len;
  }

  return offset;
}

#endif /* BUILD_IEC */
