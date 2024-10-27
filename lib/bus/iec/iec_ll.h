// ported from ninepin Linux kernel driver
// fozztexx@fozztexx.com

#ifdef BUILD_IEC

#ifndef _IEC_LL_H
#define _IEC_LL_H

#include <stdint.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define IEC_BUFSIZE     1024
#define IEC_FIRSTDEV    4
#define IEC_ALLDEV      31

typedef enum {
  IECListenCommand      = 0x20,
  IECTalkCommand        = 0x40,
  IECReopenCommand      = 0x60,
  IECCloseCommand       = 0xe0,
  IECOpenCommand        = 0xf0,
} IECCommand_t;
#define IECFileCommand IECCloseCommand

typedef struct {
  uint8_t command;
  uint8_t channel;
  uint16_t len;
  uint8_t eoi;
  uint8_t serial;
} iec_record_header;

struct iec_io;
typedef struct iec_io {
  iec_record_header header;
  uint8_t data[IEC_BUFSIZE];
  iec_io *next;
  int outpos;
} iec_io;

typedef struct {
  iec_io *head, *tail, *cur;
} iec_chain;

typedef struct {
  iec_chain in;
  iec_io out;
  SemaphoreHandle_t lock;
} iec_device;

class IECLowLevel
{
 private:
  int iec_state, iec_atnState;
  iec_device *iec_openDevices[IEC_ALLDEV+1];
  int iec_curDevice, iec_curChannel;
  uint16_t *iec_buffer;
  int iec_inpos, iec_outpos;
  int iec_readAvail = 0;
  int iec_deviceCount = 0;
  int c64slowdown = 10;
  SemaphoreHandle_t iec_canRead = nullptr;

  typedef struct {
    IECLowLevel *handler;
    void (IECLowLevel::*method)();
  } isr_handler;
  isr_handler atn_isr, clk_isr;

  void handleATN();
  void handleCLK();
  int waitForSignals(int pin, int val, int pin2, int val2, int delay);
  int readByte();
  int writeByte(int bits);
  void releaseBus(void);
  void sendInput(void);
  void newIO(int val);
  void channelIO(int val);
  int unlinkIO(iec_device *device);
  int setupTalker(void);
  void appendByte(int val);
  void processData();
  void configPin(gpio_num_t pin, gpio_int_type_t intr_type, isr_handler *handler);
  void cleanupDevice(int devnum);

  /* Sneaky ISR forwarding function in private so I don't have to
     declare handleATN() or handleCLK() as public */
  static void isr_forwarder(void *arg)
  {
    isr_handler *h = (isr_handler *) arg;
    (h->handler->*h->method)();
    return;
  }

 public:
  //IECLowLevel() { init(); }
  bool init();
  int open(int devnum);
  int close(int devnum);
  size_t hasData(int devnum);
  size_t read(int devnum, uint8_t *buf, size_t count);
  size_t write(int devnum, const uint8_t *buf, size_t count);
};

#endif /* _IEC_LL_H */
#endif /* BUILD_IEC */
