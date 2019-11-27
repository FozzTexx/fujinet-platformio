#include <Arduino.h>

#include "config.h"
#include "ssid.h"

#ifdef ESP_8266
#include <FS.h>
#define INPUT_PULLDOWN INPUT_PULLDOWN_16 // for motor pin
#elif defined(ESP_32)
#include <SPIFFS.h>
#endif

enum
{
  ID,
  COMMAND,
  AUX1,
  AUX2,
  CHECKSUM,
  ACK,
  NAK,
  PROCESS,
  WAIT
} cmdState;
volatile bool cmdFlag = false;

#define DELAY_T5 600
#define READ_CMD_TIMEOUT 12
#define CMD_TIMEOUT 50
#define STATUS_SKIP 8

unsigned long cmdTimer = 0;
byte statusSkipCount = 0;

union {
  struct
  {
    unsigned char devic;
    unsigned char comnd;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char cksum;
  };
  byte cmdFrameData[5];
} cmdFrame;

File atr;

/**
   calculate 8-bit checksum.
*/
byte sio_checksum(byte *chunk, int length)
{
  int chkSum = 0;
  for (int i = 0; i < length; i++)
  {
    chkSum = ((chkSum + chunk[i]) >> 8) + ((chkSum + chunk[i]) & 0xff);
  }
  return (byte)chkSum;
}

/**
   ISR for falling COMMAND
*/
void ICACHE_RAM_ATTR sio_isr_cmd()
{
  cmdFlag = true;
}

/**
   Get ID
*/
void sio_get_id()
{
  cmdFrame.devic = SIO_UART.read();
  if (cmdFrame.devic == 0x31)
    cmdState = COMMAND;
  else
  {
    cmdState = WAIT;
    cmdTimer = 0;
  }

#ifdef DEBUG_S
  BUG_UART.print("CMD DEVC: ");
  BUG_UART.println(cmdFrame.devic, HEX);
#endif
}

/**
   Get Command
*/
void sio_get_command()
{
  cmdFrame.comnd = SIO_UART.read();
  if (cmdFrame.comnd == 'S' && statusSkipCount >= STATUS_SKIP)
    cmdState = AUX1;
  else if (cmdFrame.comnd == 'S' && statusSkipCount < STATUS_SKIP)
  {
    statusSkipCount++;
    cmdState = WAIT;
    cmdTimer = 0;
  }
  else if (cmdFrame.comnd == 'R')
    cmdState = AUX1;
  else
  {
    cmdState = WAIT;
    cmdTimer = 0;
  }

#ifdef DEBUG_S
  BUG_UART.print("CMD CMND: ");
  BUG_UART.println(cmdFrame.comnd, HEX);
#endif
}

/**
   Get aux1
*/
void sio_get_aux1()
{
  cmdFrame.aux1 = SIO_UART.read();
  cmdState = AUX2;

#ifdef DEBUG_S
  BUG_UART.print("CMD AUX1: ");
  BUG_UART.println(cmdFrame.aux1, HEX);
#endif
}

/**
   Get aux2
*/
void sio_get_aux2()
{
  cmdFrame.aux2 = SIO_UART.read();
  cmdState = CHECKSUM;

#ifdef DEBUG_S
  BUG_UART.print("CMD AUX2: ");
  BUG_UART.println(cmdFrame.aux2, HEX);
#endif
}

/**
   Read
*/
void sio_read()
{
  byte ck;
  byte sector[128];
  int offset = (256 * cmdFrame.aux2) + cmdFrame.aux1;
  offset *= 128;
  offset -= 128;
  offset += 16; // skip 16 byte ATR Header
  atr.seek(offset, SeekSet);
  atr.read(sector, 128);

  ck = sio_checksum((byte *)&sector, 128);

  delayMicroseconds(1500); // t5 delay
  SIO_UART.write('C');     // Completed command
  SIO_UART.flush();

  // Write data frame
  SIO_UART.write(sector, 128);

  // Write data frame checksum
  SIO_UART.write(ck);
  SIO_UART.flush();
  delayMicroseconds(200);
#ifdef DEBUG_S
  BUG_UART.print("SIO READ OFFSET: ");
  BUG_UART.print(offset);
  BUG_UART.print(" - ");
  BUG_UART.println((offset + 128));
#endif
}

/**
   Status
*/
void sio_status()
{
  byte status[4] = {0x00, 0xFF, 0xFE, 0x00};
  byte ck;

  ck = sio_checksum((byte *)&status, 4);

  delayMicroseconds(1500); // t5 delay
  SIO_UART.write('C');     // Command always completes.
  SIO_UART.flush();
  delayMicroseconds(200);
  //delay(1);

  // Write data frame
  for (int i = 0; i < 4; i++)
    SIO_UART.write(status[i]);

  // Write checksum
  SIO_UART.write(ck);
  SIO_UART.flush();
  delayMicroseconds(200);
}

/**
   Process command
*/

void sio_process()
{
  switch (cmdFrame.comnd)
  {
  case 'R':
    sio_read();
    break;
  case 'S':
    sio_status();
    break;
  }

  cmdState = WAIT;
  cmdTimer = 0;
}

/**
   Send an acknowledgement
*/
void sio_ack()
{
  delayMicroseconds(500);
  SIO_UART.write('A');
  SIO_UART.flush();
  //cmdState = PROCESS;
  sio_process();
}

/**
   Send a non-acknowledgement
*/
void sio_nak()
{
  delayMicroseconds(500);
  SIO_UART.write('N');
  SIO_UART.flush();
  cmdState = WAIT;
  cmdTimer = 0;
}

/**
   Get Checksum, and compare
*/
void sio_get_checksum()
{
  byte ck;
  cmdFrame.cksum = SIO_UART.read();
  ck = sio_checksum((byte *)&cmdFrame.cmdFrameData, 4);

#ifdef DEBUG_S
  BUG_UART.print("CMD CKSM: ");
  BUG_UART.print(cmdFrame.cksum, HEX);
#endif

  if (ck == cmdFrame.cksum)
  {
#ifdef DEBUG_S
    BUG_UART.println(", ACK");
#endif
    sio_ack();
  }
  else
  {
#ifdef DEBUG_S
    BUG_UART.println(", NAK");
#endif
    sio_nak();
  }
}

void sio_incoming()
{
  switch (cmdState)
  {
  case ID:
    sio_get_id();
    break;
  case COMMAND:
    sio_get_command();
    break;
  case AUX1:
    sio_get_aux1();
    break;
  case AUX2:
    sio_get_aux2();
    break;
  case CHECKSUM:
    sio_get_checksum();
    break;
  case ACK:
    sio_ack();
    break;
  case NAK:
    sio_nak();
    break;
  case PROCESS:
    sio_process();
    break;
  case WAIT:
    SIO_UART.read(); // Toss it for now
    cmdTimer = 0;
    break;
  }
}

void setup()
{
  SPIFFS.begin();
  atr = SPIFFS.open("/autorun.atr", "r");

  // Set up pins
#ifdef DEBUG_S
  BUG_UART.begin(19200);
  BUG_UART.println();
  BUG_UART.println("atariwifi started");
#else
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
#endif
  pinMode(PIN_INT, INPUT);
  pinMode(PIN_PROC, INPUT);
  pinMode(PIN_MTR, INPUT_PULLDOWN);
  pinMode(PIN_CMD, INPUT);

  // Set up serial
  SIO_UART.begin(19200);
#ifdef ESP_8266
  SIO_UART.swap();
#endif

  // Attach COMMAND interrupt.
  //attachInterrupt(digitalPinToInterrupt(PIN_CMD), sio_isr_cmd, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CMD), sio_isr_cmd, FALLING);
  cmdState = WAIT; // Start in wait state
}

void loop()
{
  if (cmdFlag)
  {
    if (digitalRead(PIN_CMD) == LOW) // this check may not be necessary
    {
      cmdState = ID;
      cmdTimer = millis();
      cmdFlag = false;
    }
  }

  if (SIO_UART.available() > 0)
  {
    sio_incoming();
  }

  if (millis() - cmdTimer > CMD_TIMEOUT && cmdState != WAIT)
  {
#ifdef DEBUG_S
    BUG_UART.print("SIO CMD TIMEOUT: ");
    BUG_UART.println(cmdState);
#endif
    cmdState = WAIT;
    cmdTimer = 0;
  }
}
