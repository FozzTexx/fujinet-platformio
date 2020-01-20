/**
 * FujiNet Tools for CLI
 *
 * fnew - make new ATR disk on host
 *
 * usage:
 *  feject <ds#> <hs#> <sect> <sectsize> <fname>
 *
 * Author: Thomas Cherryhomes
 *  <thom.cherryhomes@gmail.com>
 *
 * Released under GPL, see COPYING
 * for details
 */

#include <atari.h>
#include <string.h>
#include <stdlib.h>
#include "sio.h"
#include "conio.h"
#include "err.h"

unsigned char buf[80];

union
{
  char host[8][32];
  unsigned char rawData[256];
} hostSlots;

union
{
  struct
  {
    unsigned char hostSlot;
    unsigned char mode;
    char file[36];
  } slot[8];
  unsigned char rawData[304];
} deviceSlots;

union
{
  struct
  {
    unsigned short numSectors;
    unsigned short sectorSize;
    unsigned char hostSlot;
    unsigned char deviceSlot;
    char filename[36];
  };
  unsigned char rawData[42];
} newDisk;

/**
 * Read host slots
 */
void host_read(void)
{
  // Query for host slots
  OS.dcb.ddevic=0x70;
  OS.dcb.dunit=1;
  OS.dcb.dcomnd=0xF4; // Get host slots
  OS.dcb.dstats=0x40;
  OS.dcb.dbuf=&hostSlots.rawData;
  OS.dcb.dtimlo=0x0f;
  OS.dcb.dbyt=256;
  OS.dcb.daux=0;
  siov();

  if (OS.dcb.dstats!=1)
    {
      err_sio();
      exit(OS.dcb.dstats);
    }
}

/**
 * Read Device Slots
 */
void disk_read(void)
{
  // Read Drive Tables
  OS.dcb.ddevic=0x70;
  OS.dcb.dunit=1;
  OS.dcb.dcomnd=0xF2;
  OS.dcb.dstats=0x40;
  OS.dcb.dbuf=&deviceSlots.rawData;
  OS.dcb.dtimlo=0x0f;
  OS.dcb.dbyt=sizeof(deviceSlots.rawData);
  OS.dcb.daux=0;
  siov();

  if (OS.dcb.dstats!=1)
    {
      err_sio();
      exit(OS.dcb.dstats);
    }
}

/**
 * Write device slots
 */
void disk_write(void)
{
  OS.dcb.ddevic=0x70;
  OS.dcb.dunit=1;
  OS.dcb.dcomnd=0xF1;
  OS.dcb.dstats=0x80;
  OS.dcb.dbuf=&deviceSlots.rawData;
  OS.dcb.dtimlo=0x0f;
  OS.dcb.dbyt=sizeof(deviceSlots.rawData);
  OS.dcb.daux=0;
  siov();

  if (OS.dcb.dstats!=1)
    {
      err_sio();
      exit(OS.dcb.dstats);
    }
}

/**
 * Create disk in slot
 */
void disk_create(unsigned short ns, unsigned short ss, unsigned char hs, unsigned char ds, char* fname)
{
  newDisk.numSectors=ns;
  newDisk.sectorSize=ss;
  newDisk.hostSlot=hs;
  newDisk.deviceSlot=ds;
  strcpy(newDisk.filename,fname);

  // Query for host slots
  OS.dcb.ddevic=0x70;
  OS.dcb.dunit=1;
  OS.dcb.dcomnd=0xE7; // TNFS Create Disk
  OS.dcb.dstats=0x80;
  OS.dcb.dbuf=&newDisk.rawData;
  OS.dcb.dtimlo=0xFE;
  OS.dcb.dbyt=sizeof(newDisk.rawData);
  OS.dcb.daux=0;
  siov();

  if (OS.dcb.dstats!=1)
    {
      err_sio();
      exit(OS.dcb.dstats);
    }
}

/**
 * Mount a host slot
 */
void host_mount(unsigned char c)
{
  if (hostSlots.host[c][0]!=0x00)
    {
      OS.dcb.ddevic=0x70;
      OS.dcb.dunit=1;
      OS.dcb.dcomnd=0xF9;
      OS.dcb.dstats=0x00;
      OS.dcb.dbuf=NULL;
      OS.dcb.dtimlo=0x01;
      OS.dcb.dbyt=0;
      OS.dcb.daux=c;
      siov();
      if (OS.dcb.dstats!=1)
	{
	  err_sio();
	  exit(OS.dcb.dstats);
	}
    }
}

/**
 * Mount device slot
 */
void disk_mount(unsigned char c, unsigned char o)
{
  OS.dcb.ddevic=0x70;
  OS.dcb.dunit=1;
  OS.dcb.dcomnd=0xF8;
  OS.dcb.dstats=0x00;
  OS.dcb.dbuf=NULL;
  OS.dcb.dtimlo=0x01;
  OS.dcb.dbyt=0;
  OS.dcb.daux1=c;
  OS.dcb.daux2=o;
  siov();
  if (OS.dcb.dstats!=1)
    {
      err_sio();
      exit(OS.dcb.dstats);
    }
}

/**
 * show options
 */
void opts(char* argv[])
{
  print(argv[0]);
  print(" <DS#> <HS#> <NS> <SS> <FNAME>\x9b\x9b");
  print("<DS#>   - device slot (1-8)\x9b");
  print("<HS#>   - host slot (1-8)\x9b");
  print("<NS>    - Number of Sectors\x9b");
  print("<SS>    - sector size (128 or 256)\x9b");
  print("<FNAME> - Image Filename\x9b");
}

/**
 * main
 */
int main(int argc, char* argv[])
{
  unsigned char ds=argv[1][0]-0x30;
  unsigned char hs=argv[2][0]-0x30;
  unsigned char dsa=argv[1][0];
  unsigned char hsa=argv[2][0];
  unsigned short ns=atoi(argv[3]);
  unsigned short ss=atoi(argv[4]);
  
  if (_is_cmdline_dos())
    {
      if (argc<5)
	{
	  opts(argv);
	  return(1);
	}
      strcpy(buf,argv[5]);
    }
  else
    {
      // DOS 2.0
      print("\x9b");
      print("DEVICE SLOT (1-8)? ");
      get_line(buf,sizeof(buf));
      ds=buf[0]-0x30;
      dsa=buf[0];

      print("HOST SLOT (1-8)? ");
      get_line(buf,sizeof(buf));
      hs=buf[0]-0x30;
      hsa=buf[0];

      print("NUMBER OF SECTORS (1-65535)? ");
      get_line(buf,sizeof(buf));
      ns=atoi(buf);

      print("SECTOR SIZE (128/256)? ");
      get_line(buf,sizeof(buf));
      ss=atoi(buf);

      print("FILENAME:\x9b");
      get_line(buf,sizeof(buf));
    }

  if (ds<1 || ds>8)
    {
      print("INVALID DRIVE SLOT NUMBER.\x9b");
      return(1);
    }

  if (hs<1 || hs>8)
    {
      print("INVALID HOST SLOT NUMBER.\x9b");
      return(1);
    }
  
  hs-=1;
  ds-=1;

  // Read in existing data from FujiNet
  host_read();
  disk_read();

  // Mount desired host
  host_mount(hs);

  // Set desired slot filename/mode
  strcpy(deviceSlots.slot[ds].file,buf);
  deviceSlots.slot[ds].mode=3; // R/W
  deviceSlots.slot[ds].hostSlot=hs;

  // Write out disk slot
  disk_write();

  // Create the disk
  print("\x9b");
  print("CREATING DISK\x9b");
  disk_create(ns,ss,hs,ds,buf);
  
  // Mount the disk.
  disk_mount(ds,3);

  print("D");
  printc(&dsa);
  print(": ");
  print("(");
  printc(&hsa);
  print(") ");
  print("(");
  print("W");
  print(") ");
  print(buf);
  print("\x9b");

  if (!_is_cmdline_dos())
    {
      print("\x9bPRESS \xA0\xD2\xC5\xD4\xD5\xD2\xCE\xA0 TO CONTINUE.\x9b");
      get_line(buf,sizeof(buf));
    }  
  
  return 0;
}
