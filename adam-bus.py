#!/usr/bin/env python3
import argparse
import csv
from enum import Enum, auto
import struct
from hexdump import hexdump

BYTE_TIME = 10 / 62500

class CMD(Enum):
  MN_RESET   = 0x00
  MN_STATUS  = 0x10
  MN_ACK     = 0x20
  MN_CLR     = 0x30
  MN_RECEIVE = 0x40
  MN_CANCEL  = 0x50
  MN_SEND    = 0x60
  MN_NACK    = 0x70
  MN_READY   = 0xD0

  NM_STATUS  = 0x80
  NM_ACK     = 0x90
  NM_CANCEL  = 0xA0
  NM_SEND    = 0xB0
  NM_NACK    = 0xC0

CMD.max_width = max(len(x.name) for x in CMD)

class DEV(Enum):
  Master     = 0x00
  Keyboard   = 0x01
  Printer    = 0x02
  Reserved   = 0x03
  Disk_1     = 0x04
  Disk_2     = 0x05
  Disk_3     = 0x06
  Disk_4     = 0x07
  Tape       = 0x08
  Network_1  = 0x09
  Network_2  = 0x0A
  Reserved_1 = 0x0B
  Reserved_2 = 0x0C
  Parallel   = 0x0D
  RS232C     = 0x0E
  Fujinet    = 0x0F

DEV.max_width = max(len(x.name) for x in DEV)

def s2u(val):
  return int(val * 1000000)

class State(Enum):
  IDLE = auto()
  INCOMPLETE = auto()
  LENGTH = auto()

def build_argparser():
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("file", help="input file")
  parser.add_argument("--flag", action="store_true", help="flag to do something")
  return parser

class AdamBusViewer:
  def __init__(self, headers, data):
    self.headers = headers
    self.data = data
    self.state = State.IDLE
    self.lastPacketTimestamp = None
    return

  def dumpItAll(self):
    for row in self.data:
      #self.processByte(row[4], row[2])
      self.processByte(int(row[1], 16), float(row[0]))
    return

  def processByte(self, value, timestamp):
    if self.state == State.IDLE:
      self.beginCommand(value, timestamp)

    elif self.state == State.INCOMPLETE:
      if s2u(timestamp - self.packet.mostRecentTimestamp) > s2u(BYTE_TIME * 2):
        # Short packet
        self.packet.remaining = 0
      else:
        self.packet.append(value, timestamp)

      if not self.packet.remaining:
        self.packetComplete(self.packet)

    elif self.state == State.LENGTH:
      if not self.packet.remaining:
        raise ValueError("Out of sync")
      self.packet.append(value, timestamp)
      if not self.packet.remaining:
        self.state = State.INCOMPLETE
        self.packet.remaining = self.packet.decodeLength(">") + 1

    else:
      raise ValueError("Unknown state", self.state)
    return

  def beginCommand(self, value, timestamp):
    if value == 0xFF:
      print(f"{value:02x} RESET")
      return

    packet = AdamNetPacket(value, timestamp)
    if packet.command == CMD.MN_RECEIVE \
       or packet.command == CMD.NM_ACK \
       or packet.command == CMD.MN_STATUS \
       or packet.command == CMD.MN_ACK \
       or packet.command == CMD.MN_READY \
       or packet.command == CMD.MN_CLR \
       or packet.command == CMD.NM_NACK:
      self.packetComplete(packet)

    elif packet.command == CMD.NM_STATUS:
      self.packet = packet
      self.state = State.INCOMPLETE
      self.packet.remaining = 5

    elif packet.command == CMD.MN_SEND \
         or packet.command == CMD.NM_SEND:
      self.packet = packet
      self.state = State.LENGTH
      self.packet.remaining = 2

    elif packet.command == CMD.MN_CANCEL \
         or packet.command == CMD.MN_NACK:
      raise NotImplementedError("What do next with", packet.command, packet.device)

    elif packet.command == CMD.NM_CANCEL:
      raise NotImplementedError("Node reply", packet.command, packet.device)

    else:
      raise ValueError("Unknown command", packet.command, packet.device, packet.timestamp)

    return

  def packetComplete(self, packet):
    if self.lastPacketTimestamp is None:
      self.lastPacketTimestamp = packet.timestamp
    if hasattr(self, 'packet') and hasattr(self.packet, 'remaining'):
      self.packet.decode()
    packet.print(self.lastPacketTimestamp)
    self.lastPacketTimestamp = packet.timestamp
    self.state = State.IDLE
    if hasattr(self, 'packet'):
      del self.packet
    return

class AdamNetPacket:
  def __init__(self, value, timestamp):
    #print(f"VALUE 0x{value:02x}")
    cmd = value & 0xF0
    dev = value & 0x0F
    self.command = CMD(cmd)
    self.device = DEV(dev)
    self.mostRecentTimestamp = self.timestamp = timestamp
    self.data = []
    self.decoded = {}
    return

  def print(self, prevTimestamp):
    if self.command.name.startswith("MN_"):
      direction = "-->"
    else:
      direction = "<--"
    info = f"{s2u(self.timestamp):7d}" \
      f" {s2u(self.timestamp - prevTimestamp):+8d}" \
      f" 0x{self.combined:02x} {self.command.name:<{CMD.max_width}}" \
      f" {direction} {self.device.name:<{DEV.max_width}}"
    if 'block' in self.decoded:
      info += f"  BLOCK {self.decoded['block']}"

    if 'checksum' in self.decoded \
       and self.decoded['checksum'] != self.decoded['checksum_calc']:
      info += " CHECKSUM MISMATCH"
    if 'length' in self.decoded and self.decoded['length'] != len(self.data) - 3:
      info += f" SHORT {self.decoded['length'] - (len(self.data) - 3)}"

    print(info.rstrip())
    if self.data:
      if len(self.data) < 64:
        hexdump(self.data, prefix="  ")
      else:
        hexdump(self.data[:2], prefix="  ")
        print("  ...")
        hexdump(self.data[-1:], prefix="  ", address=len(self.data) - 1)
      print(" ", {x: self.decoded[x] for x in self.decoded if x not in ("data", )})
    return

  def append(self, value, timestamp):
    if not self.remaining:
      raise ValueError("Out of sync")
    self.data.append(value)
    self.mostRecentTimestamp = timestamp
    self.remaining -= 1
    return

  def decode(self):
    if self.command == CMD.NM_STATUS:
      fields = struct.unpack("<HBBB", bytes(self.data))
      self.decoded = {
        'max_size': fields[0],
        'type': fields[1],
        'status': fields[2],
        'checksum': fields[3],
        'checksum_calc': self.calculateChecksum(self.data[0:-1]),
      }

    elif self.command == CMD.MN_SEND \
         or self.command == CMD.NM_SEND:
      self.decoded = {
        'length': self.decodeLength(">"),
        'data': self.data[2:-1],
        'checksum': self.data[-1],
        'checksum_calc': self.calculateChecksum(self.data[2:-1]),
      }

      if self.command == CMD.MN_SEND \
         and self.device in (DEV.Disk_1, DEV.Disk_2, DEV.Disk_3, DEV.Disk_4, DEV.Tape) \
         and self.decoded['length'] == 5:
        block = struct.unpack("<I", bytes(self.data[2:6]))[0]
        self.decoded['block'] = block
        del self.decoded['data']

    else:
      raise NotImplementedError("Don't know how to decode", self.command)

    return

  def decodeLength(self, endian):
    fields = struct.unpack(endian + "H", bytes(self.data[:2]))
    return fields[0]

  def calculateChecksum(self, values):
    checksum = 0
    for val in values:
      checksum ^= val
    return checksum

  @property
  def combined(self):
    return self.command.value | self.device.value

def main():
  args = build_argparser().parse_args()

  with open(args.file, "r") as f:
    reader = csv.reader(f)
    data = [row for row in reader]

  headers = data[0]
  data = data[1:]

  abv = AdamBusViewer(headers, data)
  abv.dumpItAll()

  return

if __name__ == "__main__":
  exit(main() or 0)
