import serial
import warnings
from time import sleep

MAX_READ_COUNT=100

class SerialBuffer:
  def __init__(self, port="/dev/ttyUSB0", baud=115200):
    self.serial = serial.Serial(port, baud, timeout=0.01)
    self.messages = []
    self.buf = []

  def is_open(self):
    return self.serial.isOpen()

  def spin_once(self):
    data = self.serial.readline()
    if len(data) != 0:
      return self.parse(data)
    else:
      return None

  def write(self, data):
    # output = self.serial.write('$' + data + '*' + str(self.checksum(data)) + '\r\n')
    output = self.serial.write('$' + data + '\r\n')
    print 'Printed message:' + str(output)

  def checksum(self, str):
    sum = 0x00
    for ch in str:
      sum ^= ord(ch)
    return sum

  def parse(self, raw):

    # print raw

    try:
      if raw[0] != '$':
        warnings.warn('Wrong frame start:' + raw)
        return None
      if raw[-1] == '\n':
        raw = raw[:-2]
      if raw[-1] == '\r':
        raw = raw[:-2]
      if raw[-3] != '*':
        warnings.warn('Cannot find *:' + raw)
        return None

      sum = self.checksum(raw[1:-3])
      if sum != int(raw[-2:], 16):
        warnings.warn('Checksum not match')
        return None

      return raw[:-3].split(',')

    except:
      return None


if __name__ == '__main__':
  sb = SerialBuffer()
  print sb.is_open()
  print sb.spin_once()
