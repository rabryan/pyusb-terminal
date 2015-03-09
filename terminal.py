#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

try:
  from future_builtins import (ascii, filter, hex, map, oct, zip)
except:
  pass

import time
import usb.core as usb
import logging as log
import threading
import sys
try:
  import Queue
except ImportError:
  import queue as Queue
import os
if os.name == "nt":
  import msvcrt


class ComPort( object ):
  def __init__( self, usb_device ):
    self.device = usb_device

    cfg = usb_device.get_active_configuration()
    cmd_itfs = list(usb.util.find_descriptor( cfg, find_all = True,
        custom_match = lambda e: (e.bInterfaceClass == 0x2) ))
    data_itfs = list(usb.util.find_descriptor( cfg, find_all = True,
        custom_match = lambda e: (e.bInterfaceClass == 0xA) ))

    if( len(cmd_itfs) != len(data_itfs) ):
      log.debug( "COM port data / command interface mismatch" )

    ports = len( data_itfs )
    log.debug("found {0} COM port\n".format(ports))

#    for i, cmd_itf, data_itf in zip( xrange(len(data_itfs)), cmd_itfs, data_itfs ):
    cmd_itf = cmd_itfs[0]
    data_itf = data_itfs[0]

    try:
      self.device.detach_kernel_driver( cmd_itf.bInterfaceNumber )
    except usb.USBError:
      pass
    except NotImplementedError:
      pass

    self.ep_in = usb.util.find_descriptor( data_itf,
        custom_match = lambda e: ( e.bEndpointAddress & 0x80 ) )
    self.ep_out = usb.util.find_descriptor( data_itf,
        custom_match = lambda e: not (e.bEndpointAddress & 0x80))

  def write( self, data ):
    ret = self.ep_out.write( data )
    if( len(data) != ret ):
      print( "Bytes written mismatch {0} vs {1}".format(len(data), ret) )

  def read( self, timeout = None, attempts = 4 ):
    ret = []
    size = self.ep_in.wMaxPacketSize
    if timeout is None:
      for i in range(attempts):
        try:
          ret += self.ep_in.read( size )
        except usb.USBError:
          pass
    else:
      for i in range(attempts):
        try:
          ret += self.ep_in.read( size, timeout )
        except usb.USBError:
          pass
    return ret

  def read_text( self, timeout = None, attempts = 4 ):
    return "".join( chr(a) for a in self.read( timeout, attempts ) )


if __name__ == '__main__':
  log.basicConfig(level=log.DEBUG)

  devices = list(usb.find(find_all=True))

  if len(devices) == 0:
    print("No devices detected")
    exit()

  selection = -1
  selected = False
  while not selected:
    for i,d in enumerate(devices):
      try:
        manufacturer = d.manufacturer
      except:
        manufacturer = "Unknown"
      print ("%d: %04x:%04x on Bus %03d Address %03d %s" % (i, d.idVendor,
        d.idProduct, d.bus, d.address, manufacturer))

    selection = input("Enter device: ")

    try:
      selection = int(selection)
      if selection < 0 or selection >= len(devices):
        raise Exception()
      selected = True
    except:
      print("Please enter number between 0 and {}".format(len(devices) - 1))

  #try:
  d = devices[selection]
  p = ComPort(d)
  #except Exception as e:
  #  print("Cannot open com port for {}".format(str(d)))
  #  print(str(e))
  #  exit()

  input_queue = Queue.LifoQueue()
  def readinput():
    while True:
      if os.name == "nt":
        command = msvcrt.getch()
      else:
        command = sys.stdin.read(1)

      input_queue.put(command)

  input_thread = threading.Thread(target=readinput)
  input_thread.daemon = True
  input_thread.start()

  while True:
    text = p.read_text(timeout=1)
    if len(text):
      print(text)

    if not input_queue.empty():
      command = input_queue.get()
      p.write(command)


# vim:shiftwidth=2
