#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

try:
  from future_builtins import (ascii, filter, hex, map, oct, zip)
except:
  pass

import sys
import time
import usb.core as usb
import threading

import logging

try:
  import Queue as queue
except ImportError:
  import queue
import os
if os.name == "nt":
  import msvcrt

log = logging.getLogger( )
log.setLevel( logging.DEBUG )
fileHandler = logging.FileHandler( "terminal.log" )
log_fmt = logging.Formatter(
    "%(levelname)s %(name)s %(threadName)-10s %(funcName)s() %(message)s" )
fileHandler.setFormatter( log_fmt )
log.addHandler( fileHandler )


# NOTE: getch taken from http://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user
class _Getch(object):
  """ Gets a single character from standard input.
      Does not echo to the screen.
  """
  def __init__(self):
    try:
      self.impl = _GetchWindows()
    except ImportError:
      self.impl = _GetchUnix()

  def __call__(self):
    char = self.impl()
    if char == '\x03':
      raise KeyboardInterrupt
    elif char == '\x04':
      raise EOFError
    return char

class _GetchUnix(object):
  def __init__(self):
    import tty
    import sys

  def __call__(self):
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr( fd )
    try:
      tty.setraw( sys.stdin.fileno() )
      ch = sys.stdin.read( 1 )
    finally:
      termios.tcsetattr( fd, termios.TCSADRAIN, old_settings )
    return ch

class _GetchWindows(object):
  def __init__(self):
    import msvcrt

  def __call__(self):
    import msvcrt
    return msvcrt.getch()

getch = _Getch()


class ComPort( object ):
  def __init__( self, usb_device ):
    self.device = usb_device
    self._isFTDI = False

    cfg = usb_device.get_active_configuration()

    if self.device.idVendor == 0x0403:       # FTDI device
      self._isFTDI = True
      log.debug( "Configuring as an FTDI device, no cmd itf" )
      cmd_itfs = None
      data_itfs = list(usb.util.find_descriptor( cfg, find_all = True,
          custom_match = lambda e: (e.bInterfaceClass == 0xFF) ))
      data_itf = data_itfs[0]
      itf_num = data_itf.bInterfaceNumber
    else:
      data_itfs = list(usb.util.find_descriptor( cfg, find_all = True,
          custom_match = lambda e: (e.bInterfaceClass == 0xA) ))
      data_itf = data_itfs[0]
      cmd_itfs = list( usb.util.find_descriptor( cfg, find_all = True,
        custom_match = lambda e: (e.bInterfaceClass == 0x2) ) )
      itf_num = cmd_itfs[0].bInterfaceNumber
      if( len(cmd_itfs) != len(data_itfs) ):
        log.debug( "COM port data / command interface mismatch" )

    ports = len( data_itfs )
    log.debug( "found {0} COM port\n".format(ports) )

    try:
      self.device.detach_kernel_driver( itf_num )
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
      log.error( "Bytes written mismatch {0} vs {1}".format( len(data), ret)  )

  def read( self, timeout = None, attempts = 4 ):
    ret = []
    size = self.ep_in.wMaxPacketSize
    kwargs = dict() if timeout is None else { "timeout":timeout }
    for i in range( attempts ):
      try:
        rv = self.ep_in.read( size, **kwargs )
        if self._isFTDI:
          status = rv[:2]       # FTDI prepends 2 flow control characters
          rv = rv[2:]
        ret += rv
      except usb.USBError:
        pass
    return ret

  def read_text( self, timeout = None, attempts = 4 ):
    return "".join( chr(a) for a in self.read( timeout, attempts ) )

def selectDevice( ):
  devices = list( usb.find( find_all=True ) )

  if len( devices ) == 0:
    print( "No devices detected" )
    return None

  selection = -1
  selected = False

  while not selected:
    for i,d in enumerate( devices ):
      try:
        manufacturer = d.manufacturer
      except:
        manufacturer = "Unknown"
      print( "%d: %04x:%04x on Bus %03d Address %03d %s" % (i, d.idVendor,
        d.idProduct, d.bus, d.address, manufacturer) )

    selection = input( "Enter device: " )

    try:
      selection = int( selection )
      if selection < 0 or selection >= len(devices):
        raise Exception()
      selected = True
    except:
      print( "Please enter number between 0 and {}".format(len(devices) - 1) )

  #try:
  d = devices[selection]
  #except Exception as e:
  #  log.error("Cannot open com port for {}".format(str(d)))
  #  print(str(e))
  #  exit()

  return d

def readinput( queue ):
  while True:
    queue.put( getch() )


if __name__ == '__main__':
  d = selectDevice(  )

  if d is None:
    exit()

  p = ComPort(d)

  input_queue = queue.Queue()
  input_thread = threading.Thread( target=lambda : readinput( input_queue ) )
  input_thread.daemon = True
  input_thread.start()

  while True:
    time.sleep( 0.2 )
    text = p.read_text( timeout=1 )
    if len( text ):
      valid = set( list(range(ord(' '), ord('~') + 1)) + [ ord('\r'), ord( '\n' ) ] )
      print(
          "".join( "\\x{:02X}".format( ord(c) ) if ord(c) not in valid else c for c in text  ),
          end="" )

    if not input_queue.empty():
      command = input_queue.get()
      print( command, end="" )
      p.write(command)


# vim:shiftwidth=2
