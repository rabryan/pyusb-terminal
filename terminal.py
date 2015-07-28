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
    return self.impl()

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

class ComPort( object ):
  def __init__( self, usb_device, start=True ):
    self.device = usb_device
    self._isFTDI = False
    self._rxinterval = 0.005            # sec
    self._rxqueue = queue.Queue()
    self._rxthread = None
    self._rxactive = False

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

    if start:
      self._startRx()

  def _startRx( self ):
    if self._rxthread is not None and (self._rxactive or self._rxthread.isAlive()):
      return
    self._rxactive = True
    self._rxthread = threading.Thread( target=self._read )
    self._rxthread.daemon = True
    self._rxthread.start()

  def _endRx( self ):
    self._rxactive = False

  def _read( self ):
    """ check ep for data, add it to queue and sleep for interval """
    while self._rxactive:
      try:
        rv = self.ep_in.read( self.ep_in.wMaxPacketSize )
        if self._isFTDI:
          status = rv[:2]       # FTDI prepends 2 flow control characters
          if status[0] != 1 or status [1] != 0x60:
            log.info( "USB Status: 0x{0:02X} 0x{1:02X}".format( *status ) )
          rv = rv[2:]
        for rvi in rv:
          self._rxqueue.put( rvi )
      except usb.USBError as e:
        log.warn( "USB Error on _read {}".format( e ) )
        pass
      time.sleep( self._rxinterval )

  def _getRxLen( self ):
    return self._rxqueue.qsize()
  rxlen = property( fget=_getRxLen )

  def readBytes( self ):
    rx = []
    while not self._rxqueue.empty():
      rx.append( self._rxqueue.get() )
    return rx

  def readText( self ):
    return "".join( chr(c) for c in self.readBytes() )

  def write( self, data ):
    ret = self.ep_out.write( data )
    if( len( data ) != ret ):
      log.error( "Bytes written mismatch {0} vs {1}".format( len(data), ret)  )
    else:
      log.debug( "{} bytes written to ep".format( ret ) )

  def disconnect( self ):
    self._endRx()
    while self._rxthread is not None and self._rxthread.isAlive():
      pass
    usb.util.dispose_resources( self.device )
    if self._rxthread is None:
      log.debug( "Rx thread never existed" )
    else:
      log.debug( "Rx thread is {}".format(
        "alive" if self._rxthread.isAlive() else "dead" ) )
    attempt=1
    while attempt < 10:
      try:
        self.device.attach_kernel_driver( 0 )
        log.debug( "Attach kernal driver on attempt {0}".format( attempt ) )
        break
      except usb.USBError:
        attempt += 1
        time.sleep( 0.1 )     # sleep seconds
    if attempt == 10:
      log.error( "Could not attach kernal driver" )

def configLog( ):
  log = logging.getLogger( )
  log.setLevel( logging.DEBUG )
  fileHandler = logging.FileHandler( "terminal.log" )
  log_fmt = logging.Formatter(
      "%(levelname)s %(name)s %(threadName)-10s %(funcName)s() %(message)s" )
  fileHandler.setFormatter( log_fmt )
  log.addHandler( fileHandler )
  return log

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

def configInputQueue(  ):
  """ configure a queue for accepting characters and return the queue
  """
  def captureInput( iqueue ):
    while True:
      c = getch()
      if c == '\x03' or c == '\x04':    # end on ctrl+c / ctrl+d
        log.debug( "Break received (\\x{0:02X})".format( ord( c ) ) )
        iqueue.put( c )
        break
      log.debug( "Input Char '{}' received".format( c if c != '\r' else '\\r' ) )
      iqueue.put( c )

  input_queue = queue.Queue()
  input_thread = threading.Thread( target=lambda : captureInput( input_queue ) )
  input_thread.daemon = True
  input_thread.start()
  return input_queue, input_thread

def fmt_text( text ):
  """ convert characters that aren't printable to hex format
  """
  PRINTABLE_CHAR = set(
            list(range(ord(' '), ord('~') + 1)) + [ ord('\r'), ord( '\n' ) ] )
  newtext = ( "\\x{:02X}".format( c ) if c not in PRINTABLE_CHAR else chr(c) for c in text )
  return "".join( newtext )

def runTerminal( d ):
  log.info( "Beginning a terminal run" )
  p = ComPort( d )
  q, t = configInputQueue()

  while True:
    if p.rxlen:
      print( fmt_text( p.readBytes() ), end="" )

    if not q.empty():
      c = q.get()
      if c == '\x03' or c == '\x04':    # end on ctrl+c / ctrl+d
        print()
        p.disconnect()
        break;
      p.write( c )

if __name__ == '__main__':
  log = configLog()
  getch = _Getch()            # init an instance

  d = selectDevice(  )
  if d is None:
    exit()

  if sys.argv[-1] == '-x':
    p = ComPort( d, start=False )
    p.disconnect()
  else:
    runTerminal( d )

# vim:shiftwidth=2
