#!/usr/bin/env python3
"""
SERIAL STATE notifications ( 2 bytes, interrupt endpoint )
  15..7 - reserved
  6   bOverRun    Received data has been discarded due to a device overrun
  5   bParity     A parity error has occurred
  4   bFraming    A framing error has occurred
  3   bRingSignal State of the ring indicator (RI)
  2   bBreak      Break state
  1   bTxCarrier  State of the data set ready (DSR)
  0   bRxCarrier  State of carrier detect (CD)

Line Coding Data Field ( 7 bytes, control endpoint )
  offset field       (bytes) Description
  --------------------------------------------------------------------
  0      dwDTERate   4       bit rate (bits per second)
  4      bCharFormat 1       stop bits ( 0 : 1bit, 1, 1.5bits, 2, 2bits)
  5      bParityType 1       0:None, 1:Odd, 2:Even, 3:Mark, 4:Space
  6      bDataBits   1       5, 6, 7, 8, 16

Control Line State Field ( 2 bytes, control endpoint )
  wValueBit  Description     (2 bytes data)
  ---------------------------
  bit 1 = 0  RTS : de-assert ( negative voltage )
  bit 1 = 1  RTS : assert    ( positive voltage )
  bit 0 = 0  DTR : de-assert ( negative voltage )
  bit 0 = 1  DTR : assert    ( positive voltage )
"""

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

CDC_SEND_ENCAPSULATED_COMMAND=0x00
CDC_GET_ENCAPSULATED_RESPONSE=0x01
CDC_SET_COMM_FEATURE=0x02
CDC_GET_COMM_FEATURE=0x03
CDC_CLEAR_COMM_FEATURE=0x04
CDC_SET_LINE_CODING=0x20
CDC_GET_LINE_CODING=0x21
CDC_SET_CONTROL_LINE_STATE=0x22
CDC_SEND_BREAK=0x23     # wValue is break time


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
    self.baudrate = 9600
    self.parity = 0
    self.stopbits = 1
    self.databits = 8

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

  def setControlLineState( self, RTS=False, DTR=False ):
    ctrlstate = [ ( 2 if RTS else 0 ) + ( 1 if DTR else 0 ), 0 ]

    txdir = 0           # 0:OUT, 1:IN
    req_type = 1        # 0:std, 1:class, 2:vendor
    recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
    req_type = (txdir<<7) + (req_type<<5) + recipient

    wlen = self.device.ctrl_transfer( req_type, CDC_SET_CONTROL_LINE_STATE,
        data_or_wLength=ctrlstate )
    log.debug( "Linecoding set, {}b sent".format( wlen ) )

  def setLineCoding( self, baudrate=None, parity=None, databits=None, stopbits=None ):
    sbits = { 1:0, 1.5:1, 2:2 }
    dbits = { 5, 6, 7, 8, 16 }
    pmodes = { 0, 1, 2, 3, 4 }
    brates = { 300, 1200, 2400, 4800, 9600, 14400,
        19200, 28800, 38400, 57600, 115200, 230400 }

    if stopbits is not None:
      if stopbits not in sbits.keys():
        valid = ", ".join( str( k ) for k in sorted( sbits.keys() ) )
        raise ValueError( "Valid stopbits are " + valid )
      self.stopbits = stopbits

    if databits is not None:
      if databits not in dbits:
        valid = ", ".join( str( d ) for d in sorted( dbits ) )
        raise ValueError( "Valid databits are " + valid )
      self.databits = databits

    if parity is not None:
      if parity not in pmodes:
        valid = ", ".join( str( pm ) for pm in sorted( pmodes ) )
        raise ValueError( "Valid parity modes are " + valid )
      self.parity = parity

    if baudrate is not None:
      if baudrate not in brates:
        brs = sorted( brates )
        dif = [ abs( br - baudrate ) for br in brs ]
        best = brs[ dif.index( min( dif ) ) ]
        raise ValueError( "Invalid baudrates, nearest valid is {}".format( best ) )
      self.baudrate = baudrate

    linecode = [ self.baudrate & 0xff, ( self.baudrate >> 8 ) & 0xff,
        ( self.baudrate >> 16 ) & 0xff, ( self.baudrate >> 24 ) & 0xff,
        sbits[ self.stopbits ], self.parity, self.databits ]

    txdir = 0           # 0:OUT, 1:IN
    req_type = 1        # 0:std, 1:class, 2:vendor
    recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
    req_type = (txdir<<7) + (req_type<<5) + recipient

    wlen = self.device.ctrl_transfer( req_type, CDC_SET_LINE_CODING,
        data_or_wLength=linecode )
    log.debug( "Linecoding set, {}b sent".format( wlen ) )

  def getLineCoding( self ):
    txdir = 1           # 0:OUT, 1:IN
    req_type = 1        # 0:std, 1:class, 2:vendor
    recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
    req_type = (txdir<<7) + (req_type<<5) + recipient

    buf = self.device.ctrl_transfer( req_type, CDC_GET_LINE_CODING,
        data_or_wLength=7 )
    self.baudrate = buf[0] + ( buf[1] << 8 ) + ( buf[2] << 16 ) + ( buf[3] << 24 )
    self.stopbits = 1 + ( buf[4] / 2.0 )
    self.parity = buf[5]
    self.databits = buf[6]
    print( "LINE CODING:" )
    print( "  {0} baud, parity mode {1}".format( self.baudrate, self.parity ) )
    print( "  {0} data bits, {1} stop bits".format( self.databits, self.stopbits ) )

  def _read( self ):
    """ check ep for data, add it to queue and sleep for interval """
    while self._rxactive:
      try:
        rv = self.ep_in.read( self.ep_in.wMaxPacketSize )
        if self._isFTDI:
          status = rv[:2]       # FTDI prepends 2 flow control characters,
                                # modem status and line status of the UART
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
  elif sys.argv[-1] == '-d':
    p = ComPort( d, start=False )
  else:
    runTerminal( d )

# vim:shiftwidth=2
