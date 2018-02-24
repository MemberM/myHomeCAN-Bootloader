#!/usr/bin/env python

import threading
import Queue
import time

import uspp.uspp as serial
import message_dispatcher as dispatcher

import os
import sys
import mhsTinyCanDriver as CanDriver
canDriver = CanDriver.MhsTinyCanDriver()
# ------------------------------------------------------------------------------
class CanException(Exception):
	pass

# ------------------------------------------------------------------------------
class Tiny(dispatcher.MessageDispatcher):
	"""Interface for all devices compatible with tinyCan
	
	see http://www.mhs-elektronik.de/
	"""
	def __init__(self, port = None, baud = 9600, debug = False):
		print "init Tiny"
		dispatcher.MessageDispatcher.__init__(self)
		self.__receiverStopEvent = threading.Event()

	def connect(self, port = None, baud = None, debug = None):
		print "connect Tiny"
		self.__receiverStopEvent.clear()
		
		# start the receiver thread
		self.__receiverThread = threading.Thread(target = self.__receive)
		self.__receiverThread.start()
		# TODO replace constants
		err = canDriver.OpenComplete(snr=None, canSpeed=125)
		print(canDriver.FormatError(err, 'OpenComplete'))
		if err:            
		    sys.exit(0)

		# Read status information
		status = canDriver.CanGetDeviceStatus()
		print(canDriver.FormatCanDeviceStatus(status[1], status[2], status[3]))

	def _decode(self, msg):
		#print "decode Tiny"
		#print msg
		#print 'id'
		#print msg.Id
		#print 'data'
		#print msg.Data[0:]
		message = Message( int(msg.Id, 16), msg.Data, msg.Flags.FlagBits.EFF, msg.Flags.FlagBits.RTR)
		return message
	
	#def _encode(self, message):
	#	print "encode Tiny"

	def disconnect(self, port = None, baud = None, debug = None):
		print "disconnect Tiny"
		canDriver.CanDownDriver()
		canDriver.so = None
		# send a stop event
		self.__receiverStopEvent.set()
		
		# wait for the two threads to stop their work
		self.__receiverThread.join()

	def send(self, message):
		#print("< " + str(message.data))
		#print("< " + str(message))
                canDriver.TransmitData(0, msgId = message.id, msgData = message.data)
                #canDriver.TransmitData(0, msgId = message.id, msgData = message.data[:0])

	def __receive(self):
		"""Receiver Thread
		
		Try to read and decode messages from the can bus.
		"""
		print "receive"
		while not self.__receiverStopEvent.isSet():
			#try:
				res = canDriver.CanReceive(count = 500)
				if res[0] > 0:
					msg = res[1][0]
					#print "RX"
					mssage = Message( msg.Id, msg.Data[0:], msg.Flags.FlagBits.EFF, msg.Flags.FlagBits.RTR)
					#message = Message( int(msg.Id, 16), msg.Data[0:], msg.Flags.FlagBits.EFF, msg.Flags.FlagBits.RTR)
					#msgs = canDriver.FormatMessages(res[1])
					#for msg in msgs:
					#	print(msg)
					self._processMessage(mssage)
				else:
					if res[0] < 0:
						print(canDriver.FormatError(res, 'CanReceive'))

				#print res
				#if res[0] > 0: # [0] = count of received msgs
				#	#print('RXraw: ' + str(res[1][0].Data[0:]))
				#	print('RXraw: ' + str(res[1].Data[0:]))
				#        #msg = self._decode(res[1][0])
				#        msg = self._decode(res[1][0].Data[0:])
				#	#msgs = canDriver.FormatMessages(res[1])
				##for msg in msgs:
				#	#print("formatted:")
			#except:
			#	self.__receiverStopEvent.wait(0.001)

# ------------------------------------------------------------------------------
class Message:
	"""Class representation of a CAN message"""
	
	def __init__(self, id, data=[], extended=True, rtr=False, timestamp=None):
		self.id = id
		self.extended = extended
		self.rtr = rtr
		self.data = data
		
		self.timestamp = timestamp
	
	def __str__(self):
		"""Create a string representation of the message"""
		buf = []
		if self.timestamp != None:
			buf.append("[%6.1f] " % (self.timestamp / 10.0))
		
		if self.extended:
			buf.append("id: %08x dlc: %d >" % (self.id, len(self.data)))
		else:
			buf.append("id: %8x dlc: %d >" % (self.id, len(self.data)))
		
		if self.rtr:
			buf.append("rtr")
		else:
			for data in self.data:
				buf.append("%02x" % data)
		
		return " ".join(buf)


# ------------------------------------------------------------------------------
class SerialInterface:
	"""Abstract base class for a CAN interface.
	
	Uses two threads, one to receive from and the other to transmit messages 
	to the serial port.
	"""
	MESSAGE = 0
	RAW = 1
	
	def __init__(self, port = None, baud = 9600, debug = False):
		
		self.port = port
		self.baudrate = baud
		self.debugFlag = debug
		
		self.isConnected = False
		
		self.__receiverStopEvent = threading.Event()
		self.__receiveQueue = Queue.Queue()
		
		self._buf = []
	
	def __del__(self):
		self.disconnect()
	
	
	def send(self, message):
		"""Send a message"""
		time.sleep(.001)
		self._interface.write(self._encode(message))
	
	def get(self, block = True, timeout = None):
		"""Get the last received message
		
		Wait for a new message if there is no one in the queue and "block"
		is set True."""	
		return self.__receiveQueue.get(block, timeout)
	
	
	def connect(self, port = None, baud = None, debug = None):
		"""Connect to a serial Port"""
		
		# close an existing connection (if there is any)
		if self.isConnected:
			self.disconnect()
		
		self.port = port if port else self.port
		self.baudrate = baud if baud else self.baudrate
		self.debugFlag = debug if debug else self.debugFlag
		
		# open serial port
		try:
			self._interface = serial.SerialPort(self.port, timeout = 200, speed = self.baudrate)
			self._interface.flush()
		except serial.SerialPortException:
			raise CanException("could not connect to %s" % self.port)
		
		self.__receiverStopEvent.clear()
		
		# start the receiver thread
		self.__receiverThread = threading.Thread(target = self.__receive)
		self.__receiverThread.start()
		
		self.isConnected = True
	
	def disconnect(self):
		"""Disconnect from the serial port"""
		if not self.isConnected:
			return
		
		# send a stop event
		self.__receiverStopEvent.set()
		
		# wait for the two threads to stop their work
		self.__receiverThread.join()
		
		# close serial port
		try:
			del self._interface
		except serial.SerialPortException, e:
			raise CanException(e)
		
		self.isConnected = False
	
	
	def _debug(self, text):
		if self.debugFlag:
			print text
	
	def _sendRaw(self, data):
		self._interface.write(data)
	
	def _decode(self, chr):
		"""Collects and decodes messages"""
		pass
	
	def _encode(self, message):
		"""Transform a CAN message to a byte stream"""
		pass
	
	
	def __receive(self):
		"""Receiver Thread
		
		Try to read and decode messages from the serial port.
		"""
		while not self.__receiverStopEvent.isSet():
			try:
				tmp = self._interface.read()
				print "RX"
				print str(tmp)
				msg = self._decode(tmp)
				#print str(msg)
				if msg:
					#self.__receiveQueue.put(msg)
					self._processMessage(msg)
			except serial.SerialPortException:
				self.__receiverStopEvent.wait(0.001)

# ------------------------------------------------------------------------------
class Usb2Can(SerialInterface, dispatcher.MessageDispatcher):
	"""Interface for all devices compatible with the CAN232 from Lawicel
	
	see http://www.can232.com/can232.pdf for further information
	"""
	def __init__(self, port = None, baud = 9600, debug = False):
		SerialInterface.__init__(self, port, baud, debug)
		dispatcher.MessageDispatcher.__init__(self)
	
	def connect(self, port = None, baud = None, debug = None):
		SerialInterface.connect(self, port, baud, debug)
		
		# set bitrate and open the channel
		self._sendRaw("S4\r")
		#self._sendRaw("S6\r")
		self._sendRaw("O\r")
	
	def _decode(self, chr):
		if chr != '\r':
			self._buf.append(chr)
			return None
		else:
			data = ''.join(self._buf)
			self._buf = []
			
			if not data:
				return None
			
			type = data[0]
			if type == 'T':
				# extended frame
				data2 = data[10:]
				message_data = []
				
				for x in range(0, len(data2), 2):
					message_data.append(int(data2[x:x + 2], 16))
				
				message = Message( int(data[1:9], 16), message_data, extended = True, rtr = False )
			
			elif type == 't':
				data2 = data[5:]
				message_data = []
				for x in range(0, len(data2), 2):
					message_data.append(int(data2[x:x + 2], 16))
				
				message = Message( int(data[1:4], 16), message_data, extended = False, rtr = False )
			
			elif type == 'R':
				message = Message( int(data[1:9], 16), [None] * int(data[9]), extended = True, rtr = True )
			
			elif type == 'r':
				message = Message( int(data[1:4], 16), [None] * int(data[4]), extended = False, rtr = True )
			
			else:
				# all other frame-types are not supported
				return None

			#only dump frames which pass the acceptance filter. This is done in Bootloader._get_message()		
			#self._debug("> " + str(message))
			
			return message
	
	def _encode(self, message):
		buf = []
		
		self._debug("< " + str(message))
		
		if message.rtr:
			if message.extended:
				buf.append("R%08x%01x" % (message.id, len(message.data)))
			else:
				buf.append("r%03x%01x" % (message.id, len(message.data)))
		else:
			if message.extended:
				buf.append("T%08x%01x" % (message.id, len(message.data)))
			else:
				buf.append("t%03x%01x" % (message.id, len(message.data)))
			
			for data in message.data:
				buf.append("%02x" % data)
		
		buf.append("\r")
		return ''.join(buf)

# ------------------------------------------------------------------------------
class CanDebugger(SerialInterface, dispatcher.MessageDispatcher):
	"""Interface to the Command Shell from the CAN Debugger"""

	def __init__(self, port, baud = 9600, debug = False):
		SerialInterface.__init__(self, port, baud, debug)
		dispatcher.MessageDispatcher.__init__(self)
		
		import re
		self.regularExpression = re.compile("^[$ ]*(?P<timestamp>\d+):[ ]+(?P<id>\w+)[ ](?P<len>\d)(( rtr)|(( >)?(?P<data>( \w\w)*)))", re.IGNORECASE)
	
	def connect(self, port = None, baud = None, debug = None):
		SerialInterface.connect(self, port, baud, debug)
		
		# set filter to receive all messages
		self._sendRaw("set filter 0 0 0\r")
		self._sendRaw("set filter 1 0 0\r")
		self._sendRaw("set filter 2 0 0\r")
		self._sendRaw("set filter 3 0 0\r")
	
	def _decode(self, chr):
		if chr != '\n':
			self._buf.append(chr)
		else:
			result = self.regularExpression.match(''.join(self._buf))
			self._buf = []
			
			if result:
				dict = result.groupdict()
				
				data = dict['data']
				if data:
					msg_data = []
					while data:
						msg_data.append(int(data[1:3], 16))
						data = data[3:]
					rtr = False
				else:
					msg_data = [None] * int(dict['len'])
					rtr = True
				
				id = int(dict['id'], 16)
				extended = True if len(dict['id']) > 3 else False
				
				timestamp = int(dict['timestamp'], 10)
				
				# create message
				message = Message(id, msg_data, extended = extended, rtr = rtr, timestamp = timestamp)
				
				self._debug("> " + str(message))
				
				return message
		
		return None
	
	def _encode(self, message):
		buf = ["> "]
		
		self._debug("< " + str(message))
		
		if message.extended:
			buf.append("%04x %d" % (message.id, len(message.data)))
		else:
			buf.append("%x %d" % (message.id, len(message.data)))
		
		if message.rtr:
			buf.append(" rtr\r")
		else:
			buf.append(" ")
			for byte in message.data:
				buf.append("%02x" % byte)
		
		buf.append("\r")
		return ''.join(buf)

# ------------------------------------------------------------------------------
class DebugInterface(SerialInterface, dispatcher.MessageDispatcher):
	"""Prints every message without sending it to a serial interface"""
	
	def __init__(self, port = None, baud = 9600):
		dispatcher.MessageDispatcher.__init__(self, None)
	
	def connect(self):
		pass
	
	def disconnect(self):
		pass
	
	def send(self, message):
		print message
	
	def sendRaw(self, data):
		pass
	
	def get(self, block, timeout):
		while 1:
			pass
