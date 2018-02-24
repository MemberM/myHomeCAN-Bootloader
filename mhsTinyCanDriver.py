#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2009, Rene Maurer <rene@cumparsita.ch>
# Copyright (C) 2009, omnitron.ch/allevents.ch
#
# Ported to Python3 by
# Copyright (C) 2014, Patrick Menschel <menschel.p@posteo.de>
# Copyright (C) 2015, Klaus Demlehner <klaus@mhs-elektronik.de>
# 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see http://www.gnu.org/licenses.
#
# Description
#   This file/module contains the class 'MhsTinyCanDriver' which
#   represents a driver for the MHS Tiny-CAN modules. Look at
#   http://www.tiny-can.de
#   for details.
#
# Requirments
#   The shared library for your MHS CAN module:
#     For Linux: libmhstcan.so
#     For Windows: mhstcan.dll 
#
#   The library/libraries must be present in one of the following
#   locations:
#   - .            (current directory)
#   - ./lib        (lib in current directory)
#   - /usr/local/lib
#   - /usr/lib
#
# ---------------------------------------------------------------------- 
# ChangeLog: 
# 08.12.2013 V0.53 Changed to Python3, P.Menschel (menschel.p@posteo.de)
# 02.01.2014 V0.54 Many Functions redone, Complete Event Handling Implemented, P.Menschel (menschel.p@posteo.de)
#                  Changed Option Handling to Dictionary, XHandling of Status Values to readable Status Info  
# 12.01.2014 V0.55 Read path of the Tiny-CAN API DLL from the windows registry
# 20.12.2014 V0.56 Support 64Bit Windows OS
# 14.05.2015 V0.60 Bug fixes, start EX-API implemantation
# 04.12.2015 V1.00 Bug fixes, multiple devices support added
# ---------------------------------------------------------------------- 
#  DLL/SO Buglist/Issues 
# - RX Event Handling of Indexes set by Filters does fail with more than one filter set, all RX Events are flushed to INDEX 0, so it seems
# - PNP Event CONNECT does not work on linux at all, same Script works on Windows ok
# ---------------------------------------------------------------------- 

VERSION = \
"""
MhsTinyCanDriver V1.00, 04.12.2015 (LGPL)
P.Menschel (menschel.p@posteo.de)
K.Demlehner (klaus@mhs-elektronik.de)
"""
from ctypes import Structure,c_char,c_int,c_uint8,c_int32,c_uint32,c_char_p,c_uint16,c_void_p,pointer,Union,POINTER,string_at,cast,byref
import os
import sys
import time
import uselogging
from utils import OptionDict2CsvString,UpdateOptionDict,CsvString2OptionDict

if sys.platform == "win32":
    from ctypes import WinDLL,WINFUNCTYPE
    if sys.version_info[0] < 3:
        from _winreg import OpenKey,CloseKey,QueryValueEx,HKEY_LOCAL_MACHINE,KEY_READ
    else:
        from winreg import OpenKey,CloseKey,QueryValueEx,HKEY_LOCAL_MACHINE,KEY_READ
else:
    from ctypes import CDLL,CFUNCTYPE  


# locations where we look for shared libraries
key = None;
if sys.platform == "win32":
    REG_TINY_CAN_API = r'Software\Tiny-CAN\API'
    REG_TINY_CAN_API_64 = r'Software\Wow6432Node\Tiny-CAN\API' 
    REG_TINY_CAN_API_PATH_ENTRY = r'PATH'
    try:
        if (sys.maxsize > 2**32):
            key = OpenKey(HKEY_LOCAL_MACHINE, REG_TINY_CAN_API_64, 0, KEY_READ)            
        else:
            key = OpenKey(HKEY_LOCAL_MACHINE, REG_TINY_CAN_API, 0, KEY_READ)
        mhs_api_path_list = QueryValueEx(key, REG_TINY_CAN_API_PATH_ENTRY);    
    except:
        key = None        
    if key:
        
        if (sys.maxsize > 2**32):
            mhs_api_path = mhs_api_path_list[0] + r'\x64';
        else:
            mhs_api_path = mhs_api_path_list[0];
        sharedLibraryLocations = [mhs_api_path, os.curdir]            
        CloseKey(key)
    else:
        sharedLibraryLocations = [os.curdir]    
else:
    sharedLibraryLocations = [os.curdir,
                              os.path.join(os.curdir,'lib'),
                              '/usr/local/lib',
                              '/usr/lib']
    
# CAN Bitrates
CAN_10K_BIT     = 10
CAN_20K_BIT     = 20
CAN_50K_BIT     = 50
CAN_100K_BIT    = 100
CAN_125K_BIT    = 125
CAN_250K_BIT    = 250
CAN_500K_BIT    = 500
CAN_800K_BIT    = 800
CAN_1M_BIT      = 1000

TCAN_ERROR_CODES = {-1:'Driver not initialized',
                    -2:'Called with Invalid Parameters',
                    -3:'Invalid Index',
                    -4:'Invalid CAN Channel',
                    -5:'Common Error',
                    -6:'FIFO Write Error',
                    -7:'Buffer Write Error',
                    -8:'FIFO Read Error',
                    -9:'Buffer Read Error',
                    -10:'Variable not found',
                    -11:'Variable is not readable',
                    -12:'Read Buffer to small for Variable',
                    -13:'Variable is not writable',
                    -14:'Write Buffer to small for Variable',
                    -15:'Below Minimum Value',
                    -16:'Above Maximum Value',
                    -17:'Access Denied',
                    -18:'Invalid CAN Speed',
                    -19:'Invalid Baud Rate',
                    -20:'Variable not assigned',
                    -21:'No Connection to Hardware',
                    -22:'Communication Error with Hardware',
                    -23:'Hardware sends wrong Number of Parameters',
                    -24:'RAM Memory too low',
                    -25:'OS does not provide enough resources',
                    -26:'OS Syscall Error',
                    -27:'Main Thread is busy'}

# Driver Status Modes
DRV_NOT_LOAD             = 0 # Driver not loaded (although impossible)
DRV_STATUS_NOT_INIT      = 1 # Driver not initialized
DRV_STATUS_INIT          = 2 # Driver is inialized
DRV_STATUS_PORT_NOT_OPEN = 3 # Port is not open
DRV_STATUS_PORT_OPEN     = 4 # Port is open
DRV_STATUS_DEVICE_FOUND  = 5 # Device was found / is connected
DRV_STATUS_CAN_OPEN      = 6 # Device is initialized and open
DRV_STATUS_CAN_RUN_TX    = 7 # Can Bus Transmit only (not used!)
DRV_STATUS_CAN_RUN       = 8 # CAN Bus is active
DRIVER_STATUS_MODES = {DRV_NOT_LOAD:'DRV_NOT_LOAD',
                       DRV_STATUS_NOT_INIT:'DRV_STATUS_NOT_INIT',
                       DRV_STATUS_INIT:'DRV_STATUS_INIT',
                       DRV_STATUS_PORT_NOT_OPEN:'DRV_STATUS_PORT_NOT_OPEN',
                       DRV_STATUS_PORT_OPEN:'DRV_STATUS_PORT_OPEN',
                       DRV_STATUS_DEVICE_FOUND:'DRV_STATUS_DEVICE_FOUND',
                       DRV_STATUS_CAN_OPEN:'DRV_STATUS_CAN_OPEN',
                       DRV_STATUS_CAN_RUN_TX:'DRV_STATUS_CAN_RUN_TX',
                       DRV_STATUS_CAN_RUN:'DRV_STATUS_CAN_RUN'}
 
# Fifo Status Modes
FIFO_STATUS_OK          = 0 # FIFO_STATUS_OK
FIFO_STATUS_OVERRUN     = 1 # FIFO STATUS OVERRUN
FIFO_STATUS_INVALID     = 2 # FIFO STATUS INVALID
FIFO_STATUS_Unknown     = 4 # FIFO STATUS Unknown Undocumented
FIFO_STATUS_MODES = {FIFO_STATUS_OK:'FIFO_STATUS_OK',
                     FIFO_STATUS_OVERRUN:'FIFO_STATUS_OVERRUN',
                     FIFO_STATUS_INVALID:'FIFO_STATUS_INVALID',
                     FIFO_STATUS_Unknown:'FIFO_STATUS_Unknown'}

# Can Status Modes
CAN_STATUS_OK           = 0 # Can Status OK
CAN_STATUS_ERROR        = 1 # Can Status ERROR
CAN_STATUS_WARNING      = 2 # Can Status WARNING
CAN_STATUS_PASSIVE      = 3 # Can Status Passive
CAN_STATUS_BUS_OFF      = 4 # Can Status BUS OFF
CAN_STATUS_INVALID      = 5 # Can Status INVALID
CAN_STATUS_MODES = {CAN_STATUS_OK:'CAN_STATUS_OK',
                    CAN_STATUS_ERROR:'CAN_STATUS_ERROR',
                    CAN_STATUS_WARNING :'CAN_STATUS_WARNING',
                    CAN_STATUS_PASSIVE:'CAN_STATUS_PASSIVE',
                    CAN_STATUS_BUS_OFF:'CAN_STATUS_BUS_OFF',
                    CAN_STATUS_INVALID:'CAN_STATUS_INVALID'}

# CAN Bus Modes
OP_CAN_NO_CHANGE        = 0 # No Change 
OP_CAN_START            = 1 # Start CAN
OP_CAN_STOP             = 2 # Stop CAN
OP_CAN_RESET            = 3 # Reset CAN
OP_CAN_LOM              = 4 # Listen Only Mode
OP_CAN_START_NO_RETRANS = 5 # No Auto Retransmission

CAN_BUS_MODES = {OP_CAN_NO_CHANGE:'OP_CAN_NO_CHANGE',
                 OP_CAN_START:'OP_CAN_START',
                 OP_CAN_STOP:'OP_CAN_STOP',
                 OP_CAN_RESET:'OP_CAN_RESET',
                 OP_CAN_LOM:'OP_CAN_LOM',
                 OP_CAN_START_NO_RETRANS:'OP_CAN_START_NO_RETRANS'}


# Command
CAN_CMD_NONE                = 0x0000 # CAN COMMAND NONE
CAN_CMD_RXD_OVERRUN_CLEAR   = 0x0001 # CAN Clear Receive Overrun Error
CAN_CMD_RXD_FIFOS_CLEAR     = 0x0002 # CAN Clear Receive FIFOs
CAN_CMD_TXD_OVERRUN_CLEAR   = 0x0004 # CAN Clear Transmit Overrun Error
CAN_CMD_TXD_FIFOS_CLEAR     = 0x0008 # CAN Clear Transmit FIFOs
CAN_CMD_HW_FILTER_CLEAR     = 0x0010 # CAN Clear HW Filters
CAN_CMD_SW_FILTER_CLEAR     = 0x0020 # CAN Clear SW Filters
CAN_CMD_TXD_BUFFER_CLEAR    = 0x0040 # CAN Clear Transmit Buffers (Interval Messages) 
CAN_CMD_ALL_CLEAR           = 0x0FFF # CAN Clear All Receive/Transmit Errors/FIFOs, SW/HW FILTERS


# SetEvent
EVENT_ENABLE_PNP_CHANGE             = 0x0001 # Enable Plug & Play Event 
EVENT_ENABLE_STATUS_CHANGE          = 0x0002 # Enable CAN Status Change Event
EVENT_ENABLE_RX_FILTER_MESSAGES     = 0x0004 # Enable CAN Receive Event for Filtered Messages
EVENT_ENABLE_RX_MESSAGES            = 0x0008 # Enable CAN Receive Event
EVENT_ENABLE_ALL                    = 0x00FF # Enable all Events
EVENT_DISABLE_PNP_CHANGE            = 0x0100 # Disable Plug & Play Event 
EVENT_DISABLE_STATUS_CHANGE         = 0x0200 # Disable CAN Status Change Event
EVENT_DISABLE_RX_FILTER_MESSAGES    = 0x0400 # Disable CAN Receive Event for Filtered Messages
EVENT_DISABLE_RX_MESSAGES           = 0x0800 # Disable CAN Receive Event
EVENT_DISABLE_ALL                   = 0xFF00 # Disable all Events

# Global Options Dictionary for TCAN API

TCAN_Options = {'CanRxDFifoSize':None,
                'CanTxDFifoSize':None,
                'CanRxDMode':None,
                'CanRxDBufferSize':None,
                'CanCallThread':None,
                'MainThreadPriority':None,
                'CallThreadPriority':None,
                'Hardware':None,
                'CfgFile':None,
                'Section':None,
                'LogFile':None,
                'LogFlags':None,
                'TimeStampMode':None,
                'CanTxAckEnable':None,
                'CanSpeed1':None,
                'CanSpeed1User':None,
                'AutoConnect':None,
                'AutoReopen':None,
                'MinEventSleepTime':None,
                'ExecuteCommandTimeout':None,
                'LowPollIntervall':None,
                'FilterReadIntervall':None,
                'ComDrvType':None,
                'Port':None,
                'ComDeviceName':None,
                'BaudRate':None,
                'VendorId':None,
                'ProductId':None,
                'Snr':None}

TCAN_Keys_CanInitDriver=['CanRxDFifoSize',
                         'CanTxDFifoSize',
                         'CanRxDMode',
                         'CanRxDBufferSize',
                         'CanCallThread',
                         'MainThreadPriority',
                         'CallThreadPriority',
                         'Hardware',
                         'CfgFile',
                         'Section',
                         'LogFile',
                         'LogFlags',
                         'TimeStampMode']
TCAN_Keys_CanDeviceOpen = ['ComDrvType',
                           'Port',
                           'ComDeviceName',
                           'BaudRate',
                           'VendorId',
                           'ProductId',
                           'Snr',
                           'CanTxAckEnable',
                           'CanSpeed1',
                           'CanSpeed1User']
TCAN_Keys_CanSetOption = ['CanTxAckEnable',
                          'CanSpeed1',
                          'CanSpeed1User',
                          'AutoConnect',
                          'AutoReopen',
                          'MinEventSleepTime',
                          'ExecuteCommandTimeout',
                          'LowPollIntervall',
                          'FilterReadIntervall']
# TIndex
class TIndexBits(Structure):
    _fields_ = [
                ('SubIndex',c_uint16),   # 16bit
                ('CanChannel',c_uint8,4),# 4bit
                ('CanDevice',c_uint8,4), # 4bit
                ('RxTxFlag',c_uint8,1),  # 1bit
                ('SoftFlag',c_uint8,1),  # 1bit
                ('Reserved',c_uint8,6)   # 6bit
                ] # 32bits total
    def __init__(self):
        self.SubIndex=0
        self.CanChannel=0
        self.CanDevice=0
        self.RxTxFlag=0
        self.SoftFlag=0
        self.Reserved=0

class TIndex(Union):
    _fields_ = [('IndexBits',TIndexBits),
                ('Uint32',c_uint32)]
    def __init__(self):
        self.Uint32 = 0
 
# TDeviceStatus
class TDeviceStatus(Structure):
    _fields_ = [('DrvStatus', c_int),
                ('CanStatus', c_uint8),
                ('FifoStatus', c_uint8)]
    def __init__(self):
        self.DrvStatus = 0
        self.CanStatus = 0
        self.FifoStatus = 0

# TCANFlagBits
class TCANFlagBits(Structure):
    _fields_ = [('DLC',c_uint8,4),      # 4bit
                ('TxD',c_uint8,1),      # 1bit
                ('Reserved1',c_uint8,1),# 1bit
                ('RTR',c_uint8,1),      # 1bit
                ('EFF',c_uint8,1),      # 1bit
                ('Source',c_uint8),     # 8bit
                ('Reserved2',c_uint16)  # 16bit
                ] # 32bits total    
    def __init__(self):
        self.DLC=0
        self.TxD=0
        self.Reserved1=0
        self.RTR=0
        self.EFF=0
        self.Source=0
        self.Reserved2=0


class TCANFlags(Union):
    _fields_ = [('FlagBits',TCANFlagBits),
                ('Uint32',c_uint32)]    
        
# TCanMsg
class TCanMsg(Structure):
    _fields_ = [('Id', c_uint32),
                ('Flags', TCANFlags),
                ('Data', c_uint8 * 8),
                ('Sec', c_uint32),
                ('USec', c_uint32)]
    def __init__(self):
        self.Id=0
        self.Flags.Uint32=0
        self.Data=0,0,0,0,0,0,0,0
        self.Sec=0
        self.USec=0

class TMsgFilterFlagsBits(Structure):
    _fields_ = [('DLC',c_uint8,4),#4bit
                ('Reserved1',c_uint8,2),#2bit
                ('RTR',c_uint8,1),#1bit
                ('EFF',c_uint8,1),#1bit
                
                ('IdMode',c_uint8,2),#2bit
                ('DlcCheck',c_uint8,1),#1bit
                ('DataCheck',c_uint8,1),#1bit
                ('Reserved2',c_uint8,4),#4bit
                
                ('Reserved3',c_uint8),#8bit
                
                ('Type',c_uint8,4),#4bit
                ('Reserved4',c_uint8,2),#2bit
                ('Mode',c_uint8,1),#1bit
                ('Enable',c_uint8,1)#1bit
                ]#32bits total
    
    def __init__(self):
        self.DLC = 0
        self.Reserved1 = 0
        self.RTR = 0
        self.EFF = 0
        self.IdMode = 0
        self.DlcCheck = 0
        self.DataCheck = 0
        self.Reserved2 = 0
        self.Reserved3 = 0
        self.Type = 0
        self.Reserved4 = 0
        self.Mode = 0
        self.Enable = 0
    
class TMsgFilterFlags(Union):
    _fields_ = [('FlagBits',TMsgFilterFlagsBits),
                ('Uint32',c_uint32)]       

# TMsgFilter
class TMsgFilter(Structure):
    _fields_ = [('Mask', c_uint32),
                ('Code', c_uint32),
                ('Flags', TMsgFilterFlags)]
                
    def __init__(self):
        self.Mask=0
        self.Code=0
        self.Flags.Uint32=0

# --------------------------------------------------------------------
# ------------------ Driver Class ------------------------------------
# --------------------------------------------------------------------
class MhsTinyCanDriver:
    canBitrates = [
               CAN_10K_BIT,
               CAN_20K_BIT,
               CAN_50K_BIT,
               CAN_100K_BIT,
               CAN_125K_BIT,
               CAN_250K_BIT,
               CAN_500K_BIT,
               CAN_800K_BIT,
               CAN_1M_BIT
               ]

    def __init__(self, dll=None, options=None, ex_mode=1):
        """
        Class Constructor
        @param dll: path to dll / shared library
        @param options: dictionary of options to be set
        @return: nothing
        """
        self.logger = uselogging.getLogger()
        self.DefaultIndex = TIndex() #default FIFO Index 0        
        self.Options = TCAN_Options
        self.ExMode = ex_mode
        self.TCDriverProperties = {}
        self.TCDeviceProperties = {} #no multidevice support yet
        if options:
            self.Options.update(options)
        self.so = None
        if dll:
            if sys.platform == "win32":
                self.so = WinDLL(dll)
            else:
                self.so = CDLL(dll)                            
        else:                        
            for sharedLibraryLocation in sharedLibraryLocations:
                try:
                    if sys.platform == "win32":
                        sharedLibrary = sharedLibraryLocation + os.sep + "mhstcan.dll"
                    else:
                        sharedLibrary = sharedLibraryLocation + os.sep + "libmhstcan.so"    
                    self.logger.info('search library {0} in {1}'.format(sharedLibrary, sharedLibraryLocation))
                    if sys.platform == "win32":
                        self.so = WinDLL(sharedLibrary)
                    else:
                        self.so = CDLL(sharedLibrary)                    
                    self.logger.info('library found')
                    break
                except OSError:
                    self.logger.error('no valid library found')
                    pass
        if not self.so:
            raise RuntimeError('library not found: ' + sharedLibrary)                      
        err = self.initDriver(self.Options)
        if ex_mode == 1 and err == 0:
             res = self.CanExCreateDevice(options = b'CanRxDFifoSize=16384')
             if res[0] >= 0:
                 self.DefaultIndex = res[1]
             err = res[0];    
        if err < 0:
            raise NotImplementedError('Device Init Failed')      

    # ----------------------------------------------------------------
    # --------------- Overall CAN Open Function ----------------------
    # ----------------------------------------------------------------
    
    def OpenComplete(self, index = None, options=None, snr=None, canSpeed=None, listenOnly=0):
        """
        High Level Function to Load the DLL/Shared Library and Open the CAN Device and Start Up the CAN Bus
        @param options: Options Dictionary for Driver
        @param snr: Serial Number of Device
        @param can_bitrate: Bitrate / CAN Speed to be set       
        @return: Error Code (0 = No Error)
        """
        if index == None:
            index = self.DefaultIndex                  
        if options and (type(options) == dict):
            UpdateOptionDict(self.Options,options,self.logger)            
        if snr:  
            UpdateOptionDict(self.Options,{'Snr':snr},self.logger)  
        #obtain CAN Speed by prio explicite given parameter >> given option dictionary >> objects option dictionary
        if canSpeed:
            UpdateOptionDict(self.Options,{'CanSpeed1':canSpeed},self.logger)           
        err = self.openDevice(index, options=self.Options)
        if err >= 0:
            err = self.startCanBus(index=index, listenOnly=listenOnly)
        if err >= 0:    
            self.logger.info('initComplete done (with success) for device with snr.: {0}'.format(snr))
        else:
            self.logger.info('init failed for device with snr.: {0}'.format(snr))
            self.CanDeviceClose(index)                             
        return err


    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
    # ----------------------------------------------------------------


    # ----------------------------------------------------------------
    # ------ High Level Functions to interact with the Driver --------
    # ----------------------------------------------------------------

    def initDriver(self, options=None):
        """
        High Level Function to init the Driver - Wrapper Global Dictionary to OptionString  
        @param options: dictionary of options to be set
        @return: Error Code (0 = No Error)
        """        
        if options and (type(options) == dict):
            UpdateOptionDict(self.Options,options,self.logger)            
        OptionString = OptionDict2CsvString(OptionDict=self.Options,Keys=TCAN_Keys_CanInitDriver)
        if self.ExMode == 1:
            err = self.CanExInitDriver(OptionString)
        else:
            err = self.CanInitDriver(OptionString)
        if err < 0:
            self.logger.error('initDriver Error-Code: {0}'.format(err))
            raise RuntimeError('Could not load Driver')
        self.TCDriverProperties.update(CsvString2OptionDict(self.CanDrvInfo()))
        return err

    def openDevice(self, index=None, serial=None, options=None):
        """
        High Level Function to open a device
        @param serial: Serial Number of Device if you wish to override global Options
        @param options: dictionary of options to be set
        @return: Error Code (0 = No Error), index currently used
        @author: Patrick Menschel (menschel.p@posteo.de)
        @todo: Handle Exception if Serial Number is given but device is not found Handle in OPENDEVICE
        """
        if index == None:
            index = self.DefaultIndex
        if options and (type(options) == dict):
            UpdateOptionDict(self.Options,options,self.logger)
        if serial:
            options.update({'snr':serial})
            UpdateOptionDict(self.Options,{'snr':serial},self.logger)                                 
        err = self.CanDeviceClose(index)
        if err < 0:
            self.logger.error('CanDeviceClose prior to CanDeviceOpen Error-Code: {0}'.format(err))          
        err = self.CanDeviceOpen(index, OptionDict2CsvString(OptionDict=options,Keys=TCAN_Keys_CanDeviceOpen))
        if err < 0:
            self.logger.error('openDevice Error-Code: {0}'.format(err))
        self.TCDeviceProperties.update(CsvString2OptionDict(self.CanDrvHwInfo(index)))
        return err
    
    def setOptions(self, options):
        """
        High Level Function to Set CAN Options
        @param options: dictionary of options to be set
        @return: Error Code (0 = No Error)
        """      
        if options and (type(options) == dict):
            UpdateOptionDict(self.Options,options,self.logger)        
        err = self.CanSetOptions(OptionDict2CsvString(OptionDict=options,Keys=TCAN_Keys_CanSetOption))
        if err < 0:
            self.logger.error('setOptions Error-Code: {0}'.format(err))
        return err
   
    def resetCanBus(self, index=None, cleanup=1):
        """
        High Level Function to Reset the CAN Bus
        @param index: Struct commonly used by the Tiny Can API 
        @return: Error Code (0 = No Error)
        @author: Patrick Menschel (menschel.p@posteo.de)
        """
        if index == None:
            index = self.DefaultIndex   
        if cleanup == 1:                 
            err = self.CanSetMode(index, OP_CAN_RESET, CAN_CMD_ALL_CLEAR)
        else:
            err = self.CanSetMode(index, OP_CAN_RESET, 0)            
        return err
    
    def startCanBus(self, index=None, listenOnly=0, cleanup=1):
        """
        High Level Function to Reset the CAN Bus
        @param index: Struct commonly used by the Tiny Can API 
        @return: Error Code (0 = No Error)
        """
        if index == None:
            index = self.DefaultIndex
        if listenOnly == 1:               
            if cleanup == 1:                 
                err = self.CanSetMode(index, OP_CAN_LOM, CAN_CMD_ALL_CLEAR)
            else:
                err = self.CanSetMode(index, OP_CAN_LOM, 0)
        else:
            if cleanup == 1:                 
                err = self.CanSetMode(index, OP_CAN_START, CAN_CMD_ALL_CLEAR)
            else:
                err = self.CanSetMode(index, OP_CAN_START, 0)                    
        return err    
 
    def stopCanBus(self, index=None, cleanup=1):
        """
        High Level Function to Reset the CAN Bus
        @param index: Struct commonly used by the Tiny Can API 
        @return: Error Code (0 = No Error)
        @author: Patrick Menschel (menschel.p@posteo.de)
        """
        if index == None:
            index = self.DefaultIndex   
        if cleanup == 1:                 
            err = self.CanSetMode(index, OP_CAN_STOP, CAN_CMD_ALL_CLEAR)
        else:
            err = self.CanSetMode(index, OP_CAN_STOP, 0)            
        return err
 
    def setCanBusSpeed(self, index=None, canSpeed=None):
        """
        High Level Function to Set the CAN Bus Speed - basically get rid of the index for now
        @param index: Struct commonly used by the Tiny Can API     
        @param canSpeed: Can Bitrate to be set 
        @return: Error Code (0 = No Error)
        @todo: handle custom bitrates here too later
        @author: Patrick Menschel (menschel.p@posteo.de)
        """
        if canSpeed == None:
          raise ValueError('canSpeed is required')
        if index == None:
            index = self.DefaultIndex
        UpdateOptionDict(self.Options,{'canSpeed1':canSpeed},self.logger)
        return self.CanSetSpeed(index, canSpeed)              
        
    # ----------------------------------------------------------------
    # ----- Format and Printing Stuff for debugging purposes ---------
    # ----------------------------------------------------------------
     
    def FormatMessages(self, messages=None):                  
        formatedMessages = []
        if messages:
            for message in messages:
                if message.Flags.FlagBits.RTR and message.Flags.FlagBits.EFF:
                    frame_format = 'EFF/RTR'
                else: 
                    if message.Flags.FlagBits.EFF:
                        frame_format = 'EFF    '
                    else:
                        if message.Flags.FlagBits.RTR:
                            frame_format = 'STD/RTR'
                        else:
                            frame_format = 'STD    '
                if message.Flags.FlagBits.TxD:
                    dir = 'TX'
                else:
                    dir = 'RX'                    
                dlc = message.Flags.FlagBits.DLC;
                msg_data = ""
                if not message.Flags.FlagBits.RTR:                                    
                    for n in range(dlc):
                        msg_data = msg_data + '%02X ' % message.Data[n]    
                formatedMessage = '{0} {1:08x} {2} {3} {4}'.format(dir, message.Id, frame_format, dlc, msg_data)
                formatedMessages.append(formatedMessage)
        return formatedMessages

    def FormatCanDeviceStatus(self, drv, can, fifo):
        """
        Simple Function to cast/format the Device Status to readable text by use of dictionaries
        @param drv: Driver Status
        @param can: Can Status
        @param fifo: Fifo Status 
        @return: A String to be printed for Information 
        @author: Patrick Menschel (menschel.p@posteo.de) 
        """        
        return 'Driver: {0}, CAN: {1}, Fifo: {2}'.format(DRIVER_STATUS_MODES[drv], CAN_STATUS_MODES[can], FIFO_STATUS_MODES[fifo])

    def FormatError(self, err_num=0, func=None):
        if err_num < 0:
            if func:
                return 'Function {0} return with error: ({1}) {2}'.format(func, err_num, TCAN_ERROR_CODES[err_num])
            else:
                return 'Error ({0}) {1}'.format(err_num, TCAN_ERROR_CODES[err_num]) 
        else:
            if func:
                return 'Function {0} return successful'.format(func)
            else:
                return None
        

    # --------------------------------------------------------------------
    # --------------- User Functions --- ---------------------------------
    # --------------------------------------------------------------------
     
    def TransmitData(self, index, msgId, msgData, msgLen=None, rtr = 0, eff = 0):
        """
        High Level Function to transmit a CAN Message
        @param index: Struct commonly used by the Tiny Can API, Drop the Index to select the FIFO      
        @param msgId: CAN ID of the Message
        @param msgData: Data of the Message
        @param rtr: Remote Transmission Request, Note: obsolete in any known CAN Protocol
        @return: Error Code (0 = No Error)    
        @todo: Return Code is 1 but no error  
        @author: Patrick Menschel (menschel.p@posteo.de) 
        """                                                
        Flags = TCANFlags()           
        if type(msgData) != list:
            self.logger.error('List expected but got {0} instead'.format(type(msgData)))
            raise ValueError('List expected but got {0} instead'.format(type(msgData)))
        for i in msgData:
            if type(i) != int:
                self.logger.error('List of integers expected but got {0} in list instead'.format(type(i)))
                raise ValueError('List of integers expected but got {0} in list instead'.format(type(i)))
        if msgLen != None:
            Flags.FlagBits.DLC = msgLen;
        else:
            l = len(msgData);
            if l > 8: 
                raise ValueError('Messages with more then 8 Bytes are not supported')
            else:                         
                Flags.FlagBits.DLC = l          
        Flags.FlagBits.RTR = rtr
        Flags.FlagBits.EFF = eff
        # Flags.FlagBits.TxD = 1 not necessary
        err = self.CanTransmit(index, msgId, msgData, flags=Flags.Uint32) 
        if err < 0:
            self.logger.error('TransmitData Error-Code: {0}'.format(err))              
        return err   
                  
    def SetIntervalMessage(self, index, msgId=None, msgData=None, msgLen=None, rtr=0, eff=0, interval=-1):
        """
        High Level Function to transmit a CAN Message in given Interval
        @param index: Struct commonly used by the Tiny Can API        
        @param msgId: CAN ID of the Message
        @param msgData: Data of the Message
        @param interval: Interval in milliseconds, shared library operates in usec   
        @param rtr: Remote Transmission Request, Note: obsolete in any known CAN Protocol
        @return: Error Code (0 = No Error) 
        @author: Patrick Menschel (menschel.p@posteo.de)    
        """                                          
        err = self.CanTransmitSet(index=index, flags=0x8000, interval=0)
        if err >= 0 and msgId != None and msgData != None:
            err = self.TransmitData(index=index, msgId=msgId, msgData=msgData, msgLen=msgLen, rtr=rtr, eff=eff)
        if err >= 0 and interval > 0:
            err = self.CanTransmitSet(index=index, flags=0x8001, interval=interval)
        if err < 0:
            self.logger.error('SetInvervalMessage Error-Code: {0}'.format(err))
        return err      
        
    
    def SetFilter(self, index=None, msgId=None, msgIdMask=None, msgIdStart=None, msgIdStop=None, msgLen=None, rtr=0, eff=0, remove=1):
        """
        High Level Function to set a CAN Message Filter
        @param index: Struct commonly used by the Tiny Can API        
        @param msgId: CAN ID of the Message
        @param msgMask: BitMasks to bit and with the CAN ID to form the filter
        @param msgLen: Message Length to be filtered for
        @param rtr: Remote Transmission Request, Note: obsolete in any known CAN Protocol
        @return: Error Code (0 = No Error), TIndex under that the Filter was set 
        @author: Patrick Menschel (menschel.p@posteo.de)    
        """
        if index == None:
            raise ValueError('index is required')                   
        filterFlags = TMsgFilterFlags()
        if msgIdMask:
            filterFlags.FlagBits.IdMode = 0   # Mask (msgIdMask) & Code (msgId)
            code = msgId
            mask = msgIdMask
        else:
            if msgIdStart:                
                filterFlags.FlagBits.IdMode = 1   # Start (msgIdStart) & Stop (msgIdStp)
                code = msgIdStart 
                mask = msgIdStop 
            else:
                filterFlags.FlagBits.IdMode = 2   # Id
                code = msgId 
                mask = 0;
        if msgLen:
            filterFlags.FlagBits.DLC = msgLen
            filterFlags.FlagBits.DlcCheck = 1
        else:
            filterFlags.FlagBits.DLC = 0
            filterFlags.FlagBits.DlcCheck = 0
        if remove == 1:    
            filterFlags.FlagBits.Mode = 0
        else:
            filterFlags.FlagBits.Mode = 1  
        filterFlags.FlagBits.RTR = rtr
        filterFlags.FlagBits.EFF = eff
            
        filterFlags.FlagBits.DataCheck = 0 # No Options given                                     
        filterFlags.FlagBits.Type = 0      # No Options known        
        filterFlags.FlagBits.Enable = 1    # Enable by default
        
        err = self.CanSetFilter(index=index, code=code, mask=mask, flags=filterFlags.Uint32)
        if err < 0:
            self.logger.error('FilterSetUp Error-Code: {0}'.format(err))
        return err


    # ----------------------------------------------------------------
    # ---- API CALLS --------------------------- ---------------------
    # ----------------------------------------------------------------

    def CanInitDriver(self, options=None):
        """
        API CALL - Constructor of the shared Library / DLL, Initialize the Tiny CAN API
        @param options: Option String - ByteString in Python3
        @return: Error Code (0 = No Error)
        """
        self.so.CanInitDriver.restype = c_int32
        err = self.so.CanInitDriver(c_char_p(options))
        if err < 0:
            self.logger.error('CanInitDriver Error-Code: {0}'.format(err))
        return err
            
    def CanDownDriver(self):
        """
        API CALL - Destructor of the shared Library / DLL Shutdown the Tiny CAN API
        @return: Nothing
        """
        self.so.CanDownDriver()
        return
    
    def CanSetOptions(self, options = None):
        """
        API CALL - Set Options for the CAN Device
        @param options: Option String - ByteString in Python3
        @return: Error Code (0 = No Error)
        """
        self.so.CanSetOptions.restype = c_int32
        err = self.so.CanSetOptions(c_char_p(options))
        if err < 0:
            self.logger.error('CanSetOptions Error-Code: {0}'.format(err))
        return err
        
    def CanDeviceOpen(self, index, options = None):
        """
        API CALL - Open a Can Device
        @param index: Struct commonly used by the Tiny CAN API
        @param options: Option String - ByteString in Python3
        @return: Error Code (0 = No Error) 
        """
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanDeviceOpen.restype = c_int32    
        err = self.so.CanDeviceOpen(c_uint32(idx), c_char_p(options))
        if err < 0:
            self.logger.error('CanDeviceOpen Error-Code: {0}'.format(err))
        return err
        
    def CanDeviceClose(self, index=None):
        """
        API CALL - Close a CAN Device
        @param index: Struct commonly used by the Tiny Can API
        @return: Error Code (0 = No Error) 
        """
        if not index:
            index = self.DefaultIndex
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanDeviceClose.restype = c_int32    
        err = self.so.CanDeviceClose(c_uint32(idx))
        if err < 0:
            self.logger.error('CanDeviceClose Error-Code: {0}'.format(err))
        return err
        
    def CanSetMode(self, index=None, mode=0, flags=0):
        """
        API CALL - Set CAN Bus Mode of Operation
        @param index: Struct commonly used by the Tiny Can API
        @param mode: Mode of Can Operation, Start, Stop, Listen Only, No Auto Retransmission
        @param flags: Bitmask of flags to clear certain Errors, Filters,... 
        @return: Error Code (0 = No Error) 
        """      
        if not index:
            index = self.DefaultIndex
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanSetMode.restype = c_int32    
        err = self.so.CanSetMode(c_uint32(idx), c_uint8(mode), c_uint16(flags)) 
        if err < 0:
            self.logger.error('CanSetMode Error-Code: {0}'.format(err))
        return err       
                
    def CanTransmit(self, index, msgId, msgData, flags): #rtr=0, eff=0): <*>
        """
        API CALL - Transmit a CAN Message
        @param index: Struct commonly used by the Tiny Can API        
        @param msgId: CAN ID of the Message
        @param msgData: Data of the Message, List of Integers
        @param flags: Flags to be set
        @return: Error Code (0 = No Error)      
        """    
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        canMSG = TCanMsg()
        canMSG.Flags.Uint32 = flags
        canMSG.Id = c_uint32(msgId)
        for i,b in enumerate(msgData):
            canMSG.Data[i] = b
        self.so.CanTransmit.restype = c_int32    
        err = self.so.CanTransmit(c_uint32(idx), pointer(canMSG), c_int(1)) # transmit once
        if err < 0:
            self.logger.error('CanTransmit Error-Code: {0}'.format(err))
        return err
            
    def CanTransmitClear(self, index=None):
        """
        API CALL - Clear the Transmit FIFO
        @param index: Struct commonly used by the Tiny Can API  
        @return: Error Code (0 = No Error)         
        """ 
        if not index:
            index = self.DefaultIndex
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanTransmitClear(c_uint32(idx))
        return
        
    def CanTransmitGetCount(self, index=None):
        """
        API CALL - Get Message Count currently in Transmit FIFO
        @param index: Struct commonly used by the Tiny Can API  
        @return: Number of Messages         
        """
        if not index:
            index = self.DefaultIndex                  
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanTransmitGetCount.restype = c_uint32    
        num = self.so.CanTransmitGetCount(c_uint32(idx))        
        return num
        
    def CanTransmitSet(self, index, flags, interval):
        """
        API CALL - Set the interval time for a CAN transmit message
        @param index: Struct commonly used by the Tiny Can API
        @param flags: BitMask of flags for certain tasks, Bit 0 =  Enable, Bit 15 = apply interval value
        @param interval: Interval in milliseconds, shared library operates in usec   
        @return: Error Code (0 = No Error)
        @todo: Return Code is 1 but no error           
        """
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        usecs = int(interval * 1000)
        self.so.CanTransmitSet.restype = c_int32
        err = self.so.CanTransmitSet(c_uint32(idx), c_uint16(flags), c_uint32(usecs))
        if err < 0:
            self.logger.error('CanTransmitSet Error-Code: {0}'.format(err))
        return err
            
    def CanReceive(self, index=None, count=1):
        """
        API CALL - Read a CAN Message from FIFO or Buffer, depends on index
        @param index: Struct commonly used by the Tiny Can API
        @param count: Number of messages to be read 
        @return: CAN Messages of specified count or Error Code
        """ 
        if not index:
            index = self.DefaultIndex      
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        res = 0    
        self.so.CanReceiveGetCount.restype = c_uint32    
        num = self.so.CanReceiveGetCount(c_uint32(idx))
        if num == 0:    
            TCanMsgArray = None
        else:
            if num < count:
                count = num;           
            TCanMsgArrayType = TCanMsg * count # Struct of multiple TCANMsg Instances without using Python List object
            TCanMsgArray = TCanMsgArrayType()
            self.so.CanReceive.restype = c_int32
            res = self.so.CanReceive(c_uint32(idx), pointer(TCanMsgArray), count)
        if res < 0:
            self.logger.info('CanReceive, Error-Code: {0}'.format(num))
            TCanMsgArray = None  # <*> Speicher freigeben ?          
        return res, TCanMsgArray
        
    def CanReceiveClear(self, index=None):
        """
        API CALL - Clear a CAN Message FIFO or Buffer, depends on index
        @param index: Struct commonly used by the Tiny Can API 
        @return: Error Code (0 = No Error)
        """   
        if not index:
            index = self.DefaultIndex
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index            
        self.so.CanReceiveClear(c_uint32(idx))
        return
        
    def CanReceiveGetCount(self, index=None):
        """
        API CALL - Get Number of CAN Messages in FIFO or Buffer, depends on index
        @param index: Struct commonly used by the Tiny Can API 
        @return: Message Number or Error Code
        """ 
        if not index:
            index = self.DefaultIndex  
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanReceiveGetCount.restype = c_uint32
        num = self.so.CanReceiveGetCount(c_uint32(idx))        
        return num
        
    def CanSetSpeed(self, index, speed):
        """
        API CALL - Set the Bitrate / Speed of the CAN BUS
        @param index: Struct commonly used by the Tiny Can API
        @param speed: Bitrate / speed 10 = 10kBit and so on 
        @return: Error Code (0 = No Error)
        """  
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanSetSpeed.restype = c_int32     
        err = self.so.CanSetSpeed(c_uint32(idx), c_uint16(speed))
        if err < 0:
            self.logger.error('CanSetSpeed Error-Code: {0}'.format(err))        
        return err
        
    def CanSetSpeedUser(self, index, value):
        """
        API CALL - Set the Bitrate / Speed of the CAN BUS
        @param index: Struct commonly used by the Tiny Can API
        @param speed: Bitrate / speed 10 = 10kBit and so on 
        @return: Error Code (0 = No Error)
        """  
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanSetSpeedUser.restype = c_int32    
        err = self.so.CanSetSpeedUser(c_uint32(idx), c_uint32(value))
        if err < 0:
            self.logger.error('CanSetSpeedUser Error-Code: {0}'.format(err))        
        return err        
    
    def CanDrvInfo(self):
        """
        API CALL - Get Driver Information from DLL / Shared Library
        @return: Version String of DLL / Shared Library
        """        
        self.so.CanDrvInfo.restype = c_char_p
        return self.so.CanDrvInfo()
        
    def CanDrvHwInfo(self, index=None):
        """
        API CALL - Get Firmware Information from Hardware Device
        @return: Version String of Hardware Device / Firmware
        """
        if not index:
            index = self.DefaultIndex 
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanDrvHwInfo.restype = c_char_p
        return self.so.CanDrvHwInfo(c_uint32(idx))    
    
    def CanSetFilter(self, index, mask, code, flags):
        """
        API CALL - Set a CAN Filter to a specific Index / SubIndex
        @param index: Struct commonly used by the Tiny Can API
        @param mask: Mask Bits that must apply bit-anded
        @param code: Message ID to be matched
        @param filter_Dlc: Length of Message to be checked for
        @param filter_Rtr: RTR Bit of Message, is there any use in it ?
        @param filter_Eff: Extended Format Flag, for 29bit 11bit comparasm, why isn't that handled automatically?
        @param filter_IdMode: Mode how Mask and Code are handled, leave at 0 for now as it's the most common method
        @param filter_DLCCheck: Flag to activate this Check
        @param filter_DataCheck: Flag to do something, What does this do ?
        @param filter_Mode: If or not Messages are destroyed that don't match the Filter
        @param filter_Enable: On/Off Switch for the Filter
        @return: Error Code (0 = No Error)
        """        
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        canMSGFilter = TMsgFilter()
        canMSGFilter.Mask = mask
        canMSGFilter.Code = code
        canMSGFilter.Flags.Uint32 = flags
        self.so.CanSetFilter.restype = c_int32
        err = self.so.CanSetFilter(c_uint32(idx), pointer(canMSGFilter))
        if err  < 0:
            self.logger.error('CanSetFilter Error-Code: {0}'.format(err))
        return int(err)
                
    def CanGetDeviceStatus(self, index=None):
        """
        API CALL - Get Device Status from Hardware Device / Firmware
        @return: Error Code, Driver Status, Can Status, Fifo Status       
        """
        if not index:
            index = self.DefaultIndex 
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        devSTAT = TDeviceStatus()
        self.so.CanGetDeviceStatus.restype = c_int32
        err = self.so.CanGetDeviceStatus(c_uint32(idx), pointer(devSTAT))
        if err < 0:
            self.logger.error('CanGetDeviceStatus Error-Code: {0}'.format(err))
        return err,devSTAT.DrvStatus,devSTAT.CanStatus,devSTAT.FifoStatus

    # ----------------------------------------------------------------
    # ---------------Event Handling Stuff ----------------------------
    # ----------------------------------------------------------------
    
    def CanSetEvents(self, events = EVENT_ENABLE_ALL):
        """
        Set Event Enable/Disable Bits for the Driver
        @param events: event mask to be set
        @return: Error Code (0 = No Error)       
        """
        err = self.so.CanSetEvents(c_uint16(events))
        if err < 0:
            self.logger.error('CanSetEvents Error-Code: {0}'.format(err))
        return err

    def CanEventStatus(self):
        """
        Check Event Callback Status
        @param events: event mask to be set
        @return: Error Code (0 = No Error)        
        """
        self.so.CanEventStatus.restype = c_uint32
        err = self.so.CanEventStatus()      
        return err

    def CanSetPnPEventCallback(self, CallbackFunc=None):
        """
        API CALL - Set the Callback Event Function for the Plug and Play Event aka Plug In/ Pull Out of Device
        @param CallbackFunc: The Function to call in case of PNP Event
        @requires: The Function specified by Callback must provide the Parameters that the Callback specifies
        @return: Error Code (0 = No Error)        
        @todo: Find out why the Callback on Linux does not work for Device connect but on Windows for Connect and Disconnect even if Driver Option is not set
        """
        if sys.platform == "win32":
            PNPCALLBACKFUNC = WINFUNCTYPE(None, c_uint32, c_uint32) # CallbackFunction Prototype for PNP Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)          
        else:
            PNPCALLBACKFUNC = CFUNCTYPE(None, c_uint32, c_uint32) # CallbackFunction Prototype for PNP Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)            
        self.PNPCallBack = PNPCALLBACKFUNC(CallbackFunc)
        err = self.so.CanSetPnPEventCallback(self.PNPCallBack)
        if err < 0:
            self.logger.error('CanSetPnPEventCallback with Function {0} raised Error Code {1}'.format(CallbackFunc,err))
        return err
        
    def CanSetStatusEventCallback(self, CallbackFunc=None):
        """
        API CALL - Set the Callback Event Function for the Status Event aka Bus Off etc.
        @param CallbackFunc: The Function to call in case of Status Event
        @requires: The Function specified by Callback must provide the Parameters that the Callback specifies
        @return: Error Code (0 = No Error)
        @author: Patrick Menschel (menschel.p@posteo.de)
        """
        if sys.platform == "win32":
            STATUSEVENTCALLBACKFUNC = WINFUNCTYPE(None, c_uint32, POINTER(TDeviceStatus)) # CallbackFunction Prototype for Status Event Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)
        else:
            STATUSEVENTCALLBACKFUNC = CFUNCTYPE(None, c_uint32, POINTER(TDeviceStatus)) # CallbackFunction Prototype for Status Event Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)
        self.StatusEventCallBack = STATUSEVENTCALLBACKFUNC(CallbackFunc)
        err = self.so.CanSetStatusEventCallback(self.StatusEventCallBack)
        if err < 0:
            self.logger.error('CanSetStatusEventCallback with Function {0} raised Error Code {1}'.format(CallbackFunc,err))
        return err           
    
    def CanSetRxEventCallback(self, CallbackFunc=None):
        """
        API CALL - Set the Callback Event Function for the CAN Rx Event
        @param CallbackFunc: The Function to call in case of Can Rx Event
        @requires: The Function specified by Callback must provide the Parameters that the Callback specifies
        @return: Error Code (0 = No Error)
        @author: Patrick Menschel (menschel.p@posteo.de)
        """
        if sys.platform == "win32":
            RXEVENTCALLBACKFUNC = WINFUNCTYPE(None, c_uint32, POINTER(TCanMsg), c_uint32) # CallbackFunction Prototype for Rx Event Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)            
        else:                                 
            RXEVENTCALLBACKFUNC = CFUNCTYPE(None, c_uint32, POINTER(TCanMsg), c_uint32) # CallbackFunction Prototype for Rx Event Callback using CFUNCTYPE(ReturnValue, Argument1, Argument2,...)
            
        self.RxEventCallBack = RXEVENTCALLBACKFUNC(CallbackFunc)
        err = self.so.CanSetRxEventCallback(self.RxEventCallBack)
        if err < 0:
            self.logger.error('CanSetRxEventCallback with Function {0} raised Error Code {1}'.format(CallbackFunc,err))
        return err
  
    def CanSetUpEvents(self, PnPEventCallbackfunc=None, StatusEventCallbackfunc=None, RxEventCallbackfunc=None):
        """
        High Level Function to Set Up Events
        @param PnPEventCallbackfunc: EventCallback in case of Plug and Play Event, e.g. someone has pulled out the cable
        @param StatusEventCallbackfunc: EventCallback in case of CAN Status Change, e.g. someone wrecked the can bus
        @param RxEventCallbackfunc: EventCallback in case of Message Receive, either filtered or not
        @return: Nothing
        """                                   
        err = self.CanSetPnPEventCallback(PnPEventCallbackfunc)
        if err:
            self.logger.error('Error while Setting PNP Callback')        
        err = self.CanSetStatusEventCallback(StatusEventCallbackfunc)
        if err:
            self.logger.error('Error while Setting Status Event Callback')       
        err = self.CanSetRxEventCallback(RxEventCallbackfunc)
        if err:
            self.logger.error('Error while Setting Rx Event Callback')
        err = self.CanSetEvents(EVENT_ENABLE_ALL)
        if err:
            self.logger.error('Error while Enabling Event Callbacks')
        return
    
    # ----------------------------------------------------------------
    # --------------- End of API Calls -------------------------------
    # ----------------------------------------------------------------
    
    def CanExGetDeviceCount(self, flags):
        self.so.CanExGetDeviceCount.restype = c_int32
        err = self.so.CanExGetDeviceCount(c_int32(flags))
        if err < 0:
            self.logger.error('CanExGetDeviceCount Error-Code: {0}'.format(err))
        return err
        
#int32 CanExGetDeviceList(struct TCanDevicesList **devices_list, int flags)
#int32 CanExGetDeviceInfo(uint32_t index, struct TCanDeviceInfo *device_info, struct TCanInfoVar **hw_info, uint32_t *hw_info_size)
#void CanExDataFree(void **data)

    def CanExCreateDevice(self, options = None):
        idx = c_uint32(0)
        self.so.CanExCreateDevice.restype = c_int32
        err = self.so.CanExCreateDevice(byref(idx), c_char_p(options))
        if err < 0:
            self.logger.error('CanExCreateDevice Error-Code: {0}'.format(err))          
        return err, idx.value        
        
    def CanExDestroyDevice(self, index):
        idx = c_uint32(indx)  
        self.so.CanExDestroyDevice.restype = c_int32        
        err = self.so.CanExDestroyDevice(byref(idx))
        if err < 0:
            self.logger.error('CanExDestroyDevice Error-Code: {0}'.format(err))
        return err        
    
    def CanExCreateFifo(self, size, event_obj, event, channels):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index  
        self.so.CanExCreateFifo.restype = c_int32  
        err = self.so.CanExCreateFifo(c_uint32(idx), c_uint32(size), c_void_p(event_obj), c_uint32(event), c_uint32(channels))
        if err < 0:
            self.logger.error('CanExCreateFifo Error-Code: {0}'.format(err))
        return err

    def CanExBindFifo(self, fifo_index, device_index, bind):
        self.so.CanExBindFifo.restype = c_int32
        err = self.so.CanExBindFifo(c_uint32(fifo_index), c_uint32(device_index), c_uint32(bind))
        if err < 0:
            self.logger.error('CanExBindFifo Error-Code: {0}'.format(err))
        return err        

    def CanExCreateEvent(self):
        self.so.CanExCreateEvent.restype = c_void_p    
        event_obj = self.so.CanExCreateEvent()
        return event_obj;

    def CanExSetObjEvent(self, index, source, event_obj, event):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        self.so.CanExSetObjEvent.restype = c_int32 
        self.so.CanExSetObjEvent(c_uint32(idx), c_uint32(source), c_void_p(event_obj), c_uint32(event))
        if err < 0:
            self.logger.error('CanExSetObjEvent Error-Code: {0}'.format(err))
        return err

    def CanExSetEvent(self, event_obj, event):
        self.so.CanExSetEvent(c_void_p(event_obj), c_uint32(event))
        return

    def CanExSetEventAll(self, event):
        self.so.CanExSetEventAll(c_uint32(event))
        return        
        
    def CanExResetEvent(self, event_obj, event):    
        self.so.CanExResetEvent(c_void_p(event_obj), c_uint32(event))
        return
    
    def CanExWaitForEvent(self, event_obj, timeout):   
        self.so.CanExWaitForEvent.restype = c_uint32
        events = CanExWaitForEvent(c_void_p(event_obj), c_uint32(timeout))
        return events

    def CanExInitDriver(self, options = None):
        self.so.CanExInitDriver.restype = c_int32
        err = self.so.CanExInitDriver(c_char_p(options))
        if err < 0:
            self.logger.error('CanExInitDriver Error-Code: {0}'.format(err))
        return err

    def CanExSetOptions(self, index, name, options):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index
        if name == None:
            raise ValueError('name is required')    
        self.so.CanExSetOptions.restype = c_int32
        err = self.so.CanExSetOptions(c_uint32(idx), c_char_p(options))
        if err < 0:
            self.logger.error('CanExSetOptions Error-Code: {0}'.format(err))
        return err

    def CanExSetAsByte(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')    
        self.so.CanExSetAsByte.restype = c_int32
        err = self.so.CanExSetAsByte(c_uint32(idx), c_char_p(name), c_char(value))
        if err < 0:
            self.logger.error('CanExSetAsByte Error-Code: {0}'.format(err))
        return err

    def CanExSetAsWord(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsWord.restype = c_int32
        err = self.so.CanExSetAsWord(c_uint32(idx), c_char_p(name), c_uint8(value))
        if err < 0:
            self.logger.error('CanExSetAsWord Error-Code: {0}'.format(err))
        return err

    def CanExSetAsLong(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsLong.restype = c_int32
        err = self.so.CanExSetAsLong(c_uint32(idx), c_char_p(name), c_int32(value))
        if err < 0:
            self.logger.error('CanExSetAsLong Error-Code: {0}'.format(err))
        return err

    def CanExSetAsUByte(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsUByte.restype = c_int32
        err = self.so.CanExSetAsUByte(c_uint32(idx), c_char_p(name), c_uint8(value))
        if err < 0:
            self.logger.error('CanExSetAsUByte Error-Code: {0}'.format(err))
        return err

    def CanExSetAsUWord(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsUWord.restype = c_int32        
        err = self.so.CanExSetAsUWord(c_uint32(idx), c_char_p(name), c_uint16(value))
        if err < 0:
            self.logger.error('CanExSetAsUWord Error-Code: {0}'.format(err))
        return err
        
    def CanExSetAsULong(self, index, name, value):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsULong.restype = c_int32        
        err = self.so.CanExSetAsULong(c_uint32(idx), c_char_p(name), c_uint32(value))
        if err < 0:
            self.logger.error('CanExSetAsULong Error-Code: {0}'.format(err))
        return err
        
    def CanExSetAsString(self, index, name, value = None):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        self.so.CanExSetAsString.restype = c_int32        
        err = self.so.CanExSetAsString(c_uint32(idx), c_char_p(name), c_char_p(value))
        if err < 0:
            self.logger.error('CanExSetAsString Error-Code: {0}'.format(err))
        return err
        
    def CanExGetAsByte(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')        
        value = c_char(0)        
        self.so.CanExGetAsByte.restype = c_int32                
        err = self.so.CanExGetAsByte(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsByte Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsWord(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')        
        value = c_int16(0)        
        self.so.CanExGetAsWord.restype = c_int32                
        err = self.so.CanExGetAsWord(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsWord Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsLong(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')
        value = c_int32(0)                
        self.so.CanExGetAsLong.restype = c_int32                
        err = self.so.CanExGetAsLong(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsLong Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsUByte(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')
        value = c_uint8(0)                
        self.so.CanExGetAsUByte.restype = c_int32        
        err = self.so.CanExGetAsUByte(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsUByte Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsUWord(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')
        value = c_uint16(0)                
        self.so.CanExGetAsUWord.restype = c_int32        
        err = self.so.CanExGetAsUWord(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsUWord Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsULong(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        value = c_uint32(0)    
        self.so.CanExGetAsULong.restype = c_int32        
        err = self.so.CanExGetAsULong(c_uint32(idx), c_char_p(name), byref(value))
        if err < 0:
            self.logger.error('CanExGetAsULong Error-Code: {0}'.format(err))
        return err,value
        
    def CanExGetAsString(self, index, name):
        if type(index) == TIndex:
            idx = index.Uint32
        else:
            idx = index    
        if name == None:
            raise ValueError('name is required')            
        str = pointer(c_uint8(0))    
        self.so.CanExGetAsString.restype = c_int32        
        err = self.so.CanExGetAsString(c_uint32(idx), c_char_p(name) , byref(str))
        if err < 0:
            self.logger.error('CanExGetAsString Error-Code: {0}'.format(err))
        if not str:
          value = None
        else:
          value = string_at(str)
          c = cast(str, POINTER(c_void_p))
          self.so.CanExDataFree(byref(c))    
        return err,value

    
        
