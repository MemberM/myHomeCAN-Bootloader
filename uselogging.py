#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2009, Rene Maurer <rene@cumparsita.ch>
# Copyright (C) 2009, omnitron.ch/allevents.ch
#
# Time-stamp: <2009-08-29 21:04:01 rene>
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
#     Setup python logging.
#     This module may be used to reduce the complexity of python
#     logging a bit. It uses the root logger only. Note, that
#     there is only one root logger in the system.
#
#     Supported log outputs:
#     - console (stderr)
#     - rotating files (with the possibility to compress old files)
#
# Usage
#     To use logging just add the following lines:
#     >>> import uselogging
#     >>> logger = logging.getLogger()
#     
#     Doing the above gives you the chance to add logging stuff
#     elsewhere in your application code. Note, that any logging
#     outputs are still disabled per default.
#     >>> logger.log(uselogging.APPLIC, 'this an application message')
#     >>> logger.critical('this a critical message')
#     >>> logger.error('this a error message')
#     >>> logger.warning('this a warning message')
#     >>> logger.info('this an info message')
#     >>> logger.debug('this a debug message')
#
#     The following line enables console logging for all levels:
#     >>> uselogging.enableConsoleLogging()
#
#     The following line enables logging for errors and above:
#     >>> uselogging.enableConsoleLogging(level=logging.ERROR)
#
#     If one needs the possibility to enable logging for just
#     one module, a filter has to be setup.
#     >>> logger.addFilter(uselogging.PathFilter(pattern)
#
#     Use a filter to enable logging for '~/app/com/module.py' only:
#     >>>logger.addFiter(uselogging.PathFilter('app.com.module')
#
#     Use a filter to enable logging for all of '~/app/com/':
#     >>>logger.addFiter(logging.PathFilter('app.com.')
#
#     Enable logging to a file
#     >>> logfile = '/home/xxx/log/logfile.log'
#     >>> uselogging.enableFileLogging(logfile, level=logging.DEBUG)
#
#     Enable only application messages (no errors and so one)
#     >>> logfile = '/home/user/application/log/logfile.log'
#     >>> uselogging.enableFileLogging(logfile, level=logging.APPLIC)
#
# ---------------------------------------------------------------------- 

import os
import glob
import time
import zipfile

try:
    import logging
    from logging import handlers
    loggingAvailable = True
except ImportError:
    # If your python environment does not have the logging module
    # (e.g. jython 2.2 comes without logging module) you must write
    # your own logger
    loggingAvailable = False
    print ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print ("Warning: python logging module not available")
    print ("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------

if loggingAvailable:

    # format configurations
    consoleLogFormat = '%(asctime)s %(module)s: %(message)s'
    fileLogFormat = '%(asctime)s %(module)s: %(message)s'
    dateFormat = '%Y-%m-%d %H:%M:%S'

    # level configurations
    APPLIC = logging.CRITICAL + 1
    DISABLE = logging.CRITICAL + 2
    LOWLEVEL = logging.DEBUG - 1
    logging.addLevelName(APPLIC, 'APPLIC')
    logging.addLevelName(DISABLE, 'DISABLE')
    logging.addLevelName(LOWLEVEL, 'LOWLEVEL')

    # default configurations
    defaultLogFileName = 'log.log'
    defaultHistoryLogFiles = 31

    # low level logging configurations
    defaultLowlevelFileLogFormat = '%(asctime)s: %(message)s'
    defaultLowlevelLogFileName = 'lowlevel.log'
    defaultLowlevelRotatingInterval = 30 # minutes
    defaultLowlevelHistoryFiles = 48 # 24h


    # logger
    def getLogger():
        return logging.getLogger()


    # Extended version of TimedRotatingFileHandler that compress logs on rollover
    # TODO: this code is not portable from python 2.5 to python m.n!
    class TimedCompressedRotatingFileHandler(logging.handlers.TimedRotatingFileHandler):

        def doRollover(self):
            # the follwoing code is an 1:1 copy from /usr/lib/python/2.5/loging/handlers.py...
            self.stream.close()
            # get the time that this sequence started at and make it a TimeTuple
            t = self.rolloverAt - self.interval
            timeTuple = time.localtime(t)
            dfn = self.baseFilename + "." + time.strftime(self.suffix, timeTuple)
            if os.path.exists(dfn):
                os.remove(dfn)
            os.rename(self.baseFilename, dfn)
            if self.backupCount > 0:
                # find the oldest log file and delete it
                s = glob.glob(self.baseFilename + ".20*")
                if len(s) > self.backupCount:
                    s.sort()
                    os.remove(s[0])
            #print "%s -> %s" % (self.baseFilename, dfn)
            if self.encoding:
                self.stream = codecs.open(self.baseFilename, 'w', self.encoding)
            else:
                self.stream = open(self.baseFilename, 'w')
            self.rolloverAt = self.rolloverAt + self.interval
            # ...copy of code ends here
            # create the zip file
            if os.path.exists(dfn + ".zip"):
                os.remove(dfn + ".zip")
            file = zipfile.ZipFile(dfn + ".zip", "w")
            file.write(dfn, os.path.basename(dfn), zipfile.ZIP_DEFLATED)
            file.close()
            os.remove(dfn)


    # define a black hole stream
    class NullStream:
        def write(a=None, b=None, c=None): pass
        def flush(a=None, b=None, c=None): pass


    # configure root logger
    logging.basicConfig(level=DISABLE, stream=NullStream())


    # enable console logger
    def enableConsoleLogging(level=logging.DEBUG, format=None):
        if not format: format = consoleLogFormat
        logging.getLogger().setLevel(logging.DEBUG)

        consoleLogger = logging.StreamHandler()
        consoleLogger.setFormatter(logging.Formatter(format, datefmt=dateFormat)) 
        consoleLogger.setLevel(level)
        logging.getLogger().addHandler(consoleLogger)


    #enable rotating file logger
    def enableFileLogging(fileNamePath=defaultLogFileName, level=logging.DEBUG, format=fileLogFormat, compressOld=True):
        if not format: format = fileLogFormat
        logging.getLogger().setLevel(logging.DEBUG)
        if compressOld:
            rotatingFileLogger = TimedCompressedRotatingFileHandler(fileNamePath, when='midnight', interval=1, backupCount=defaultHistoryLogFiles)
        else:
            rotatingFileLogger = handlers.TimedRotatingFileHandler(fileNamePath, when='midnight', interval=1, backupCount=defaultHistoryLogFiles)
        rotatingFileLogger.setFormatter(logging.Formatter(format, datefmt=dateFormat)) 
        rotatingFileLogger.setLevel(level)
        logging.getLogger().addHandler(rotatingFileLogger)


    # enable lowlevel rotating file logger
    def enableLowlevelFileLogging(fileNamePath=defaultLowlevelLogFileName, level=LOWLEVEL, format=defaultLowlevelFileLogFormat, compressOld=True):
        if not format: format = fileLogFormat
        logging.getLogger().setLevel(level)
        if compressOld:
            rotatingFileLogger = TimedCompressedRotatingFileHandler(fileNamePath, when='M', \
                                                                    interval=defaultLowlevelRotatingInterval, backupCount=defaultLowlevelHistoryFiles)
        else:
            rotatingFileLogger = handlers.TimedRotatingFileHandler(fileNamePath, when='M',  \
                                                                   interval=defaultLowlevelRotatingInterval, backupCount=defaultLowlevelHistoryFiles)
        rotatingFileLogger.setFormatter(logging.Formatter(format, datefmt=dateFormat)) 
        rotatingFileLogger.setLevel(level)
        rotatingFileLogger.addFilter(LevelFilter(level))
        logging.getLogger().addHandler(rotatingFileLogger)

    # filter on path with pattern
    class PathFilter(logging.Filter):
        def __init__(self, pattern):
            self.pattern = pattern
        def filter(self, record):
            return (record.pathname.replace(os.sep, '.').find(self.pattern) > -1)

    # filter on exacty one level
    class LevelFilter(logging.Filter):
        def __init__(self, level):
            self.level = level
        def filter(self, record):
            return self.level == record.levelno

# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------

if not loggingAvailable:


    # dumy logger
    # you may add your serious stuff here
    class Logger:
        
        def crtical(self, a):
            print (a)
        def error(self, a):
            print (a)
        def warning(self, a):
            print (a)
        def info(self, a):
            print (a)
        def debug(self, a):
            print (a)
        def crtical(self, a):
            print (a)
        def crtical(self, a):
            print (a)
        def log(self, loglevel, a):
            print (a)


    def getLogger():
        return Logger()
