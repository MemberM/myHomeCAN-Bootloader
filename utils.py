#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2014, Patrick Menschel <menschel.p@posteo.de>
# File: utils.py
# Time-stamp: 
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
#     Some more or less stupid simple functions to cope with option handling
#     of the Tiny CAN Driver, extends mhsTinyCanDriver.py somehow
#
# ---------------------------------------------------------------------- 

def OptionDict2CsvString(OptionDict = {},Keys = []):
    """
    Turn a Dictionary of Key:Value Tupples into a csv like string
    @param OptionDict: dictionary of Key:Value Tupples
    @param Keys: Keys to Use for the OptionString
    @return: String of format Key1=Value1;Key2=Value2,... or NONE if OptionDict was empty
    """
    OptStringArray = []
    if OptionDict:
        for op in OptionDict:
            if OptionDict[op] and (op in Keys):
                OptStringArray.append('{0}={1}'.format(op,OptionDict[op]))
        OptionString = ';'.join(OptStringArray)
#         print(OptionString)
        return OptionString.encode()
    else:
        return None
        

def UpdateOptionDict(OptionDict, Option2UpdateDict,logger):
    """
    Update an Option Dictionary and log every Change using a logger
    @param OptionDict: Option Dictionary to be updated
    @param Option2UpdateDict: Option Dictionary to update the OptionDict
    @param logger: a logger object to log the Changes
    @return: Nothing
    """    
    for option in Option2UpdateDict:
        if option in OptionDict:
            logger.info('Changed Option {0} from {1} to {2}'.format(option,OptionDict[option],Option2UpdateDict[option]))
    OptionDict.update(Option2UpdateDict)
    return

def String2Type(StrUKnwTp):
    if ('.' in StrUKnwTp) and (len(StrUKnwTp.split('.'))==2):#isfloat
        return float(StrUKnwTp)
    elif StrUKnwTp.isnumeric() or StrUKnwTp.isdecimal() or StrUKnwTp.isdigit():#isinteger
        return int(StrUKnwTp)                     
    elif StrUKnwTp.isalnum() and (StrUKnwTp.startswith('0x') or StrUKnwTp.startswith('0X')):#ishexadecimal
        return int(StrUKnwTp[2:],16)
    else:
        return StrUKnwTp#isstring    


def CsvString2OptionDict(CsvString = b''):
    """
    Update an Option Dictionary from a CsvString
    @param CsvString: String by format Key1=Value1;Key2=Value2,... Python3 bytestring!!
    @return: OptionDictionary
    """
    OptionDict = {}
    if CsvString is None: 
        return OptionDict   
    keyValuePairs = CsvString.decode().split(';')
    for kvp in keyValuePairs:
        if kvp:
            key,value = kvp.split('=')
            if ',' in value:
                uvalues = [String2Type(val.strip()) for val in value.split(',')]
            else:
                uvalues = String2Type(value)
            OptionDict.update({key:uvalues})
    return OptionDict
        