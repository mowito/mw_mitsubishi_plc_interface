 #!/usr/bin/python3

 # ************************************************************************ #
 # COPYRIGHT MOWITO ROBOTIC SYSTEMS Pvt Ltd.                                #
 # __________________                                                       #
 #                                                                          #
 # NOTICE:  All information contained herein is, and remains                #
 # the property of Mowito Robotic Systems Pvt. Ltd and its suppliers,       #
 # if any.  The intellectual and technical concepts contained               #
 # herein are proprietary to Mowito Robotic Systems Pvt. Ltd                #
 # and its suppliers and are protected by trade secret or copyright law.    #
 # Dissemination of this information or reproduction of this material       #
 # is strictly forbidden unless prior written permission is obtained        #
 # from Mowito Robotic System Pvt. Ltd.                                     #
 # ************************************************************************ #
 
 # ************************************************************************ #
 # This code writes data to a register on the Siemens S7 series PLC         #
 # Author :  Ankur Bodhe (for Mowito Robotic Systems Pvt. Ltd)              #
 # Developed for Ruchagroup by Mowito Robotic Systems Pvt. Ltd.             #
 # ************************************************************************ #

import snap7
from snap7.util import *
import struct
from snap7.snap7types import *
import sys
import rospy

class output(object):
	bool=1
	int=2
	real=3
	word=4
	dword=5

class PLC:
    # define constructor
    def __init__(self):
        self.plc = snap7.client.Client()
        self.ip_address = "0.0.0.0"
        self.connection_status = False
        self.debug = False

    def set_plc_address(self, address):
        self.ip_address = address

    def plc_connection_status(self):
        self.connection_status = self.plc.get_connected()
        return self.connection_status

    def connect_to_plc(self):
        self.plc = snap7.client.Client()
        self.plc.connect(self.ip_address, 0, 1)
        self.connection_status = self.plc.get_connected()
        if(self.plc.get_connected() == False):
            print("ERROR : Unable to connect to PLC")
        else:
            print("Successfully connected to PLC")

    def read_register(self, register, returnByte=False):
        area=0x83
        length=1
        type=0
        out=None
        bit=0
        start=0
        if(register[0].lower()=='m'):
            area=0x83
        if(register[0].lower()=='q'):
            area=0x82
        if(register[0].lower()=='i'):
            area=0x81
        
        if(register[1].lower()=='x'): #bit
            length=1
            out=output().bool
            start = int(register.split('.')[0][2:])
        if(register[1].lower()=='b'): #byte
            length=1
            out=output().int
            start = int(register[2:])
        if(register[1].lower()=='w'): #word
            length=2
            out=output().int
            start = int(register[2:])
        if(register[1].lower()=='d'):
            out=output().dword
            length=4
            if(register.split(".") != -1):
                start = int(register.split('.')[0][2:])
            else:
                start = int(register[2:])
        if('freal' in register.lower()): #double word (real numbers)
            length=4
            start=int(register.lower().replace('freal',''))
            out=output().real
		#print start,hex(area)
        if(output().bool==out):
            bit = int(register.split('.')[1])
        if(self.debug):
            print (register[0].lower(),bit)
        self.plc.read_area(area,0,start,length)
        mbyte=self.plc.read_area(area,0,start,length)
		#print str(mbyte),start,length
        if(returnByte):
            return mbyte
        elif(output().bool==out):
            return get_bool(mbyte,0,bit)
        elif(output().int==out):
            return get_int(mbyte,start)
        elif(output().real==out):
            return get_real(mbyte,0)
        elif(output().dword==out):
            return get_dword(mbyte,0)
        elif(output().word==out):
            return get_int(mbyte,start)

    def write_to_register(self, register, value):
        data=self.read_register(register,True) 
        area=0x83
        length=1
        type=0
        out=None
        bit=0
        start=0
        if(register[0].lower()=='m'):
            area=0x83
        if(register[0].lower()=='q'):
            area=0x82
        if(register[0].lower()=='i'):
            area=0x81
        if(register[1].lower()=='x'): #bit
            length=1
            out=output().bool
            start = int(register.split('.')[0][2:])
            bit = int(register.split('.')[1])
            set_bool(data,0,bit,int(value))
        if(register[1].lower()=='b'): #byte
            length=1
            out=output().int
            start = int(register[2:])
            set_int(data,0,value)
        if(register[1].lower()=='d'):
            out=output().dword
            length=4
            if(register.find(".") != -1):
                start = int(register.split('.')[0][2:])
                set_dword(data,0,value)
            else:
                start = int(register[2:])
                set_dword(data,0,value)
        if(register[1].lower()=='w'):
            out=output().word
            length=4
            if(register.find(".") != -1):
                start = int(register.split('.')[0][2:])
            else:
                start = int(register[2:])
            set_dword(data,0,value)
        if('freal' in register.lower()): #double word (real numbers)
            length=4
            start=int(register.lower().replace('freal',''))
            out=output().real
			#print data
            set_real(data,0,value)
        return self.plc.write_area(area,0,start,data)

    def ReadMemoryBlock(self, area, byte, bit, datatype):
        result = self.plc.read_area(area,0,byte,datatype)
        if datatype==S7WLBit:
            return get_bool(result,0,bit)
        elif datatype==S7WLByte or datatype==S7WLWord:
            return get_int(result,0)
        elif datatype==S7WLReal:
            return get_real(result,0)
        elif datatype==S7WLDWord:
            return get_dword(result,0)
        else:
            return None

    def WriteMemoryBlock(self, area, byte, bit, datatype, value):
        result = self.plc.read_area(area,0,byte,datatype)
        if datatype==S7WLBit:
            set_bool(result,0,bit,value)
        elif datatype==S7WLByte or datatype==S7WLWord:
            set_int(result,0,value)
        elif datatype==S7WLReal:
            set_real(result,0,value)
        elif datatype==S7WLDWord:
            set_dword(result,0,value)
        self.plc.write_area(area,0,byte,result)

    def plc_read(self, register):
        if(register.find(".") != -1):
            return self.read_register(register)
        else:
            if(register[0].lower()=='m'):
                areaM=0x83
            if(register[0].lower()=='q'):
                areaM=0x82
            if(register[0].lower()=='i'):
                areaM=0x81
        if(register[1].lower()=='d'):
            datatype = S7WLDWord
        if(register[1].lower()=='w'):
            datatype = S7WLWord
        addr_str = (register[2:])
        addr = int(addr_str) 
        return self.ReadMemoryBlock(areaM, addr, 0, datatype)

    def plc_write(self, register, value):
        if(register.find(".") != -1):
            self.write_to_register(register, value)
        else:
            if(register[0].lower()=='m'):
                areaM=0x83
            if(register[0].lower()=='q'):
                areaM=0x82
            if(register[0].lower()=='i'):
                areaM=0x81
        if(register[1].lower()=='d'):
            datatype = S7WLDWord
        if(register[1].lower()=='w'):
            datatype = S7WLWord
            addr_str = register[2:]
            addr = int(addr_str)
        self.WriteMemoryBlock(areaM, addr, 0, datatype, value)
