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
import logging
import sys

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
    
def setup_custom_logger(name):
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s',
                                datefmt='%Y-%m-%d %H:%M:%S')
    handler = logging.FileHandler('plc_init_checks.log', mode='w')
    handler.setFormatter(formatter)
    screen_handler = logging.StreamHandler(stream=sys.stdout)
    screen_handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.addHandler(screen_handler)
    return logger

# Driver function
def main():
    # create an object of type PLC
    PLC1 = PLC()
    
    # set the IP address for PLC1
    PLC1.set_plc_address("192.168.0.123")

    # Setup Logger
    logger = setup_custom_logger('plc_init_check')
    logger.info('Beginning Interface initialization checks with Siemens S7 PLC')

    # connect to PLC
    logger.info('Connecting to PLC')
    try:
        PLC1.connect_to_plc()
    except snap7.snap7exceptions.Snap7Exception:
        logger.info('Connecting to PLC Failed... exiting')
    # List for the registers to be read and wrtitten to
    boolean_registers = ["QX0.1", "QX0.2", "IX0.0", "IX0.1", "IX0.2", "IX0.3", "IX0.4"]
    real_registers    = ["MD6.0", "MD10", "MD14", "MD18"]

    # Write to 
    # Perform read and write operations only if PLC is connected
    if (PLC1.plc_connection_status() == True):
        logger.info('Connected to PLC successfully')
        #print("***** Performing Read and Write Operations on PLC ******")
        logger.info('Performing Read and Write Operations on PLC')
        # Perform write operations on boolean registers on PLC
        for reg in boolean_registers:
            #print("+++ Performing Read/Write on Register : " + reg + " +++")
            logger.info('Performing Read/Write on Register : %s', reg)
            bool_val = True
            success = 0
            # loop to perform read write on register 5 times
            i = 0
            for i in range(0,5):
                # write to register
                if (reg[0] =='I'):
                    print("Tests on 'I' registers require user to set input on TIA STEP7 portal")
                    print("Set value : "+ str(bool_val) + " to register " + reg)
                    raw_input("Press ENTER when the above instruction is carried out")
                else:
                    #print("Writing value : "+ str(bool_val) + " to register " + reg)
                    PLC1.plc_write(reg, bool_val)
                logger.info('Writing value : %s to register %s', str(bool_val), reg)
                # read register value
                reg_val = PLC1.plc_read(reg)
                #print("Value read from register " + reg + "is : " + str(reg_val))
                logger.info('Value read from register %s is : %s', reg, str(reg_val))
                # comparing if write and read register is true
                if (reg_val == bool_val):
                    success = success + 1
                # toggle bool_val
                bool_val = not bool_val
                # increment counter i
                i = i + 1
            # print success rate of read/write on register and write to log file
            #print("Read/Write on register " + reg + "completed with success rate : " + str(success*20) + "%")
            logger.info('Read/Write on register %s completed with success rate : %s', reg, str(success*20))
            # printing Register read/write test status
            if((success*20)==100):
                #print("Read/Write on Register : " + reg + " Test.... " + "[PASSED]")
                logger.info('Read/Write on Register : %s Test....[PASSED]', reg)
            else:
                #print("Read/Write on Register : " + reg + " Test.... " + "[FAILED]")
                logger.info('Read/Write on Register : %s Test....[FAILED]', reg)
                #print("Please check log file for potential issues/solutions")
                logger.error('Incoherency in read/write on register %s. Potential issues could be issues on port on PLC. Please use PLC software to debug', reg)

        # Perform write operations on real valued registers on PLC
        for reg in real_registers:
            #print("+++ Performing Read/Write on Register : " + reg + " +++")
            logger.info('Performing Read/Write on Register : %s', reg)
            success = 0
            i = 0
            for i in range(0,5):
                write_val = i + 1
                # write to register
                #print("Writing value : "+ str(write_val) + " to register " + reg)
                logger.info('Writing value : %s to register %s', str(write_val), reg)
                PLC1.plc_write(reg, write_val)
                # read register value
                read_val = PLC1.plc_read(reg)
                #print("Value read from register " + reg + " is : " + str(read_val))
                logger.info('Value read from register %s is : %s', reg, str(read_val))
                # comparing if write and read register is true
                if (read_val == write_val):
                    success = success + 1
                # increment counter i
                i = i + 1
            # print success rate of read/write on register and write to log file
            print("Read/Write on register " + reg + "completed with success rate : " + str(success*20) + "%")
            logger.info('Read/Write on register %s completed with success rate : %s', reg, str(success*20))
            # printing Register read/write test status
            if((success*20)==100):
                #print("Read/Write on Register : " + reg + " Test.... " + "[PASSED]")
                logger.info('Read/Write on Register : %s Test....[PASSED]', reg) 
            else:
                print("Read/Write on Register : " + reg + " Test.... " + "[FAILED]")
                logger.info('Read/Write on Register : %s Test....[FAILED]', reg)
                #print("Please check log file for potential issues/solutions")
                logger.error('Incoherency in read/write on register %s. Potential issues could be issues on port on PLC. Please use PLC software to debug', reg)

        # Print a statement to indicate initialization check status
        print (" ======== PLC INTERFACE TESTS COMPLETE ========")
        logger.info('Completed PLC Interface checks')
        print ("Please check log file for details related to the checks")
    else:
        logger.error('Unable to perform read/write checks on PLC as PLC is not connected.')
        logger.debug('Please check PC-Router/Switch link OR PLC-router/switch link')
        logger.info('Exiting PLC interface initialization checks')
        

if __name__ == '__main__':
    main()
