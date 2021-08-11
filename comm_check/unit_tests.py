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
 # Developed for DiFActo by Mowito Robotic Systems Pvt. Ltd.                #
 # ************************************************************************ #

import pymcprotocol
import logging
import sys

    
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
    pymc3e = pymcprotocol.Type3E()
    
    # set the IP address for PLC1
    #pymc3e.setaccessopt(commtype="ascii")
    ip_addr = "192.168.0.39"
    port    = 8888

    # Setup Logger
    logger = setup_custom_logger('plc_init_check')
    logger.info('Beginning Interface initialization checks with Mitsubishi Q03UDE PLC')

    # connect to PLC
    logger.info('Connecting to PLC')
    try:
        pymc3e.connect(ip_addr, port, timeout=5.0)
    except OSError:
        pymc3e._is_connected = False

    # List for the registers to be read and wrtitten to
    boolean_registers = ["M1000", "M1001", "M1002", "M1003", "M1004", "M1005", "M1006", "M1007", "M1008", "M1009"]
    real_registers = ["D100", "D101", "D110", "D111"]
    bit_reg = []
    real_reg = []
    bool_val_list = []
    real_val = []

    # Perform read and write operations only if PLC is connected
    if (pymc3e._is_connected == True):
        logger.info('Connected to PLC successfully')
        #print("***** Performing Read and Write Operations on PLC ******")
        logger.info('Performing Read and Write Operations on PLC')
        # Perform write operations on boolean registers on PLC
        for reg in boolean_registers:
            #print("+++ Performing Read/Write on Register : " + reg + " +++")
            logger.info('Performing Read/Write on Register : %s', reg)
            bool_val = False
            success = 0
            bit_reg.append(reg)
            # loop to perform read write on register 5 times
            i = 0
            for i in range(0,5):
                # write to register
                #print("Writing value : "+ str(bool_val) + " to register " + reg)
                if bool_val:
                    bool_val_list.append(1)
                else:
                    bool_val_list.append(0)
                pymc3e.randomwrite_bitunits(bit_devices=bit_reg, values = bool_val_list)
                logger.info('Writing value : %s to register %s', str(bool_val), reg)
                # read register value
                reg_val = pymc3e.batchread_bitunits(headdevice = reg, readsize = 1)
                #print("Value read from register " + reg + "is : " + str(reg_val))
                logger.info('Value read from register %s is : %s', reg, str(reg_val[0]))
                # comparing if write and read register is true
                if (reg_val[0] == bool_val_list[0]):
                    success = success + 1
                # toggle bool_val
                bool_val = not bool_val
                # increment counter i
                i = i + 1
                # remove bool_val
                bool_val_list.pop(0)
            # remove register from list
            bit_reg.remove(reg)

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
            real_reg.append(reg)
            for i in range(0,5):
                write_val = (i + 1)*0.1 + 5
                real_val.append(write_val)
                # write to register
                logger.info('Writing value : %s to register %s', str(write_val), reg)
                #pymc3e.batchwrite_wordunits(headdevice = reg, values = real_val)
                pymc3e.randomwrite(word_devices=[], word_values=[], dword_devices=[reg], dword_values=[real_val])
                # read register value
                _, read_val = pymc3e.randomread(word_devices=[], word_values=[], dword_devices=[reg], dword_values=[real_val])
                #print("Value read from register " + reg + " is : " + str(read_val))
                logger.info('Value read from register %s is : %s', reg, str(read_val[0]))
                # comparing if write and read register is true
                if (read_val[0] == write_val):
                    success = success + 1
                # increment counter i
                i = i + 1
                # remove write_val from real_val list
                real_val.remove(write_val)
            # remove register from list
            real_reg.remove(reg)
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
        print("\nERROR : Cannot perform Read/Write operations on PLC\n")
        logger.error('Unable to perform read/write checks on PLC as PLC is not connected.')
        logger.debug('Please check PC-Router/Switch link OR PLC-router/switch link')
        #print("Exiting PLC diagnostics. Please refer to LOG file for details.")
        logger.info('Exiting PLC interface initialization checks')
        
if __name__ == '__main__':
    main()
