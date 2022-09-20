import serial, time, sys, threading
from arbotix_python.ax12 import *
from PyQt5 import QtCore,QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from controller import Ui_Form
import datetime
import serial.tools.list_ports
import pandas as pd
import numpy as np

class ArbotiXException(Exception):
    pass

class ArbotiX:

    ## @brief Constructs an ArbotiX instance, optionally opening the serial connection.
    ##
    ## @param port The name of the serial port to open.
    ## 
    ## @param baud The baud rate to run the port at. 
    ##
    ## @param timeout The timeout to use for the port. When operating over a wireless link, you may need to
    ## increase this.
    ##
    ## @param open Whether to open immediately the serial port.
    def __init__(self, port="/dev/ttyACM0", baud=115200, timeout=0.1, open_port=True):
        self._mutex = threading._allocate_lock()
        self._ser = serial.Serial()
        
        self._ser.port = port
        self._ser.baudrate = baud
        self._ser.timeout = timeout

        if open_port:
            self._ser.open()

        ## The last error level read back
        self.error = 0

    def __write__(self, msg):
        try:
            self._ser.write(msg)
        except serial.SerialException as e:
            self._mutex.release()
            raise ArbotiXException(e)

    def openPort(self):
        self._ser.close()
        try:
            self._ser.open()
        except serial.SerialException as e:
            raise ArbotiXException(e)

    def closePort(self):
        self._ser.close()

    ## @brief Read a dynamixel return packet in an iterative attempt.
    ##
    ## @param mode This should be 0 to start reading packet. 
    ##
    ## @return The error level returned by the device. 
    def getPacket(self, mode, id=-1, leng=-1, error=-1, params = None):
        try:
            d = self._ser.read()
        except Exception as e:
            print(e)
            return None
        # need a positive byte
        if not d or d == '':
            return None

        # now process our byte
        if mode == 0:           # get our first 0xFF
            if d == b'\xff':   
                return self.getPacket(1)
            else:
                return self.getPacket(0)
        elif mode == 1:         # get our second 0xFF
            if d == b'\xff':
                return self.getPacket(2)
            else:
                return self.getPacket(0)
        elif mode == 2:         # get id
            if d != b'\xff':
                return self.getPacket(3, ord(d))
            else:              
                return self.getPacket(0)
        elif mode == 3:         # get length
            return self.getPacket(4, id, ord(d))
        elif mode == 4:         # read error    
            self.error = d
            if leng == 2:
                return self.getPacket(6, id, leng, ord(d), list())
            else:
                return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:         # read params
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:         # read checksum
            checksum = id + leng + error + sum(params) + ord(d)
            if checksum % 256 != 255:
                return None
            return params
        # fail
        return None

    ## @brief Send an instruction to the device. 
    ##
    ## @param index The ID of the servo to write.
    ##
    ## @param ins The instruction to send.
    ##
    ## @param params A list of the params to send.
    ##
    ## @param ret Whether to read a return packet.
    ##
    ## @return The return packet, if read.
    def execute(self, index, ins, params, ret=True):
        values = None
        self._mutex.acquire()  
        try:      
            self._ser.flushInput()
        except Exception as e:
            print(e)
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params))%256)
        packet = bytearray()
        packet.append(0xFF)
        packet.append(0xFF)
        packet.append(index)
        packet.append(length)
        packet.append(ins)
        self.__write__(packet)
        for val in params:
            self.__write__(bytes([val]))
        self.__write__(bytes([checksum]))
        if ret:
            values = self.getPacket(0)
        self._mutex.release()
        return values
    
    ## @brief Read values of registers.
    ##
    ## @param index The ID of the servo.
    ## 
    ## @param start The starting register address to begin the read at.
    ##
    ## @param length The number of bytes to read.
    ##
    ## @return A list of the bytes read, or -1 if failure.
    def read(self, index, start, length):
        values = self.execute(index, AX_READ_DATA, [start, length])
        if values == None:
            return -1        
        else:
            return values

    ## @brief Write values to registers.
    ##
    ## @param index The ID of the servo.
    ##
    ## @param start The starting register address to begin writing to.
    ##
    ## @param values The data to write, in a list.
    ##
    ## @return The error level.
    def write(self, index, start, values):
        self.execute(index, AX_WRITE_DATA, [start] + values)
        return self.error     

    ## @brief Write values to registers on many servos.
    ##
    ## @param start The starting register address to begin writing to.
    ##
    ## @param values The data to write, in a list of lists. Format should be
    ## [(id1, val1, val2), (id2, val1, val2)]
    def syncWrite(self, start, values):
        output = list()
        for i in values:
            output = output + i 
        length = len(output) + 4                # length of overall packet
        lbytes = len(values[0])-1               # length of bytes to write to a servo               
        self._mutex.acquire()  
        try:      
            self._ser.flushInput()
        except:
            pass  
        packet = bytearray()
        packet.append(0xFF)
        packet.append(0xFF)
        packet.append(254)
        packet.append(length)
        packet.append(AX_SYNC_WRITE)
        self.__write__(packet)
        self.__write__(bytes([start]))              # start address
        self.__write__(bytes([lbytes]))             # bytes to write each servo
        for i in output:
            self.__write__(bytes([i]))
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + start + lbytes + sum(output))%256)
        self.__write__(bytes([checksum]))
        self._mutex.release()

    ## @brief Read values of registers on many servos.
    ##
    ## @param servos A list of the servo IDs to read from.
    ##
    ## @param start The starting register address to begin reading at.
    ##
    ## @param length The number of bytes to read from each servo.
    ##
    ## @return A list of bytes read.
    def syncRead(self, servos, start, length):
        return self.execute(0xFE, AX_SYNC_READ, [start, length] + servos )
    
    ## @brief Set baud rate of a device.
    ##
    ## @param index The ID of the device to write (Note: ArbotiX is 253).
    ##
    ## @param baud The baud rate.
    ##
    ## @return The error level.
    def setBaud(self, index, baud):
        return self.write(index, P_BAUD_RATE, [baud, ])

    ## @brief Get the return level of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The return level, .
    def getReturnLevel(self, index):
        try:
            return int(self.read(index, P_RETURN_LEVEL, 1)[0])
        except:
            return -1

    ## @brief Set the return level of a device.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The return level.
    ##
    ## @return The error level.
    def setReturnLevel(self, index, value):
        return self.write(index, P_RETURN_LEVEL, [value])        

    ## @brief Turn on the torque of a servo.
    ##
    ## @param index The ID of the device to enable.
    ##
    ## @return The error level.
    def enableTorque(self, index):
        return self.write(index, P_TORQUE_ENABLE, [1])

    ## @brief Turn on the torque of a servo.
    ##
    ## @param index The ID of the device to disable.
    ##
    ## @return The error level.
    def disableTorque(self, index):
        return self.write(index, P_TORQUE_ENABLE, [0])

    ## @brief Set the status of the LED on a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value 0 to turn the LED off, >0 to turn it on
    ##
    ## @return The error level.
    def setLed(self, index, value):
        return self.write(index, P_LED, [value])

    ## @brief Set the position of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The position to go to in, in servo ticks.
    ##
    ## @return The error level.
    def setPosition(self, index, value):
        return self.write(index, P_GOAL_POSITION_L, [value%256, value>>8])

    ## @brief Set the speed of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The speed to write.
    ##
    ## @return The error level.
    def setSpeed(self, index, value):
        return self.write(index, P_GOAL_SPEED_L, [value%256, value>>8])

    ## @brief Get the position of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo position.
    def getPosition(self, index):
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1

    ## @brief Get the speed of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo speed.
    def getSpeed(self, index):
        values = self.read(index, P_PRESENT_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1
        
    ## @brief Get the goal speed of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo goal speed.
    def getGoalSpeed(self, index):
        values = self.read(index, P_GOAL_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1

    ## @brief Get the voltage of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The voltage, in Volts.
    def getVoltage(self, index):
        try:
            return int(self.read(index, P_PRESENT_VOLTAGE, 1)[0])/10.0
        except:
            return -1    

    ## @brief Get the temperature of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The temperature, in degrees C.
    def getTemperature(self, index):
        try:
            return int(self.read(index, P_PRESENT_TEMPERATURE, 1)[0])
        except:
            return -1

    ## @brief Determine if a device is moving.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return True if servo is moving.
    def isMoving(self, index):
        try:
            d = self.read(index, P_MOVING, 1)[0]
        except:
            return True
        return d != 0
    
    ## @brief Put a servo into wheel mode (continuous rotation).
    ##
    ## @param index The ID of the device to write.
    def enableWheelMode(self, index):
        self.write(index, P_CCW_ANGLE_LIMIT_L, [0,0])

    ## @brief Put a servo into servo mode.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param resolution The resolution of the encoder on the servo. NOTE: if using 
    ## 12-bit resolution servos (EX-106, MX-28, etc), you must pass resolution = 12.
    ##
    ## @return 
    def disableWheelMode(self, index, resolution=10):
        resolution = (2 ** resolution) - 1
        self.write(index, P_CCW_ANGLE_LIMIT_L, [resolution%256,resolution>>8])

    ## Direction definition for setWheelSpeed
    FORWARD = 0
    ## Direction definition for setWheelSpeed
    BACKWARD = 1

    ## @brief Set the speed and direction of a servo which is in wheel mode (continuous rotation).
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param direction The direction of rotation, either FORWARD or BACKWARD
    ##
    ## @param speed The speed to move at (0-1023).
    ##
    ## @return 
    def setWheelSpeed(self, index, direction, speed):
        if speed > 1023:
            speed = 1023
        if direction == self.FORWARD:
            # 0~1023 is forward, it is stopped by setting to 0 while rotating to CCW direction.
            self.write(index, P_GOAL_SPEED_L, [speed%256, speed>>8])
        else:
            # 1024~2047 is backward, it is stopped by setting to 1024 while rotating to CW direction.
            speed += 1024
            self.write(index, P_GOAL_SPEED_L, [speed%256, speed>>8])

    ###########################################################################
    # Extended ArbotiX Driver

    ## Helper definition for analog and digital access.
    LOW = 0
    ## Helper definition for analog and digital access.
    HIGH = 0xff
    ## Helper definition for analog and digital access.
    INPUT = 0
    ## Helper definition for analog and digital access.
    OUTPUT = 0xff

    # ArbotiX-specific register table
    # We do Model, Version, ID, Baud, just like the AX-12
    ## Register base address for reading digital ports
    REG_DIGITAL_IN0 = 5
    REG_DIGITAL_IN1 = 6
    REG_DIGITAL_IN2 = 7
    REG_DIGITAL_IN3 = 8
    ## Register address for triggering rescan
    REG_RESCAN = 15
    # 16, 17 = RETURN, ALARM
    ## Register address of first analog port (read only).
    ## Each additional port is BASE + index.
    ANA_BASE = 18
    ## Register address of analog servos. Up to 10 servos, each
    ## uses 2 bytes (L, then H), pulse width (0, 1000-2000ms) (Write only)
    SERVO_BASE = 26
    # Address 46 is Moving, just like an AX-12
    REG_DIGITAL_OUT0 = 47

    ## @brief Force the ArbotiX2 to rescan the Dynamixel busses.
    def rescan(self):
        self.write(253, self.REG_RESCAN, [1,])

    ## @brief Get the value of an analog input pin.
    ##
    ## @param index The ID of the pin to read (0 to 7).
    ##
    ## @param leng The number of bytes to read (1 or 2).
    ##
    ## @return 8-bit/16-bit analog value of the pin, -1 if error.
    def getAnalog(self, index, leng=1):
        try:
            val = self.read(253, self.ANA_BASE+int(index), leng)
            return sum(val[i] << (i * 8) for i in range(leng))
        except:
            return -1

    ## @brief Get the value of an digital input pin.
    ##
    ## @param index The ID of the pin to read (0 to 31).
    ##
    ## @return 0 for low, 255 for high, -1 if error.
    def getDigital(self, index):
        try:
            if index < 32:
                x = self.read(253, self.REG_DIGITAL_IN0 + int(index/8), 1)[0]
            else:
                return -1
        except:
            return -1
        if x & (2**(index%8)):
            return 255
        else:
            return 0

    ## @brief Get the value of an digital input pin.
    ##
    ## @param index The ID of the pin to write (0 to 31).
    ##
    ## @param value The value of the port, >0 is high.
    ##
    ## @param direction The direction of the port, >0 is output.
    ##
    ## @return -1 if error.
    def setDigital(self, index, value, direction=0xff):
        if index > 31: return -1
        if value == 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [1])
        elif value > 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [3])
        elif value > 0 and direction == 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [2])
        else:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [0])
        return 0

    ## @brief Set the position of a hobby servo.
    ##
    ## @param index The ID of the servo to write (0 to 7).
    ##
    ## @param value The position of the servo in milliseconds (1500-2500). 
    ## A value of 0 disables servo output.
    ##
    ## @return -1 if error.
    def setServo(self, index, value):
        if index > 7: return -1
        if value != 0 and (value < 500 or value > 2500):
            print("ArbotiX Error: Servo value out of range:", value)
        else:
            self.write(253, self._SERVO_BASE + 2*index, [value%256, value>>8])
        return 0

    def query(self, max_id = 18, baud = 1000000):
        k = 0                   # how many id's have we printed
        for i in range(max_id):
            if self.getPosition(i+1) != -1:
                if k > 8:
                    k = 0
                    print("")
                print(repr(i+1).rjust(4), end="\t"),
                k = k + 1
            else:
                if k > 8:
                    k = 0
                    print("")
                print(" ....", end="\t")
                k = k + 1
            sys.stdout.flush()
        print("")


    def convertBaud(self, b):
        if b == 500000:
            return 3
        elif b == 400000:
            return 4
        elif b == 250000:
            return 7
        elif b == 200000:
            return 9
        elif b == 115200:
            return 16
        elif b == 57600:
            return 34
        elif b == 19200:    
            return 103
        elif b == 9600:
            return 207
        else:
            return 1    # default to 1Mbps

class main_gui (QtWidgets.QDialog, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.Pbon_1.clicked.connect(self.Pbon_1_handle)  ## PUSH BUTTON ON TORQUE SERVO 1
        self.Pbon_2.clicked.connect(self.Pbon_2_handle)  ## PUSH BUTTON ON TORQUE SERVO 2
        self.Pbon_3.clicked.connect(self.Pbon_3_handle)  ## PUSH BUTTON ON TORQUE SERVO 3
        self.Pbon_4.clicked.connect(self.Pbon_4_handle)  ## PUSH BUTTON ON TORQUE SERVO 4
        self.Pbon_5.clicked.connect(self.Pbon_5_handle)  ## PUSH BUTTON ON TORQUE SERVO 5
        self.Pbon_torque.clicked.connect(self.Pbon_torque_handle)  ## PUSH BUTTON ON TORQUE ALL SERVO
        self.Pboff_1.clicked.connect(self.Pboff_1_handle)  ## PUSH BUTTON OFF TORQUE SERVO 1
        self.Pboff_2.clicked.connect(self.Pboff_2_handle)  ## PUSH BUTTON OFF TORQUE SERVO 2
        self.Pboff_3.clicked.connect(self.Pboff_3_handle)  ## PUSH BUTTON OFF TORQUE SERVO 3
        self.Pboff_4.clicked.connect(self.Pboff_4_handle)  ## PUSH BUTTON OFF TORQUE SERVO 4
        self.Pboff_5.clicked.connect(self.Pboff_5_handle)  ## PUSH BUTTON OFF TORQUE SERVO 5
        self.Pboff_torque.clicked.connect(self.Pboff_torque_handle)  ## PUSH BUTTON OFF TORQUE ALL SERVO
        self.Vslider_1.valueChanged.connect(self.Vslider_1_slide_it)  ## DIRECTION MOVEMENT SLIDER SERVO 1
        self.Vslider_2.valueChanged.connect(self.Vslider_2_slide_it)  ## DIRECTION MOVEMENT SLIDER SERVO 2
        self.Vslider_3.valueChanged.connect(self.Vslider_3_slide_it)  ## DIRECTION MOVEMENT SLIDER SERVO 3
        self.Vslider_4.valueChanged.connect(self.Vslider_4_slide_it)  ## DIRECTION MOVEMENT SLIDER SERVO 4
        self.Vslider_5.valueChanged.connect(self.Vslider_5_slide_it)  ## DIRECTION MOVEMENT SLIDER SERVO 5
        self.Vslider_speed.valueChanged.connect(self.Vslider_speed_slide_it)  ## DIRECTION MOVEMENT SPEED RATE ALL SERVO
        self.pBdef_1.clicked.connect(self.pBdef_1_handle)  ## PUSH BUTTON DEFAULT SERVO 1
        self.pBdef_2.clicked.connect(self.pBdef_2_handle)  ## PUSH BUTTON DEFAULT SERVO 2
        self.pBdef_3.clicked.connect(self.pBdef_3_handle)  ## PUSH BUTTON DEFAULT SERVO 3
        self.pBdef_4.clicked.connect(self.pBdef_4_handle)  ## PUSH BUTTON DEFAULT SERVO 4
        self.pBdef_5.clicked.connect(self.pBdef_5_handle)  ## PUSH BUTTON DEFAULT SERVO 5
        self.pBdef_speed.clicked.connect(self.pBdef_speed_handle)  ## PUSH BUTTON DEFAULT SPEED RATE
        self.Pbupload.clicked.connect(self.Pbupload_handle)  ## PUSH BUTTON UPLOAD DATA
        self.Pbservo.clicked.connect(self.Pbservo_handle)  ## PUSH BUTTON CEK SERVO
        self.Pbdefault.clicked.connect(self.Pbdefault_handle)  ## PUSH BUTTON DEFAULT TO ALL COMAND
        self.Pbcon.clicked.connect(self.Pbcon_handle)  ## PUSH BUTTON CONNECT PORT
        self.Pbrefresh.clicked.connect(self.Pbrefresh_handle)  ## PUSH BUTTON REFRESH DATA
        self.Pbstop.clicked.connect(self.Pbstop_handle)  ## PUSH BUTTON STOP RUNNING ROBOT
        self.servo1= 510  ## VARIABLE DEFAULT SERVO 1
        self.servo2= 336  ## VARIABLE DEFAULT SERVO 2
        self.servo3= 1023 - self.servo2  ## VARIABLE DEFAULT SERVO 3
        self.servo4= 426  ## VARIABLE DEFAULT SERVO 4
        self.servo5= 1023 - self.servo4  ## VARIABLE DEFAULT SERVO 5
        self.servo6= 687  ## VARIABLE DEFAULT SERVO 6
        self.servo7= 512  ## VARIABLE DEFAULT SERVO 7
        self.speed=100  ## VARIABLE DEFAULT SPEED RATE ALL SERVO

        #BARU
        self.Pbgetpos.clicked.connect(self.Pbgetpos_handle)  ## PUSH BUTTON SERVOS GET POSE
        self.Pbbrowse.clicked.connect(self.Pbbrowse_handle)  ## PUSH BUTTON BROWSE MOTION DATA
        self.Pbrun1x.clicked.connect(self.Pbrun1x_handle)  ## PUSH BUTTON RUN ROBOT ONE TIME
        self.Pbrunloop.clicked.connect(self.Pbrunloop_handle)  ## PUSH BUTTON RUN ROBOT LOOPING
        self.Pbstop.clicked.connect(self.Pbstop_handle)  ## PUSH BUTTON STOP RUN ROBOT
        self.connected = False

    def IS_CONNECTED(self):
        self.frame_1.setEnabled(True)
        self.frame_2.setEnabled(True)
        self.frame_3.setEnabled(True)
        self.frame_4.setEnabled(True)
        self.frame_5.setEnabled(True)
        self.frame_6.setEnabled(True)
        self.frame_8.setEnabled(True)
        self.frame_9.setEnabled(True)

    def IS_NOT_CONNECTED(self):
        self.frame_1.setEnabled(False)
        self.frame_2.setEnabled(False)
        self.frame_3.setEnabled(False)
        self.frame_4.setEnabled(False)
        self.frame_5.setEnabled(False)
        self.frame_6.setEnabled(False)
        self.frame_8.setEnabled(False)
        self.frame_9.setEnabled(False)
        
        ###PB BARU
    def Pbgetpos_handle(self):  ## FILL BUTTON GET SERVOS POSE
        self.Vslider_1.setValue(int(self.arbotix.getPosition(1)))
        self.Vslider_2.setValue(int(self.arbotix.getPosition(2)))
        self.Vslider_3.setValue(int(self.arbotix.getPosition(4)))
        self.Vslider_4.setValue(int(self.arbotix.getPosition(6)))
        self.Vslider_5.setValue(int(self.arbotix.getPosition(7)))
             
    def Pbbrowse_handle(self):  ## FILL BUTTON INSERT MOTION FILE
        options = QFileDialog.Options()
        options = QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","CSV Files (*.csv)", options=options)
        if fileName:
            self.label_9.setText(str(fileName))

    def Pbrun1x_handle(self):  ## FILL BUTTON RUN ROBOT ONE TIME
        df = pd.read_csv(self.label_9.text())
        df = df.to_numpy()
        for row in df:            
            self.servo1 = int(row[0])
            self.servo2 = int(row[1])
            self.servo3 = 1023 - int(row[1])
            self.servo4 = int(row[2])
            self.servo5 = 1023 - int(row[2])
            self.servo6 = int(row[3])
            self.servo7 = int(row[4])
            self.speed = int(row[5])

            self.arbotix.setSpeed(1,  self.speed )
            self.arbotix.setSpeed(2,  self.speed )
            self.arbotix.setSpeed(3,  self.speed )
            self.arbotix.setSpeed(4,  self.speed )
            self.arbotix.setSpeed(5,  self.speed )
            self.arbotix.setSpeed(6,  self.speed )
            self.arbotix.setSpeed(7,  self.speed )
            self.arbotix.setPosition( 1 , self.servo1 )
            self.arbotix.setPosition( 2 , self.servo2 )
            self.arbotix.setPosition( 3 , self.servo3 )
            self.arbotix.setPosition( 4 , self.servo4 )
            self.arbotix.setPosition( 5 , self.servo5 )
            self.arbotix.setPosition( 6 , self.servo6 )
            self.arbotix.setPosition( 7 , self.servo7 )

            targetArr = np.array([self.servo1, self.servo2, self.servo3, self.servo4, self.servo5, self.servo6, self.servo7])
            TH = 15
            while True:
                currentArr = np.array([
                    int(self.arbotix.getPosition(1)),
                    int(self.arbotix.getPosition(2)),
                    int(self.arbotix.getPosition(3)),
                    int(self.arbotix.getPosition(4)),
                    int(self.arbotix.getPosition(5)),
                    int(self.arbotix.getPosition(6)),
                    int(self.arbotix.getPosition(7))
                ])

                error = np.subtract(targetArr,currentArr)
                print("ERROR : ",error, end = "\r", flush=True)
                print(end='\x1b[2K') 
                if all(abs(x) < TH for x in error):
                    break
        
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" DONE"
        self.Monitor.append(text)
        print("DONE !!!", end = "\r", flush=True)
        print(end='\x1b[2K') 
                    
    def Pbrunloop_handle(self):  ## FILL BUTTON RUN ROBOT LOOPING
        pass
        '''
            while True :
                with open(self.label_9.text(), newline='') as csvfile:
                    reader = csv.reader(csvfile, delimiter=',')
                    next(reader, None) #skip header
                    print("looping")
                    for row in reader:
                        curtime = time.time()
                        print(row)
                        self.arbotix.setSpeed(1,  int(row[5]) )
                        self.arbotix.setSpeed(2,  int(row[5]) )
                        self.arbotix.setSpeed(3,  int(row[5]) )
                        self.arbotix.setSpeed(4,  int(row[5]) )
                        self.arbotix.setSpeed(5,  int(row[5]) )
                        self.arbotix.setSpeed(6,  int(row[5]) )
                        self.arbotix.setSpeed(7,  int(row[5]) )
                        self.arbotix.setPosition( 1 , int(row[0]) )
                        self.arbotix.setPosition( 2 , int(row[1]) )
                        self.arbotix.setPosition( 3 , 1023 - int(row[1]) )
                        self.arbotix.setPosition( 4 , int(row[2]) )
                        self.arbotix.setPosition( 5 , 1023 - int(row[2]) )
                        self.arbotix.setPosition( 6 , int(row[3]) )
                        self.arbotix.setPosition( 7 , int(row[4]) )
                        while True:
                            if time.time() - curtime > int(row[6]):
                                break
        '''

    def Pbstop_handle(self):
        self.arbotix.disableTorque(1)
        self.arbotix.disableTorque(2)
        self.arbotix.disableTorque(3)
        self.arbotix.disableTorque(4)
        self.arbotix.disableTorque(5)
        self.arbotix.disableTorque(6)
        self.arbotix.disableTorque(7)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" STOP RUNNING"
        self.Monitor.append(text)
        print("stop")

        ###Push Button ON pada channel 1 sampai 5 dan Push Button ON pada Torque###

    def Pbon_1_handle(self):  ## FILL BUTTON ON TORQUE SERVO 1
        self.arbotix.enableTorque(1)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 1 ON"
        self.Monitor.append(text)
    def Pbon_2_handle(self):  ## FILL BUTTON ON TORQUE SERVO 2 & 3
        self.arbotix.enableTorque(2)
        self.arbotix.enableTorque(3)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 2 & 3 ON"
        self.Monitor.append(text)
    def Pbon_3_handle(self):  ## FILL BUTTON ON SERVO TORQUE 4 & 5
        self.arbotix.enableTorque(4)
        self.arbotix.enableTorque(5)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 4 & 5 ON"
        self.Monitor.append(text)
    def Pbon_4_handle(self):  ## FILL BUTTON ON TORQUE SERVO 5
        self.arbotix.enableTorque(6)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 6 ON"
        self.Monitor.append(text)
    def Pbon_5_handle(self):  ## FILL BUTTON ON TORQUE SERVO 7
        self.arbotix.enableTorque(7)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 7 ON"
        self.Monitor.append(text)
    def Pbon_torque_handle(self):  ## FILL BUTTON ON TORQUE ALL SERVO
        self.arbotix.enableTorque(1)
        self.arbotix.enableTorque(2)
        self.arbotix.enableTorque(3)
        self.arbotix.enableTorque(4)
        self.arbotix.enableTorque(5)
        self.arbotix.enableTorque(6)
        self.arbotix.enableTorque(7)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" TORQUE ON"
        self.Monitor.append(text)

    ###Push Button OFF pada Channel 1 sampai 5 dan Push Button OFF pada Torque###

    def Pboff_1_handle(self):  ## FILL BUTTON OFF TORQUE SERVO 1
        self.arbotix.disableTorque(1)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 1 OFF"
        self.Monitor.append(text)
    def Pboff_2_handle(self):  ## FILL BUTTON OFF TORQUE SERVO 2 & 3
        self.arbotix.disableTorque(2)
        self.arbotix.disableTorque(3)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 2 & 3 OFF"
        self.Monitor.append(text)
    def Pboff_3_handle(self):  ## FILL BUTTON OFF TORQUE SERVO 4 & 5
        self.arbotix.disableTorque(4)
        self.arbotix.disableTorque(5)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 4 & 5 OFF"
        self.Monitor.append(text)
    def Pboff_4_handle(self):  ## FILL BUTTON OFF TORQUE SERVO 6
        self.arbotix.disableTorque(6)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 6 OFF"
        self.Monitor.append(text)
    def Pboff_5_handle(self):  ## FILL BUTTON OFF TORQUE SERVO 7
        self.arbotix.disableTorque(7)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 7 OFF"
        self.Monitor.append(text)
    def Pboff_torque_handle(self):  ## FILL BUTTON OFF TORQUE ALL SERVO
        self.arbotix.disableTorque(1)
        self.arbotix.disableTorque(2)
        self.arbotix.disableTorque(3)
        self.arbotix.disableTorque(4)
        self.arbotix.disableTorque(5)
        self.arbotix.disableTorque(6)
        self.arbotix.disableTorque(7)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" TORQUE OFF"
        self.Monitor.append(text)
        
        
        ###Slider dari Channel 1 sampai Channel 5 dan Slider Speed###

    def Vslider_1_slide_it(self, value):  ## FILL SLIDER SERVO 1
        self.servo1= value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 1 POSITION"
        self.Monitor.append(text)
    def Vslider_2_slide_it(self, value):  ## FILL SLIDER SERVO 2 & 3
        self.servo2= value
        self.servo3= 1023 - value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 2 & 3 POSITION"
        self.Monitor.append(text)
    def Vslider_3_slide_it(self, value):  ## FILL SLIDER SERVO 4 & 5
        self.servo4= value
        self.servo5= 1023 - value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 4 & 5 POSITION"
        self.Monitor.append(text)
    def Vslider_4_slide_it(self, value):  ## FILL SLIDER SERVO 6
        self.servo6= value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 6 POSITION"
        self.Monitor.append(text)
    def Vslider_5_slide_it(self, value):  ## FILL SLIDER SERVO 7
        self.servo7= value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 7 POSITION"
        self.Monitor.append(text)
    def Vslider_speed_slide_it(self, value):  ## FILL SLIDER FOR SPEED ALL SERVO
        self.speed= value
        self.Monitor.append(str(value))
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SPEED RATE"
        self.Monitor.append(text)
    
    
        ###Push Button Default pada Channel 1 sampai 5 dan Push Button Default Speed###

    def pBdef_1_handle(self):  ## FILL BUTTON DEFAULT SERVO 1
        self.Vslider_1.setValue(510)
        self.arbotix.setSpeed(1,  int(self.speed) )
        self.arbotix.setPosition( 1 , int(self.servo1) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 1 DEFAULT"
        self.Monitor.append(text)
    def pBdef_2_handle(self):  ## FILL BUTTON DEFAULT SERVO 2 & 3
        self.Vslider_2.setValue(336)
        self.arbotix.setPosition( 2 , int(self.servo2) )
        self.arbotix.setSpeed(2,  int(self.speed) )
        self.arbotix.setPosition( 3 , int(self.servo3) )
        self.arbotix.setSpeed(3,  int(self.speed) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 2 DEFAULT"
        self.Monitor.append(text)
    def pBdef_3_handle(self):  ## FILL BUTTON SERVO 4 & 5
        self.Vslider_3.setValue(426)
        self.arbotix.setPosition( 4 , int(self.servo4) )
        self.arbotix.setSpeed(4,  int(self.speed) )
        self.arbotix.setPosition( 5 , int(self.servo5) )
        self.arbotix.setSpeed(5,  int(self.speed) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 3 DEFAULT"
        self.Monitor.append(text)
    def pBdef_4_handle(self):  ## FILL BUTTON DEFAULT SERVO 6
        self.Vslider_4.setValue(687)
        self.arbotix.setSpeed(6,  int(self.speed) )
        self.arbotix.setPosition( 6 , int(self.servo6) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 4 DEFAULT"
        self.Monitor.append(text)
    def pBdef_5_handle(self):  ## FILL BUTTON DEFAULT SERVO 7
        self.Vslider_5.setValue(512)
        self.arbotix.setSpeed(7,  int(self.speed) )
        self.arbotix.setPosition( 7 , int(self.servo7) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SERVO 5 DEFAULT"
        self.Monitor.append(text)
    def pBdef_speed_handle(self):  ## FILL BUTTON DEFAULT SPEED
        self.Vslider_speed.setValue(100)
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" SPEED DEFAULT"
        self.Monitor.append(text)
    
    
        ###Push Button Upload ###

    def Pbupload_handle(self):  ## FILL BUTTON UPLOAD DATA
        #print (self.servo1 , self.servo2, self.servo3 , self.servo4 , self.servo5 , self.servo6 , self.servo7)
        self.arbotix.setSpeed(1,  int(self.speed) )
        self.arbotix.setSpeed(2,  int(self.speed) )
        self.arbotix.setSpeed(3,  int(self.speed) )
        self.arbotix.setSpeed(4,  int(self.speed) )
        self.arbotix.setSpeed(5,  int(self.speed) )
        self.arbotix.setSpeed(6,  int(self.speed) )
        self.arbotix.setSpeed(7,  int(self.speed) )
        self.arbotix.setPosition( 1 , int(self.servo1) )
        self.arbotix.setPosition( 2 , int(self.servo2) )
        self.arbotix.setPosition( 3 , int(self.servo3) )
        self.arbotix.setPosition( 4 , int(self.servo4) )
        self.arbotix.setPosition( 5 , int(self.servo5) )
        self.arbotix.setPosition( 6 , int(self.servo6) )
        self.arbotix.setPosition( 7 , int(self.servo7) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" UPLOAD SUCCES"
        self.Monitor.append(text)
    
    
        ###Push Button Cek Servo###

    def Pbservo_handle(self):
        self.arbotix._ser.timeout = 0.25
        self.arbotix.query()
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" READY TO USE"
        self.Monitor.append(text)
    
    
        ###Push Button Default###

    def Pbdefault_handle(self):
        self.Vslider_1.setValue(510)
        self.Vslider_2.setValue(336)
        self.Vslider_3.setValue(426)
        self.Vslider_4.setValue(687)
        self.Vslider_5.setValue(512)
        self.Vslider_speed.setValue(100)
        self.arbotix.setSpeed(1,  int(self.speed) )
        self.arbotix.setSpeed(2,  int(self.speed) )
        self.arbotix.setSpeed(3,  int(self.speed) )
        self.arbotix.setSpeed(4,  int(self.speed) )
        self.arbotix.setSpeed(5,  int(self.speed) )
        self.arbotix.setSpeed(6,  int(self.speed) )
        self.arbotix.setSpeed(7,  int(self.speed) )
        self.arbotix.setPosition( 1 , int(self.servo1) )
        self.arbotix.setPosition( 2 , int(self.servo2) )
        self.arbotix.setPosition( 3 , int(self.servo3) )
        self.arbotix.setPosition( 4 , int(self.servo4) )
        self.arbotix.setPosition( 5 , int(self.servo5) )
        self.arbotix.setPosition( 6 , int(self.servo6) )
        self.arbotix.setPosition( 7 , int(self.servo7) )
        now = datetime.datetime.now()
        text = now.strftime("%H:%M:%S") +" DEFAULT MODE"
        self.Monitor.append(text)
    
    
        ###Push Button Refresh###

    def Pbrefresh_handle(self):

        portData = serial.tools.list_ports.comports()
        self.Port.clear()
        self.Port.addItem("DISCONNECT")
        for i in range(0,len(portData)):
            port = portData[i]
            strPort = str(port)
            if '/dev/tty' in strPort :
                splitPort=strPort.split(' ')
                self.Port.addItem(splitPort[0])
                
        
        ###Push Button Connect###

    def Pbcon_handle(self):
        PORT = str(self.Port.currentText())
        if PORT != "DISCONNECT":
            
            self.arbotix = ArbotiX(port=PORT)

            self.IS_CONNECTED()

            now = datetime.datetime.now()
            text = now.strftime("%H:%M:%S") +" CONNECTED TO "+str(PORT)
            self.Monitor.append(text)
        
        else :
            self.IS_NOT_CONNECTED()

            now = datetime.datetime.now()
            text = now.strftime("%H:%M:%S") +" DISCONNECTED"
            self.Monitor.append(text)


if __name__=='__main__':
    import sys 
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    window = main_gui()
    window.setWindowTitle('CONTROLLER')
    window.show()
    sys.exit(app.exec_())