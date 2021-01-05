#!/usr/bin/python3 -u
from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import sys
from enum import IntEnum
import time
from adafruit_motorkit import MotorKit
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23017 import MCP23017
from w1thermsensor import W1ThermSensor   # Easy use of 1-Wire temperature sensors

# ======================================================
class DebrisTape(Device):
    # device attributes
   
    limitL = attribute(
        dtype="bool",
        label="Limit left",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    limitR = attribute(
        dtype="bool",
        label="Limit right",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    motorL_temperature = attribute(
        dtype="float",
        label="Motor L temperature",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    motorD_temperature = attribute(
        dtype="float",
        label="Motor D temperature",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    motorR_temperature = attribute(
        dtype="float",
        label="Motor R temperature",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    loops = attribute(
        dtype="int",
        label="Loops since reset / restart",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    
    direction = attribute(
        dtype="bool",
        label="Direction. 0:L=>R, 1:R=>L",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )
    
    autoReverse = attribute(
        dtype="bool",
        label="Auto Reverse?",  # make default value = True! How do i do this here already?
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    __direction = 0
    __autoReverse = True
    __limitR = False
    __limitL = False
    __motorR_temperature = 999.99
    __motorD_temperature = 999.99
    __motorL_temperature = 999.99
    
    # Commands:
    
    @command
    def init_device(self):
        self.set_state(DevState.OFF)
        super().init_device()
        self.info_stream("init_device()")
        self.open()
        
        self.stop_all()
        self.info_stream('All motors have been stopped!')
        
        self.velocity_pull = 1
        
        self.velocity_break = 0
        self.velocity_tension = 1
        self.debug_stream('Device has done the init.')
        
        self.__loops = 0 # 2do: Should store the value on disk and only reset it manually if tape is replaced!
        
    
    @command
    def delete_device(self):
        self.stop_all()
        self.set_state(DevState.OFF)
        
    @command
    def open(self):
        self.kit = MotorKit()
        self.i2c = busio.I2C(board.SCL, board.SDA)        
        self.mcp = MCP23017(self.i2c) # MCP23017
        
        self.motorL = self.kit.motor1   #Break / Tension motor on the Left
        self.motorR = self.kit.motor3   #Break / Tension motor on the Right
        self.motorD = self.kit.motor2   #Driver motor
        
        self.switchR = self.mcp.get_pin(0) # GPA0
        self.switchL = self.mcp.get_pin(1) # GPA1

        self.switchR.direction = digitalio.Direction.INPUT
        self.switchR.pull = digitalio.Pull.UP
        
        self.switchL.direction = digitalio.Direction.INPUT
        self.switchL.pull = digitalio.Pull.UP
        self.set_state(DevState.ON)
    
    @command
    def start_all(self):
        """Direction ==>  0:L=>R, 1:R=>L
        
        # for break & tensions motors:
            # positive throttle = dispense tape
            # negative throttle = collect  tape
        
        # for driver motor:
            # positive throttle = move in positive direction
        """
        self.set_state(DevState.MOVING)
        if self.__direction: # Right => Left
            self.motorL.throttle = -self.velocity_tension
            self.motorD.throttle = +self.velocity_pull
            self.motorR.throttle = -self.velocity_break
        else: # Left => Right
            self.motorL.throttle = +self.velocity_break
            self.motorD.throttle = -self.velocity_pull
            self.motorR.throttle = +self.velocity_tension

    @command
    def stop_all(self):
        self.set_state(DevState.ON)
        self.motorL.throttle = 0
        self.motorD.throttle = 0
        self.motorR.throttle = 0
    
    @command
    def reset_loops(self):
        self.__loops = 0
    
    # Attribute read/write methods
    def read_limitL(self):
        return self.__limitL 

    def read_limitR(self):
        return self.__limitR
    
    def read_motorL_temperature(self):
        return self.__motorL_temperature
    
    def read_motorD_temperature(self):
        return self.__motorD_temperature
    
    def read_motorR_temperature(self):
        return self.__motorR_temperature
    
    def read_loops(self):
        return self.__loops
    
    def read_direction(self):
        return self.__direction
    
    def write_direction(self, value):
        
        if self.get_state() == DevState.MOVING:
            self.__direction = value
            self.start_all()
        else:
            self.__direction = value
        
    def read_autoReverse(self):
        return self.__autoReverse
    
    def write_autoReverse(self, value):
        self.__autoReverse = value
    
    @command(polling_period = 1000)
    def monitor_switches(self):
        #t1 = time.time()
        
        self.__limitL = self.switchL.value
        self.__limitR = self.switchR.value
        self.debug_stream(str(self.__limitL))
        self.debug_stream(str(self.__limitR))
        
        
        if self.__limitL and self.__limitR:
            self.stop_all()
            self.warn_stream('Both limit_switches are active! Something is broken! Init or start_all to continue.')
            self.set_state(DevState.FAULT)
        
        
        else:
            if self.get_state() == DevState.MOVING:
                if (self.__direction and self.__limitL) or (not self.__direction and self.__limitR):
                    self.__loops += 1
                    if self.__autoReverse:
                        self.__direction = not self.__direction
                        
                        #t2 = time.time()
                        #print("current round trips: {:d} "
                        #      "with duration {:f} min".format(self.round_trips, (t2-t1)/60))
                        #t1 = t2
                        self.start_all()
                    else:
                        self.stop_all()
                        self.set_state(DevState.ALARM)
                        
    @command(polling_period = 10000)
    def monitor_temperature(self):                
        self.__motorR_temperature  = W1ThermSensor(40, "3c01d607b685").get_temperature()
        self.debug_stream(str(self.__motorR_temperature))
        self.__motorD_temperature  = W1ThermSensor(40, "3c01d6077e70").get_temperature()
        self.debug_stream(str(self.__motorD_temperature))
        self.__motorL_temperature  = W1ThermSensor(40, '3c01d6072740').get_temperature()
        self.debug_stream(str(self.__motorL_temperature))
        if self.__motorR_temperature > 40 or self.__motorD_temperature > 40 or self.__motorL_temperature > 40:
            self.set_state(DevState.ALARM)

if __name__ == "__main__":
    DebrisTape.run_server()

