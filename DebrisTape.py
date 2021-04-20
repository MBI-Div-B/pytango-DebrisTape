#!/usr/bin/python3 -u
from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import time
from w1thermsensor import W1ThermSensor   # Easy use of 1-Wire temperature sensors

# ======================================================
class DebrisTape(Device):
    
    MotorLDevice = device_property(
        dtype="str", default_value="/domain/family/member"
    )
    
    MotorRDevice = device_property(
        dtype="str", default_value="/domain/family/member"
    )
    
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
        memorized=True,
        hw_memorized=True,        
    )
    
    autoReverse = attribute(
        dtype="bool",
        label="Auto Reverse?",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        memorized=True,
        hw_memorized=True,
    )
    
    velocity = attribute(
        dtype="int",
        label="Velocity",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        unit="Hz",
        memorized=True,
        hw_memorized=True,
    )

    __limitR = False
    __limitL = False
    __motorR_temperature = 999.99
    __motorL_temperature = 999.99
    
    def init_device(self):
        self.set_state(DevState.OFF)
        super().init_device()
        self.info_stream("init_device()")
        
        try:
            self.motor_left = DeviceProxy(self.MotorLDevice)
            self.info_stream('Connected to Motor Devices: {:s}'.format(self.MotorLDevice))
        except:
            self.error_stream('Could not connect to Motor Devices: {:s}'.format(self.MotorLDevice))
            self.set_state(DevState.FAULT)
            
        try:
            self.motor_right = DeviceProxy(self.MotorRDevice)
            self.info_stream('Connected to Motor Devices: {:s}'.format(self.MotorRDevice))
        except:
            self.error_stream('Could not connect to Motor Devices: {:s}'.format(self.MotorRDevice))
            self.set_state(DevState.FAULT)
        
        self.stop_all()
        self.__loops = 0
    
    @command
    def delete_device(self):
        self.stop_all()
        self.set_state(DevState.OFF)        
    
    @command
    def start_all(self):
        """Direction ==>  0:L=>R, 1:R=>L
        
        
        motor_left pulls with positive jog
        motor_right pulls with negative jog
        
        # for driver motor:
            # positive throttle = move in positive direction
        """
        self.set_state(DevState.MOVING)
        if self.__direction: # Right => Left
            self.motor_right.stop()
            self.motor_left.velocity = self.__velocity
            self.motor_left.jog_plus()
            pass
        else: # Left => Right
            self.motor_left.stop()
            self.motor_right.velocity = self.__velocity
            self.motor_right.jog_minus()

    @command
    def stop_all(self):
        self.set_state(DevState.ON)
        self.motor_left.stop()
        self.motor_right.stop()
    
    @command
    def reset_loops(self):
        self.__loops = 0
    
    # Attribute read/write methods
    def read_limitL(self):
        return self.__limitL 

    def read_limitR(self):
        return self.__limitR
    
    def read_motorL_temperature(self):
        return -1
    
    def read_motorR_temperature(self):
        return -1
    
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
        
    def read_velocity(self):
        return self.__velocity
    
    def write_velocity(self, value):
        self.__velocity = value
    
    @command(polling_period = 500)
    def monitor_switches(self):        
        self.__limitL = self.motor_left.hw_limit_minus
        self.debug_stream(str(self.__limitL))
        self.__limitR = self.motor_right.hw_limit_minus
        self.debug_stream(str(self.__limitR))        
        
        if self.__limitL and self.__limitR:
            self.stop_all()
            self.warn_stream('Both limit_switches are active!'
                             'Something is broken! Init or start_all to continue.')
            self.set_state(DevState.FAULT)        
        
        else:
            if self.get_state() == DevState.MOVING:
                if (self.__direction and self.__limitL) or (not self.__direction and self.__limitR):
                    self.__loops += 1
                    if self.__autoReverse:
                        self.__direction = not self.__direction
                        self.start_all()
                    else:
                        self.stop_all()
                        self.set_state(DevState.ALARM)
                        
    @command() #polling_period = 10000)
    def monitor_temperature(self):
        try:
            self.__motorR_temperature  = W1ThermSensor(40, "3c01d607b685").get_temperature()
            self.debug_stream(str(self.__motorR_temperature))
        except:
            self.__motorR_temperature  = 0
            self.debug_stream('Could not get temperature of right motor.')          
        try:
            self.__motorL_temperature  = W1ThermSensor(40, '3c01d6072740').get_temperature()
            self.debug_stream(str(self.__motorL_temperature))
        except:
            self.__motorL_temperature  = 0
            self.debug_stream('Could not get temperature of left motor.')            
            
        if self.__motorR_temperature > 40 or self.__motorD_temperature > 40 or self.__motorL_temperature > 40:
            self.set_state(DevState.ALARM)

    @command()
    def clear_state(self):
        self.set_state(DevState.ON)
        
        
if __name__ == "__main__":
    DebrisTape.run_server()
