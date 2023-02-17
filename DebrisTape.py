#!/usr/bin/python3 -u
from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import time
from w1thermsensor import W1ThermSensor   # Easy use of 1-Wire temperature sensors
import math
# ======================================================
class DebrisTape(Device):
    
    MotorLDevice = device_property(
        dtype="str", default_value="/domain/family/member"
    )
    
    MotorRDevice = device_property(
        dtype="str", default_value="/domain/family/member"
    )
    #Properties for Tape trackin
    Thickness_in_um = device_property(
        dtype = 'int16',
        default_value= 50
    )
    Inner_radius_in_mm = device_property(
        dtype ='int16',
        default_value= 15
    )
    Outer_radius_in_mm = device_property(
        dtype ='int16',
        default_value= 60
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
        hw_memorized=True
    )
    
    velocity = attribute(
        dtype="int",
        label="Velocity",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        unit="Hz"
    )
    tape_progress = attribute(
        dtype = float,
        label = 'Tape progrss in Percent',
        access =AttrWriteType.READ,
        unit = '%'

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
        self.db = Database()
        temp = self.db.get_device_property(self.get_name(), ["loops"])
        '''if temp != {}:
            self.__loops = temp["loops"] # sets loops to remembered value
        else:
            self.__loops = 0
            self.remember_loops()'''
        self.tot_turns=1000*(self.Outer_radius_in_mm-self.Inner_radius_in_mm)/(self.Thickness_in_um)
        self.length_of_tape= (2*3.14*self.Outer_radius_in_mm*self.tot_turns-3.14*self.Thickness_in_um/1000*self.tot_turns**2)
        self.last_pos =0
        self.__direction = True
        self.last_no_turns=0
    
    
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
            self.motor_left.jog_plus()
            pass
        else: # Left => Right
            self.motor_left.stop()
            self.motor_right.jog_minus()

    @command
    def stop_all(self):
        self.set_state(DevState.ON)
        self.motor_left.stop()
        self.motor_right.stop()
    
    @command
    def reset_loops(self):
        self.__loops = 0
        self.remember_loops()
    
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
        if value != self.__direction:
            #only relevant for motor step position
            if value:
                self.motor_left.set_position(self.tape_turns(self.length_of_tape*(100-self.read_tape_progress())/100)*360)
            else:
                self.motor_right.set_position(self.tape_turns(self.length_of_tape*(100-self.read_tape_progress())/100)*-360)
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
        self.__velocity = self.motor_left.velocity
        return self.__velocity
    
    def write_velocity(self, value):
        self.__velocity = value
        self.motor_left.velocity = value
        self.motor_right.velocity = value

    @command(polling_period = 1000)
    def read_tape_progress(self):
        if self.__direction:
            temp_rot = self.motor_left.position
        else:
            temp_rot = self.motor_right.position
        # = self.motor_right.position+self.motor_left.position #because L increases and R decreases while running
        
        current_no_turns = abs(temp_rot)/360 + self.last_no_turns
        self.debug_stream(str(current_no_turns))
        return 100*(self.tape_length(current_no_turns)/self.length_of_tape)
        #TODO speed adjust
    def tape_length(self,turns):
        return 2*3.14*self.Inner_radius_in_mm*turns+3.14*self.Thickness_in_um/1000*turns**2
    def tape_turns(self,length):
        #self.debug_stream()
        a = 2*3.14*self.Inner_radius_in_mm
        b=3.14*self.Thickness_in_um/1000
        self.debug_stream('c')
        self.debug_stream(str(a**2+4*b*length))
        self.debug_stream(str((-a+math.sqrt(a**2+4*b*length))/(2*b)))
        return (-a+math.sqrt(a**2+4*b*length))/(2*b)
    @command(polling_period = 1000)
    def monitor_switches(self):        
        self.__limitL = self.motor_left.hw_limit_minus
        #self.debug_stream(str(self.__limitL))
        self.__limitR = self.motor_right.hw_limit_minus
        #self.debug_stream(str(self.__limitR))        
        
        if self.__limitL and self.__limitR:
            self.stop_all()
            self.warn_stream('Both limit_switches are active!'
                             'Something is broken! Init or start_all to continue.')
            self.set_state(DevState.FAULT)        
        
        else:
            if self.get_state() == DevState.MOVING:
                if (self.__direction and self.__limitL) or (not self.__direction and self.__limitR): # tape at limit
                    self.__loops += 1
                    #self.remember_loops()
                    if self.__autoReverse:
                        
                        self.set_state(DevState.MOVING)
                        temp_run_current_L = self.motor_left.run_current
                        temp_run_current_R = self.motor_right.run_current
                        temp_velo = self.__velocity
                        if self.__direction: # Right => Left
                            self.motor_right.run_current=0.3*temp_run_current_R
                            self.motor_right.jog_minus()
                            self.motor_left.jog_plus()
                            self.motor_left.velocity =1000
                            time.sleep(0.2)
                            self.motor_left.velocity = temp_velo
                            self.motor_left.run_current=0.3*temp_run_current_L
                            self.motor_right.run_current=temp_run_current_R
                            self.motor_right.velocity =1000
                            time.sleep(0.2)
                            self.motor_right.set_position(0)
                            self.motor_left.set_position(self.tot_turns)
                            self.write_direction(not self.__direction)
                            self.motor_left.stop()
                            self.debug_stream("direction change")
                            self.motor_left.run_current=temp_run_current_L
                            self.motor_right.velocity = temp_velo
                            
                        else: # Left => Right
                            self.motor_left.run_current=0.3*temp_run_current_L
                            self.motor_left.jog_plus()
                            self.motor_right.jog_minus()
                            self.motor_right.velocity =1000
                            time.sleep(0.2)
                            self.motor_right.velocity = temp_velo
                            self.motor_right.run_current=0.3*temp_run_current_R
                            self.motor_left.run_current=temp_run_current_L
                            self.motor_left.velocity =1000
                            time.sleep(0.2)
                            self.motor_left.set_position(0)
                            self.motor_right.set_position(self.tot_turns)
                            self.write_direction(not self.__direction)
                            self.motor_right.stop()
                            self.debug_stream("direction change")
                            self.motor_right.run_current=temp_run_current_R
                            self.motor_left.velocity = temp_velo
                        
                        self.start_all()
                        
                    else:
                        self.stop_all()
                        self.set_state(DevState.ALARM)

    @command()
    def clear_state(self):
        self.set_state(DevState.ON)


    @command()
    def new_tape_inserted(self):
        self.__loops = 0
        #self.remember_loops()
        
        if self.__limitL:
            
            self.motor_right.set_position(0)
            self.motor_left.set_position(self.tot_turns)
            self.write_direction(False)
        elif self.__limitR:
            
            self.motor_left.set_position(0)
            self.motor_right.set_position(self.tot_turns)
            self.write_direction(True)
        else:
            self.debug_stream('Tape is not wound up enough')
            self.set_status('Tape is not wound up enough')
            self.set_state(DevState.ALARM)
        self.last_no_turns=0
    def tensioning(self):
        self.set_state(DevState.MOVING)
        if self.__direction: # Right => Left
            self.debug_stream('t: hc=0.3')
            self.motor_right.hold_current=0.3
            self.debug_stream('t: stop')
            self.motor_right.stop()
            self.debug_stream('t: v remember')
            temp_velo = self.__velocity
            self.debug_stream('t: v=1000')
            self.motor_left.velocity = 1000
            self.debug_stream('t: jp')
            self.motor_left.jog_plus()
            self.debug_stream('t: sleep 200')
            time.sleep(0.3)
            self.debug_stream('t: hc=0')
            self.motor_right.hold_current=0
            self.debug_stream('t: set v back')
            self.motor_left.velocity = temp_velo
            self.debug_stream('t: done')
        else: # Left => Right
            self.debug_stream('t: hc = 0.3')
            self.motor_left.hold_current=0.3
            self.debug_stream('t: stop')
            self.motor_left.stop()
            self.debug_stream('t: v remember')
            temp_velo = self.__velocity
            self.debug_stream('t: v=1000')
            self.motor_right.velocity = 1000
            self.debug_stream('t: jm')
            self.motor_right.jog_minus()
            self.debug_stream('t: sleep 200')
            time.sleep(0.3)
            self.debug_stream('t: hc=0')
            self.motor_left.hold_current=0
            self.debug_stream('t: set b back')
            self.motor_right.velocity = temp_velo
            self.debug_stream('t: done')

    def remember_loops(self):
        self.db.put_device_property(self.get_name(), {"Loops": self.__loops})

    def track_tape(self):
        #TODO memorize Tape Progress

        pass
        
        
if __name__ == "__main__":
    DebrisTape.run_server()
