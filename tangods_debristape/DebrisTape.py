from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import time
from enum import IntEnum
from w1thermsensor import W1ThermSensor  # Easy use of 1-Wire temperature sensors
import math


# ======================================================
class DebrisTape(Device):
    class Direction(IntEnum):
        LEFT_TO_RIGHT = 0
        RIGHT_TO_LEFT = 1

    MotorLDevice = device_property(dtype="str", default_value="domain/family/member")

    MotorRDevice = device_property(dtype="str", default_value="domain/family/member")
    # Properties for Tape tracking
    Thickness_in_um = device_property(dtype="int16", default_value=50)
    Inner_radius_in_mm = device_property(dtype="int16", default_value=15)
    Outer_radius_in_mm = device_property(dtype="int16", default_value=60)
    motor_left_jog_direction = device_property(
        dtype=bool,
        default_value=True,
        doc="True - plus direction, False - minus. Depends on motor orientation. For spec beamline needs to be false, for scattering - true",
    )
    motor_right_jog_direction = device_property(
        dtype=bool,
        default_value=False,
        doc="True - plus direction, False - minus. Depends on motor orientation. Must be false for both of beamlines.",
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

    loops = attribute(
        dtype="int",
        label="Loops since reset / restart",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    direction = attribute(
        dtype=Direction,
        label="Direction",
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
    )
    tape_progress = attribute(
        dtype=float,
        label="Tape progress in percent",
        access=AttrWriteType.READ,
        unit="%",
    )

    def init_device(self):
        self.set_state(DevState.OFF)
        super().init_device()
        self.info_stream("init_device()")


        self.__limitR = False
        self.__limitL = False

        try:
            self.motor_left = DeviceProxy(self.MotorLDevice)
            self.info_stream(
                "Connected to Motor Devices: {:s}".format(self.MotorLDevice)
            )
        except:
            self.error_stream(
                "Could not connect to Motor Devices: {:s}".format(self.MotorLDevice)
            )
            self.set_state(DevState.FAULT)

        try:
            self.motor_right = DeviceProxy(self.MotorRDevice)
            self.info_stream(
                "Connected to Motor Devices: {:s}".format(self.MotorRDevice)
            )
        except:
            self.error_stream(
                "Could not connect to Motor Devices: {:s}".format(self.MotorRDevice)
            )
            self.set_state(DevState.FAULT)

        self.stop_all()
        self.db = Database()
        temp = self.db.get_device_property(self.get_name(), ["loops"])
        self.__loops = temp

        self.tot_turns = (
            1000
            * (self.Outer_radius_in_mm - self.Inner_radius_in_mm)
            / (self.Thickness_in_um)
        )
        self.length_of_tape = (
            2 * math.pi * self.Outer_radius_in_mm * self.tot_turns
            - math.pi * self.Thickness_in_um / 1000 * self.tot_turns**2
        )
        self.last_pos = 0
        self.__direction = True
        self.last_no_turns = 0

    def delete_device(self):
        self.stop_all()
        self.set_state(DevState.OFF)

    @command
    def start_all(self):
        """Direction ==>  0:L=>R, 1:R=>L

        depends on the setup the motors can pull with either positive or negative jog

        for scattering beamline:
        motor_left pulls with positive jog
        motor_right pulls with negative jog

        for spectroscopy beamline:
        motor_left pulls with negative jog
        motor_right pulls with negative jog
        # for driver motor:
            # positive throttle = move in positive direction
        """
        self.set_state(DevState.MOVING)
        if self.__direction:  # Right => Left
            self.motor_right.stop()
            if self.motor_left_jog_direction:
                self.motor_left.jog_plus()
            else:
                self.motor_left.jog_minus()
        else:  # Left => Right
            self.motor_left.stop()
            if self.motor_right_jog_direction:
                self.motor_right.jog_plus()
            else:
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

    def read_loops(self):
        return self.__loops

    def read_direction(self):
        # from 0/1 to enum
        return self.Direction(self.__direction)

    def write_direction(self, value):
        # from enum to 0/1
        value = value.value
        if value != self.__direction:
            # only relevant for motor step position
            if value:
                self.motor_left.set_position(
                    self.tape_turns(
                        self.length_of_tape * (100 - self.read_tape_progress()) / 100
                    )
                    * 360
                )
            else:
                self.motor_right.set_position(
                    self.tape_turns(
                        self.length_of_tape * (100 - self.read_tape_progress()) / 100
                    )
                    * -360
                )
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

    @command(polling_period=1000)
    def read_tape_progress(self):
        if self.__direction:
            temp_rot = self.motor_left.position
        else:
            temp_rot = self.motor_right.position
        # = self.motor_right.position+self.motor_left.position #because L increases and R decreases while running

        current_no_turns = abs(temp_rot) / 360 + self.last_no_turns
        self.debug_stream(str(current_no_turns))
        return 100 * (self.tape_length(current_no_turns) / self.length_of_tape)
        # TODO speed adjust

    def tape_length(self, turns):
        return (
            2 * math.pi * self.Inner_radius_in_mm * turns
            + math.pi * self.Thickness_in_um / 1000 * turns**2
        )

    def tape_turns(self, length):
        # self.debug_stream()
        a = 2 * math.pi * self.Inner_radius_in_mm
        b = math.pi * self.Thickness_in_um / 1000
        self.debug_stream("c")
        self.debug_stream(str(a**2 + 4 * b * length))
        self.debug_stream(str((-a + math.sqrt(a**2 + 4 * b * length)) / (2 * b)))
        return (-a + math.sqrt(a**2 + 4 * b * length)) / (2 * b)

    @command(polling_period=1000)
    def monitor_switches(self):
        if 'limit' in self.motor_left.Status():
            self.__limitL = True
        else:
            self.__limitL = False

        if 'limit' in self.motor_right.Status():
            self.__limitR = True
        else:
            self.__limitR = False

        if self.__limitL and self.__limitR:
            self.stop_all()
            self.warn_stream(
                "Both limit_switches are active!"
                "Something is broken! Init or start_all to continue."
            )
            self.set_state(DevState.FAULT)

        else:
            if self.get_state() == DevState.MOVING:
                if (self.__direction and self.__limitL) or (
                    not self.__direction and self.__limitR
                ):  # tape at limit
                    self.__loops += 1
                    self.remember_loops()
                    self.stop_all()
                    self.set_state(DevState.ALARM)

    @command()
    def clear_state(self):
        self.set_state(DevState.ON)

    @command()
    def new_tape_inserted(self):
        self.__loops = 0
        self.remember_loops()

        if self.__limitL:
            self.motor_right.set_position(0)
            self.motor_left.set_position(self.tot_turns)
            self.write_direction(False)
        elif self.__limitR:
            self.motor_left.set_position(0)
            self.motor_right.set_position(self.tot_turns)
            self.write_direction(True)
        else:
            self.debug_stream("Tape is not wound up enough")
            self.set_status("Tape is not wound up enough")
            self.set_state(DevState.ALARM)
        self.last_no_turns = 0

    def remember_loops(self):
        self.db.put_device_property(self.get_name(), {"Loops": self.__loops})


if __name__ == "__main__":
    DebrisTape.run_server()
