from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import time
from enum import IntEnum

# from w1thermsensor import W1ThermSensor  # Easy use of 1-Wire temperature sensors
import math


class Direction(IntEnum):
    LEFT_TO_RIGHT = 0
    RIGHT_TO_LEFT = 1


# ======================================================
class DebrisTape(Device):
    MotorLDevice = device_property(dtype="str", default_value="domain/family/member")
    LoopsLeft = device_property(float)
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
        dtype=bool,
        label="Limit left",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    limitR = attribute(
        dtype=bool,
        label="Limit right",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )
    loops_left = attribute(
        dtype=float,
        label="loops left on the current (moving) roll",
        access=AttrWriteType.READ_WRITE,
    )

    direction = attribute(
        dtype=Direction,
        label="Direction",
        access=AttrWriteType.READ_WRITE,
        memorized=True,
        hw_memorized=True,
    )

    velocity = attribute(
        dtype=float,
        label="Velocity",
        format=".2f",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        unit="Hz",
    )
    tape_progress = attribute(
        dtype=float,
        label="Tape progress in percent",
        access=AttrWriteType.READ_WRITE,
        polling_period=1000,
        unit="%",
        memorized=True,
        hw_memorized=True,
    )
    full_tapes_loops = attribute(
        dtype=float,
        label="calculated loops based on radius",
        display_level=DispLevel.EXPERT,
        access=AttrWriteType.READ,
    )

    def init_device(self):
        self.set_state(DevState.OFF)
        super().init_device()
        self.info_stream("init_device()")

        self.__limitR = False
        self.__limitL = False
        self.__velocity = None
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
        # total loops on the full roll of tape, calculated once and is a constant
        self.db = Database()
        self.FULL_TAPE_LOOPS = (self.Outer_radius_in_mm - self.Inner_radius_in_mm) / (
            self.Thickness_in_um / 1000
        )
        self.stop_all()
        self.write_loops_left(self.LoopsLeft)
        

        """
        instead of converting length and loops back in forth and
        accumlate the error we would rather "think" in terms of loops
        the single conversion from length to loops is needed when
        new tape is inserted
        moreover, this is basically a constant value dependent only
        on properties as thikness, inner and outer radius
        """
        self.reset_previous_motor_positions()
        self.__direction = None

    def delete_device(self):
        self.stop_all()
        self.set_state(DevState.OFF)

    @command
    def start_all(self):
        """
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
        if self.__direction == Direction.RIGHT_TO_LEFT:
            self.motor_right.stop()
            if self.motor_left_jog_direction:
                self.motor_left.jog_plus()
            else:
                self.motor_left.jog_minus()
        elif self.__direction == Direction.LEFT_TO_RIGHT:
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

    # Attribute read/write methods
    def read_limitL(self):
        return self.__limitL

    def read_limitR(self):
        return self.__limitR

    def read_loops_left(self):
        return self._loops_left
    
    def read_full_tapes_loops(self):
        return self.FULL_TAPE_LOOPS

    # loops left cannot be less then 0 and greater then self.FULL_TAPE_LOOPS
    def write_loops_left(self, value):
        self._loops_left = min(max(0, value), self.FULL_TAPE_LOOPS)
        self.db.put_device_property(self.get_name(), {"LoopsLeft": self._loops_left})

    def read_direction(self):
        return self.__direction

    def write_direction(self, value):
        # conversion from tango's 0/1 to enum
        value = Direction(value)
        # if direction is being changed
        if self.__direction is None:
            self.__direction = value
        if value != self.__direction:
            # "inverting" the loops left
            was_moving = self.get_state() == DevState.MOVING
            self.stop_all()
            self.__direction = value
            self.write_loops_left(self.FULL_TAPE_LOOPS - self.read_loops_left())
            if was_moving:
                self.start_all()

    def read_velocity(self):
        self.__velocity = self.motor_left.velocity
        return self.__velocity

    def write_velocity(self, value):
        self.__velocity = value
        self.motor_left.velocity = value
        self.motor_right.velocity = value

    def read_tape_progress(self):
        self.write_tape_progress((1 - self.read_loops_left() / self.FULL_TAPE_LOOPS) * 100)
        return self._tape_progress

    def write_tape_progress(self, value):
        self._tape_progress = value

    @command(polling_period=1000)
    def update_tape_progress(self):
        if self.get_state() == DevState.MOVING:
            # position from phytron is given in degrees
            motor_left_new_position = self.motor_left.position
            motor_right_new_position = self.motor_right.position

            if self.previous_motor_right_position is None:
                self.previous_motor_right_position = motor_right_new_position

            if self.previous_motor_left_position is None:
                self.previous_motor_left_position = motor_left_new_position

            motor_left_diff = abs(
                self.previous_motor_left_position - motor_left_new_position
            )
            motor_right_diff = abs(
                self.previous_motor_right_position - motor_right_new_position
            )
            self.previous_motor_left_position = self.motor_left.position
            self.previous_motor_right_position = self.motor_right.position
            # while only one of the motor is running we can just take the maximum
            # value of the diffs
            # actually, if we would have both motor running their change of position should be the same
            degree_diff = max(motor_left_diff, motor_right_diff)
            # in loops from degrees
            loops_diff = degree_diff / 360
            # decreasing the loops left
            self.write_loops_left(self.read_loops_left() - loops_diff)
            # 100% tape progress corresponds to 0 loops left
            # 0% tape progress coreesponds to self.FULL_TAPE_LOOPS (see const) loops left
            # the self.read_loops_left garantues that the boundaries above hold

    @command(polling_period=1000)
    def monitor_switches(self):
        if "limit" in self.motor_left.Status():
            self.__limitL = True
        else:
            self.__limitL = False

        if "limit" in self.motor_right.Status():
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
                if (self.__direction == Direction.RIGHT_TO_LEFT and self.__limitL) or (
                    self.__direction == Direction.LEFT_TO_RIGHT and self.__limitR
                ):  # tape at limit
                    self.stop_all()
                    self.set_state(DevState.ALARM)

    @command()
    def clear_state(self):
        self.set_state(DevState.ON)

    @command()
    def new_tape_inserted(self):
        if self.__limitL:
            self.motor_right.set_position(0)
            self.motor_left.set_position(0)
            self.reset_previous_motor_positions()
            self.write_direction(Direction.LEFT_TO_RIGHT)
        elif self.__limitR:
            self.motor_left.set_position(0)
            self.motor_right.set_position(0)
            self.reset_previous_motor_positions()
            self.write_direction(Direction.RIGHT_TO_LEFT)
        else:
            self.debug_stream("Tape is not wound up enough")
            self.set_status("Tape is not wound up enough")
            self.set_state(DevState.ALARM)

    def reset_previous_motor_positions(self):
        self.previous_motor_right_position = None
        self.previous_motor_left_position = None

    def delete_device(self):
        super().delete_device()       


if __name__ == "__main__":
    DebrisTape.run_server()
