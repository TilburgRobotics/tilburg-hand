import array
import os
import sys
import json
import ctypes
from enum import IntEnum

if sys.platform == 'linux':
    import fcntl
    import termios

import serial.tools.list_ports as port

from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
                           DXL_LOWORD, DXL_HIWORD, DXL_LOBYTE, DXL_HIBYTE, COMM_SUCCESS)


class Finger(IntEnum):
    """ Constants to identify the position of each joint in the motor vectors, for example
    to identify individual joints from the vector returned by get_encoder_vector(), or to
    select individual joints in the vector passed to set_position_vector().
    """
    THUMB_IP = 0
    THUMB_MCP = 1
    THUMB_ABD = 2
    THUMB_CMC = 3

    INDEX_DIP = 4
    INDEX_PIP = 5
    INDEX_MCP = 6
    INDEX_ABD = 7

    MIDDLE_DIP = 8
    MIDDLE_PIP = 9
    MIDDLE_MCP = 10
    MIDDLE_ABD = 11

    RING_DIP = 12
    RING_PIP = 13
    RING_MCP = 14
    RING_ABD = 15


class Wrist(IntEnum):
    PITCH = 16
    YAW = 17


class OperatingMode(IntEnum):
    """ Constants to identify the 3 control modes for the motor. The default mode is POSITION, which acts as servo
    control with a pre-defined PID controlled. More details at
    https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode .
    """
    POSITION = 3
    CURRENT = 0
    CURRENT_BASED_POSITION = 5


class Unit(IntEnum):
    """ Constants to identify the units used in each function.
    """
    RAW = 1
    """
    RAW: raw values directly read/written in the Dynamixel motors.
    """

    NORMALIZED = 2
    """
    NORMALIZED: values clipped in [0, 1], automatically mapped to the full range of the joints.
                   0 corresponds to 'joint open/extended', while 1 corresponds to 'joint closed'.
                   For abduction-adduction joints (lateral finger movement)), 0.5 is the middle position,
                   0 is movement towards the thumb (regardless of hand left/right), and 1 is movement away
                   from the thumb.
    """

    DEGREES = 3
    """
    DEGREES: values are expressed in degrees, within the valid range for each joint. Low values extend the joint
             while high value flex/close the joint.
    """

    DEG_PER_SECOND = 4
    """
    DEG_PER_SECOND: angular velocity reading in degrees/s.
    """

    REV_PER_MINUTE = 5
    """
    REV_PER_MINUTE: angular velocity reading in revolutions/min.
    """


# The interface is specifically designed for Dynamixel XL-330 motors; some addresses may be different if other motors
# are used instead.
PROTOCOL_VERSION = 2.0               # See which protocol version is used in the Dynamixel
BAUDRATE = 4000000             # Dynamixel default baudrate : 1000000

ADDRESS_OPERATING_MODE = 11
ADDRESS_TORQUE_ENABLE = 64
ADDRESS_LED = 65
ADDRESS_GOAL_CURRENT = 102
ADDRESS_GOAL_POSITION = 116
ADDRESS_PRESENT_CURRENT = 126
ADDRESS_PRESENT_VELOCITY = 128
ADDRESS_PRESENT_POSITION = 132
ADDRESS_PRESENT_TEMPERATURE = 146


def _set_low_latency_mode(serial, low_latency_settings):
    buf = array.array('i', [0] * 32)

    try:
        # get serial_struct
        fcntl.ioctl(serial.fd, termios.TIOCGSERIAL, buf)

        # set or unset ASYNC_LOW_LATENCY flag
        if low_latency_settings:
            buf[4] |= 0x2000
        else:
            buf[4] &= ~0x2000

        # set serial_struct
        fcntl.ioctl(serial.fd, termios.TIOCSSERIAL, buf)
    except IOError as e:
        raise ValueError('Failed to update ASYNC_LOW_LATENCY flag to {}: {}'.format(low_latency_settings, e))


def _np_clip(value, min_, max_):
    if value < min_:
        return min_
    elif value > max_:
        return max_
    return value

def _clip(value, val1, val2):
    if val1 <= val2:
        return _np_clip(value, val1, val2)
    else:
        return _np_clip(value, val2, val1)


def _convert_unit(unit):
    if isinstance(unit, str):
        unit = unit.lower()
        if unit == 'raw':
            return Unit.RAW
        elif unit == 'normalized':
            return Unit.NORMALIZED
        elif unit == 'degrees':
            return Unit.DEGREES
        elif unit == 'deg_per_second':
            return Unit.DEG_PER_SECOND
        elif unit == 'rev_per_minute':
            return Unit.REV_PER_MINUTE
    return unit


class TilburgHandMotorInterface:
    """ Main Python interface for the Tilburg Hand.

    Example::

        from tilburg_hand import TilburgHandMotorInterface, Unit

        motors = TilburgHandMotorInterface()
        ret = motors.connect()

        pos_normalized = [0.9, 0.7, 0.2, 0.5, 0.0, 0, 0, 0.9, 0.0, 0, 0, 0.1, 0.9, 0.85, 0.85, 0, 0, 0]
        motors.set_pos_vector(pos_normalized, unit=Unit.NORMALIZED)
        sleep(3)

        motors.goto_zero_position()
        sleep(1)

        motors.disconnect()

    """
    def __init__(self, config_file=None,
                 calibration_file=None,
                 hand_orientation="right",
                 default_unit=Unit.NORMALIZED,
                 verbose=False):
        """
        :param config_file: path to config.json configuration file. If None, the default config.json is used
                            (located within the tilburg-hand Python library, in the subfolder
                            `tilburg_hand/motorgui/config.json’).
        :type config_file: str, optional

        :param calibration_file: path to calibration.json file with the range of each joint and the default
                                 zero position. If None, the default calibration.json is used (located within
                                 $HOME/tilburg_hand/calibration.json, which is generated the first time the motor
                                 interface is called).
        :type calibration_file: str, optional

        :param hand_orientation: orientation of the hand (i.e., 'left', or 'right'). This is used to create a default
                                 calibration.json file if none is found.
        :type hand_orientation: str, optional

        :param default_unit: default unit to use for controlling the motors position and the encoder readings. Valid
                             values are Unit.RAW, Unit.NORMALIZED, and Unit.DEGREES. Optionally, the uni can be
                             expressed as a string ('raw', 'normalized', 'degrees').
        :type default_unit: Unit, optional

        The config.json file contains information to connect to the U2D2 interface board via USB. By default, no port
        name is set and the library will automatically attempt to detect the correct port using the U2D2 VID:PID code.
        If you wish to select the usb port manually, please modify the config.json file.
        """
        self.dynamixel_portHandler = None
        self.dynamixel_packetHandler = None
        self.dynamixel_groupSyncWrite_position = None

        if config_file is None or not os.path.exists(config_file):
            print("ERROR: Config file [", config_file, "] does not exist!")
            config_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'motorgui', 'config.json')
            if not os.path.exists(config_file):
                return
            print("\t Using config file ", config_file)

        with open(config_file) as config:
            config_data = json.load(config)

        self.usb_port = config_data['motors_dynamixelsdk_port']
        self.dynamixel_board_vid_pid = config_data['motors_dynamixelsdk_VID_PID']
        self.joint_motor_mapping = config_data['joint_motor_mapping']

        self.n_motors = len(self.joint_motor_mapping)

        self.motor_enabled = None

        self.default_unit = _convert_unit(default_unit)
        if self.default_unit not in [Unit.RAW, Unit.NORMALIZED, Unit.DEGREES]:
            print("WARNING: default unit [", self.default_unit, "] not recognized. Defaulting to 'normalized'.")
            self.default_unit = Unit.NORMALIZED
        self.verbose = verbose

        # Code to list usb devices and devids
        portlist = list(port.comports())
        for p in portlist:
            device = p.device
            usb_info = p.usb_info()

            if self.usb_port == '' and usb_info.startswith("USB VID:PID=" + self.dynamixel_board_vid_pid):
                # U2D2 board
                self.usb_port = device
                print("U2D2 board found!")
                break

        self.device_port_found = (self.usb_port != '')

        self.motor_calib_min_range_deg = [0]*self.n_motors
        self.motor_calib_max_range_deg = [0]*self.n_motors
        self.motor_calib_min_range_ticks = [0]*self.n_motors
        self.motor_calib_max_range_ticks = [0]*self.n_motors
        self.motor_calib_zero_pos_ticks = None

        self.motor_enabled = [False]*self.n_motors
        self.connected = False

        hand_orientation = hand_orientation.lower()
        self.hand_orientation = hand_orientation
        assert (hand_orientation == 'left' or
                hand_orientation == 'right' or
                hand_orientation == '' or
                hand_orientation is None), \
               'hand_orientation should be a string either "left" or "right"'

        if calibration_file is not None:
            self.load_calibration(calibration_file)
        else:
            default_calibration = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                                               'motorgui',
                                               'calibration.json')
            self.load_calibration(default_calibration)
            calibration_file = os.path.join(os.path.expanduser("~"), 'tilburg_hand', 'calibration.json')
            self.save_calibration(calibration_file)

    def connect(self):
        """ Start a connection to the Tilburg Hand motors.

        If connection is successful and the calibration file does not contain initial default motor positions
        (motor_calib_zero_pos_ticks), the default initial position is filled in with the position of the motors
        at startup.

        The method also sets useful fields such as `self.connected` (True/False), and `self.motor_enabled' (list of
        booleans for each motor, depending on whether each motor was detected or not).

        :return: 1 on successful connection, -1 on error.
        :rtype: int
        """
        if not self.device_port_found:
            return -1

        self.dynamixel_portHandler = PortHandler(self.usb_port)
        self.dynamixel_packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.dynamixel_groupSyncWrite_position = GroupSyncWrite(self.dynamixel_portHandler,
                                                                self.dynamixel_packetHandler,
                                                                start_address=ADDRESS_GOAL_POSITION,
                                                                data_length=4)
        self.dynamixel_groupSyncWrite_current = GroupSyncWrite(self.dynamixel_portHandler,
                                                               self.dynamixel_packetHandler,
                                                               start_address=ADDRESS_GOAL_CURRENT,
                                                               data_length=2)

        self.dynamixel_groupSyncRead_position = GroupSyncRead(self.dynamixel_portHandler,
                                                              self.dynamixel_packetHandler,
                                                              start_address=ADDRESS_PRESENT_POSITION,
                                                              data_length=4)
        self.dynamixel_groupSyncRead_current = GroupSyncRead(self.dynamixel_portHandler,
                                                             self.dynamixel_packetHandler,
                                                             start_address=ADDRESS_PRESENT_CURRENT,
                                                             data_length=2)
        self.dynamixel_groupSyncRead_velocity = GroupSyncRead(self.dynamixel_portHandler,
                                                              self.dynamixel_packetHandler,
                                                              start_address=ADDRESS_PRESENT_VELOCITY,
                                                              data_length=4)

        if self.dynamixel_portHandler.openPort():
            print("Motor board connected!")
        else:
            print("Failed to connect to the motor board.")
            return -1

        if not self.dynamixel_portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the motor baudrate")
            return -1

        if sys.platform == 'linux':
            _set_low_latency_mode(self.dynamixel_portHandler.ser, True)
        else:
            print("WARNING: it is advised to configure the USB port in use for low-latency mode. This is only"
                  "performed automatically on Linux. For Windows, please see: "
                  "https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting")

        self.connected = True

        self.motor_enabled = self.check_enabled_motors()

        if self.motor_calib_zero_pos_ticks is None:
            self.motor_calib_zero_pos_ticks = self.get_all_encoders()

        self.set_torques(True)
        self.set_leds(True)

        return 1

    def disconnect(self):
        """ Closes the connection to the Tilburg Hand motors. Please note that this is required, or else the motors
        will remain in Torque-enabled mode.
        """

        if self._assert_connected():
            self.set_torques(False)
            self.set_torques(False)
            self.dynamixel_portHandler.closePort()
            self.connected = False

    def _assert_connected(self):
        if self.connected:
            return True
        else:
            print("You should first call the connect() method!")
            return False

    def load_calibration(self, calibration_file):
        if not os.path.exists(calibration_file):
            print("Calibration file [", calibration_file, "] does not exist.")

            new_calibration_folder = os.path.join(os.path.expanduser("~"), "tilburg_hand")
            if not os.path.exists(new_calibration_folder):
                os.makedirs(new_calibration_folder)

            calibration_file = os.path.join(new_calibration_folder, 'calibration.json')

        if not os.path.exists(calibration_file):
            print("Creating new default calibration file in: ", new_calibration_folder)

            motor_calib = []
            for i in range(18):
                motor_calib.append([0, 0, 0, 0, 0])  # minrange_deg, maxrange_deg, min_ticks, max_ticks, zero_ticks
                # if max_ticks < min_ticks, then the axis is inverted

            motor_calib[Finger.THUMB_IP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.THUMB_MCP] = (0, 90, 2048, 1024, 2048)

            motor_calib[Finger.INDEX_DIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.INDEX_PIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.INDEX_MCP] = (0, 95, 2048, 967, 2048)

            motor_calib[Finger.MIDDLE_DIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.MIDDLE_PIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.MIDDLE_MCP] = (0, 95, 2048, 967, 2048)

            motor_calib[Finger.RING_DIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.RING_PIP] = (-5, 95, 1991, 3128, 2048)
            motor_calib[Finger.RING_MCP] = (0, 95, 2048, 967, 2048)

            if self.hand_orientation == 'left':
                motor_calib[Finger.THUMB_ABD] = (0, 100, 2048, 3185, 2048)
                motor_calib[Finger.THUMB_CMC] = (0, 90, 2048, 1024, 2048)

                motor_calib[Finger.INDEX_ABD] = (-25, 25, 1763, 2332, 2048)
                motor_calib[Finger.MIDDLE_ABD] = (-25, 25, 1763, 2332, 2048)
                motor_calib[Finger.RING_ABD] = (-25, 25, 1763, 2332, 2048)
            else:
                motor_calib[Finger.THUMB_ABD] = (0, 100, 2048, 910, 2048)
                motor_calib[Finger.THUMB_CMC] = (0, 90, 2048, 3072, 2048)

                motor_calib[Finger.INDEX_ABD] = (-25, 25, 2332, 1763, 2048)
                motor_calib[Finger.MIDDLE_ABD] = (-25, 25, 2332, 1763, 2048)
                motor_calib[Finger.RING_ABD] = (-25, 25, 2332, 1763, 2048)

            self.motor_calib_zero_pos_ticks = [0]*18
            for i in range(18):
                self.motor_calib_min_range_deg[i] = motor_calib[i][0]
                self.motor_calib_max_range_deg[i] = motor_calib[i][1]
                self.motor_calib_min_range_ticks[i] = motor_calib[i][2]
                self.motor_calib_max_range_ticks[i] = motor_calib[i][3]
                self.motor_calib_zero_pos_ticks[i] = motor_calib[i][4]

            self.save_calibration(os.path.join(new_calibration_folder, "calibration.json"))
        else:
            with open(calibration_file) as calib_file:
                calibration = json.load(calib_file)

            self.motor_calib_min_range_deg = calibration['motor_calib_min_range_deg']
            self.motor_calib_max_range_deg = calibration['motor_calib_max_range_deg']
            self.motor_calib_min_range_ticks = calibration['motor_calib_min_range_ticks']
            self.motor_calib_max_range_ticks = calibration['motor_calib_max_range_ticks']
            self.motor_calib_zero_pos_ticks = calibration.get('motor_calib_zero_pos_ticks', None)
            self.hand_orientation = calibration.get('hand_orientation', '')

    def save_calibration(self, calibration_file):
        calibration = {'motor_calib_min_range_deg': self.motor_calib_min_range_deg,
                       'motor_calib_max_range_deg': self.motor_calib_max_range_deg,
                       'motor_calib_min_range_ticks': self.motor_calib_min_range_ticks,
                       'motor_calib_max_range_ticks': self.motor_calib_max_range_ticks,
                       'motor_calib_zero_pos_ticks': self.motor_calib_zero_pos_ticks,
                       'hand_orientation': self.hand_orientation}
        if self.motor_calib_zero_pos_ticks is not None:
            calibration['motor_calib_zero_pos_ticks'] = self.motor_calib_zero_pos_ticks
        with open(calibration_file, 'w') as outfile:
            json.dump(calibration, outfile)

    def check_enabled_motors(self):
        """ Detect which motors are connected.

        :return: list of booleans for each motor, identifying which motors were detected.
        :rtype: list
        """

        motor_id_enabled = [False]*self.n_motors

        if not self._assert_connected():
            return motor_id_enabled

        for i in range(self.n_motors):
            id = self.joint_motor_mapping[i]

            dxl_model_number, dxl_comm_result, dxl_error = \
                self.dynamixel_packetHandler.ping(self.dynamixel_portHandler, id)
            if dxl_comm_result == COMM_SUCCESS:
                motor_id_enabled[i] = True

            if self.verbose:
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))
                else:
                    print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, dxl_model_number))

        return motor_id_enabled

    def set_operating_mode(self, mode=OperatingMode.POSITION):
        """ Set the desired operating mode. You *must* call this function with motor torque off,
        i.e., temporarily setting set_torques(False).

        :param mode: one of OperatingMode modes.
                    Allowed modes are:
                        OperatingMode.POSITION: control using set_pos_vector and set_pos_single
                        OperatingMode.CURRENT:  control using set_current_vector
                        OperatingMode.CURRENT_BASED_POSITION: control using both position and current
        :type mode: OperatingMode, optional

        """
        if not self._assert_connected():
            return

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                dxl_comm_result, dxl_error = self.dynamixel_packetHandler.write1ByteTxRx(self.dynamixel_portHandler,
                                                                                         self.joint_motor_mapping[i],
                                                                                         address=ADDRESS_OPERATING_MODE,
                                                                                         data=mode)

    def set_pos_vector(self, positions, unit=None, margin_pct=0.05):
        """ Set the position of all motors at the same time (the vector must have 16 or 18 components).

        :param positions: vector with a position value for each motor
        :type positions: list

        :param unit: unit of the values in the position vector (Unit.RAW, Unit.NORMALIZED, Unit.DEGREES).
                     If None, the default unit is used, as selected in the constructor.
        :type unit: Unit, optional

        :param margin_pct: if normalized positions are chosen, then values from [0,1] are renormalized to
                           [margin_pct, 1-margin_pct] to decrease the likelihood of self collisions.
        0 corresponds to open, 1 to closed (e.g., in flex joints)

        :type margin_pct: float, optional
        """
        if not self._assert_connected():
            return

        unit = _convert_unit(unit)
        if unit == Unit.RAW or (unit is None and self.default_unit == Unit.RAW):
            for i in range(len(positions)):
                positions[i] = int(_clip(positions[i],
                                         self.motor_calib_min_range_ticks[i],
                                         self.motor_calib_max_range_ticks[i]))
        elif unit == Unit.NORMALIZED or (unit is None and self.default_unit == Unit.NORMALIZED):
            for i in range(len(positions)):
                positions[i] = _np_clip(positions[i], 0, 1)
                min_val = self.motor_calib_min_range_ticks[i]*(1.0+margin_pct)
                max_val = self.motor_calib_max_range_ticks[i]*(1.0-margin_pct)
                positions[i] = int(positions[i]*(max_val-min_val) + min_val)
        elif unit == Unit.DEGREES or (unit is None and self.default_unit == Unit.DEGREES):
            for i in range(len(positions)):
                minrange_deg = self.motor_calib_min_range_deg[i]
                maxrange_deg = self.motor_calib_max_range_deg[i]
                minrange_raw = self.motor_calib_min_range_ticks[i]*(1.0+margin_pct)
                maxrange_raw = self.motor_calib_max_range_ticks[i]*(1.0-margin_pct)
                pos_deg = _clip(positions[i], minrange_deg, maxrange_deg)
                positions[i] = int((pos_deg - minrange_deg) / (maxrange_deg - minrange_deg) *
                                   (maxrange_raw-minrange_raw) + minrange_raw)

        for i in range(len(positions)):
            if self.motor_enabled[i]:
                param_goal_position = [0, 0, 0, 0]
                pos = int(positions[i])
                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos))
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos))
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos))
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos))
                self.dynamixel_groupSyncWrite_position.addParam(self.joint_motor_mapping[i], param_goal_position)

        dxl_comm_result = self.dynamixel_groupSyncWrite_position.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))

        self.dynamixel_groupSyncWrite_position.clearParam()

    def set_pos_single(self, motor_id, position, unit=None, margin_pct=0.05):
        """ Set the position of a single motors.

        :param motor_id: id of the motor to control (Finger.x)
        :type motor_id: int

        :param position: position value for each motor
        :type position: int

        :param unit: unit of the values in the position vector (Unit.RAW, Unit.NORMALIZED, Unit.DEGREES).
                     If None, the default unit is used, as selected in the constructor.
        :type unit: Unit, optional

        :param margin_pct: if normalized positions are chosen, then values from [0,1] are renormalized to
                           [margin_pct, 1-margin_pct] to decrease the likelihood of self collisions.
        0 corresponds to open, 1 to closed (e.g., in flex joints)

        :type margin_pct: float, optional
        """
        if not self._assert_connected():
            return

        unit = _convert_unit(unit)
        if unit == Unit.RAW or (unit is None and self.default_unit == Unit.RAW):
            position = int(_clip(position,
                                 self.motor_calib_min_range_ticks[motor_id],
                                 self.motor_calib_max_range_ticks[motor_id]))
        elif unit == Unit.NORMALIZED or (unit is None and self.default_unit == Unit.NORMALIZED):
            position = _np_clip(position, 0, 1)
            min_val = self.motor_calib_min_range_ticks[motor_id]*(1.0+margin_pct)
            max_val = self.motor_calib_max_range_ticks[motor_id]*(1.0-margin_pct)
            position = int(position*(max_val-min_val) + min_val)
        elif unit == Unit.DEGREES or (unit is None and self.default_unit == Unit.DEGREES):
            minrange_deg = self.motor_calib_min_range_deg[motor_id]
            maxrange_deg = self.motor_calib_max_range_deg[motor_id]
            minrange_raw = self.motor_calib_min_range_ticks[motor_id]*(1.0+margin_pct)
            maxrange_raw = self.motor_calib_max_range_ticks[motor_id]*(1.0-margin_pct)
            position = _clip(position, minrange_deg, maxrange_deg)
            position = int((position - minrange_deg) / (maxrange_deg - minrange_deg) *
                           (maxrange_raw-minrange_raw) + minrange_raw)

        dxl_comm_result, dxl_error = self.dynamixel_packetHandler.write4ByteTxRx(self.dynamixel_portHandler,
                                                                                 self.joint_motor_mapping[motor_id],
                                                                                 address=ADDRESS_GOAL_POSITION,
                                                                                 data=position)
        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))

    def get_encoder_vector(self, unit=None):
        """ Get the position of all motors at the same time.

        :param unit: unit of the values in the position vector (Unit.RAW, Unit.NORMALIZED, Unit.DEGREES).
                     If None, the default unit is used, as selected in the constructor.
        :type unit: Unit, optional

        :return: the encoder values for all motors, converted to the desired unit.
        :rtype: int or float
        """
        positions = [512]*self.n_motors

        if self._assert_connected():
            for i in range(self.n_motors):
                if self.motor_enabled[i]:
                    self.dynamixel_groupSyncRead_position.addParam(self.joint_motor_mapping[i])

            dxl_comm_result = self.dynamixel_groupSyncRead_position.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("ERROR: get_all_encoders %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))

            for i in range(self.n_motors):
                if self.motor_enabled[i]:
                    dxl_getdata_result = self.dynamixel_groupSyncRead_position.isAvailable(self.joint_motor_mapping[i],
                                                                                           ADDRESS_PRESENT_POSITION,
                                                                                           4)
                    if not dxl_getdata_result:
                        print("[ID:%03d] groupSyncRead getdata failed" % self.joint_motor_mapping[i])
                    else:
                        present_position = self.dynamixel_groupSyncRead_position.getData(self.joint_motor_mapping[i],
                                                                                         ADDRESS_PRESENT_POSITION,
                                                                                         4)
                        positions[i] = present_position

            self.dynamixel_groupSyncRead_position.clearParam()

        # Encoder readings are 'raw'
        unit = _convert_unit(unit)
        if unit == Unit.NORMALIZED or (unit is None and self.default_unit == Unit.NORMALIZED):
            for i in range(self.n_motors):
                positions[i] = (float(positions[i]) - self.motor_calib_min_range_ticks[i]) / \
                               (self.motor_calib_max_range_ticks[i]-self.motor_calib_min_range_ticks[i])
        elif unit == Unit.DEGREES or (unit is None and self.default_unit == Unit.DEGREES):
            for i in range(self.n_motors):
                positions[i] = (float(positions[i]) - self.motor_calib_min_range_deg[i]) / \
                               (self.motor_calib_max_range_deg[i]-self.motor_calib_min_range_deg[i])

        return positions

    def get_encoder_single(self, motor_id, unit=None):
        """ Get the position of a single motor.

        :param motor_id: id of the motor (Finger.x)
        :type motor_id: int

        :param unit: unit of the value for the position (Unit.RAW, Unit.NORMALIZED, Unit.DEGREES).
                     If None, the default unit is used, as selected in the constructor.
        :type unit: Unit, optional

        :return: the encoder value for the motor with motor_id, converted to the desired unit.
        :rtype: int or float
        """
        if self._assert_connected():
            dxl_present_position, dxl_comm_result, dxl_error = \
                self.dynamixel_packetHandler.read4ByteTxRx(self.dynamixel_portHandler,
                                                           self.joint_motor_mapping[motor_id],
                                                           ADDRESS_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("ERROR: ENCODERS: %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
                return 512
            elif dxl_error != 0:
                print("ERROR: ENCODERS: %s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))
                return 512
        else:
            dxl_present_position = 512

        # Encoder readings are 'raw'
        unit = _convert_unit(unit)
        if unit == Unit.NORMALIZED or (unit is None and self.default_unit == Unit.NORMALIZED):
            dxl_present_position = (float(dxl_present_position) - self.motor_calib_min_range_ticks[motor_id]) / \
                                   (self.motor_calib_max_range_ticks[motor_id] -
                                    self.motor_calib_min_range_ticks[motor_id])
        elif unit == Unit.DEGREES or (unit is None and self.default_unit == Unit.DEGREES):
            dxl_present_position = (float(dxl_present_position) - self.motor_calib_min_range_deg[motor_id]) / \
                                   (self.motor_calib_max_range_deg[motor_id]-self.motor_calib_min_range_deg[motor_id])

        return dxl_present_position

    def set_current_vector(self, currents):
        """ Set the target current/force for all motors at the same time (the vector must have 16 or 18 components).

        :param currents: vector with a current value for each motor (raw in [-1700, 1700], roughly corresponding to
                         1mA per tick).
        :type currents: list
        """
        if not self._assert_connected():
            return

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                param_goal_current = [DXL_LOBYTE(currents[i]), DXL_HIBYTE(currents[i])]
                self.dynamixel_groupSyncWrite_current.addParam(self.joint_motor_mapping[i], param_goal_current)

        dxl_comm_result = self.dynamixel_groupSyncWrite_current.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))

        self.dynamixel_groupSyncWrite_current.clearParam()

    def set_current_single(self, motor_id, current):
        """ Set the target current/force for a single motor.

        :param motor_id: id of the motor (Finger.x)
        :type motor_id: int

        :param current: vector with a current value for the selected motor (raw in [-1700, 1700], roughly corresponding
                        to 1mA per tick).
        :type current: int
        """
        if not self._assert_connected():
            return

        dxl_comm_result, dxl_error = self.dynamixel_packetHandler.write2ByteTxRx(self.dynamixel_portHandler,
                                                                                 self.joint_motor_mapping[motor_id],
                                                                                 address=ADDRESS_GOAL_CURRENT,
                                                                                 data=current)
        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("ERROR: setposall %s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))

    def get_current_vector(self):
        """ Get the current measurement (roughly indicative of torque) for all motors at the same time. This can be used to
        detect torque/forces applied to each motor.

        :return: the value of current measurements for all motors (roughly corresponding to 1mA per tick).
        :rtype: int
        """
        currents = [0]*self.n_motors

        if not self._assert_connected():
            return currents

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                self.dynamixel_groupSyncRead_current.addParam(self.joint_motor_mapping[i])

        dxl_comm_result = self.dynamixel_groupSyncRead_current.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: get_all_currents %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                dxl_getdata_result = self.dynamixel_groupSyncRead_current.isAvailable(self.joint_motor_mapping[i],
                                                                                      ADDRESS_PRESENT_CURRENT,
                                                                                      2)
                if not dxl_getdata_result:
                    print("[ID:%03d] groupSyncRead getdata failed" % self.joint_motor_mapping[i])
                else:
                    present_current = self.dynamixel_groupSyncRead_current.getData(self.joint_motor_mapping[i],
                                                                                   ADDRESS_PRESENT_CURRENT,
                                                                                   2)
                    currents[i] = ctypes.c_short(present_current).value

        self.dynamixel_groupSyncRead_current.clearParam()
        return currents

    def get_velocity_vector(self, unit=None):
        """ Get the velocity of all motors at the same time.

        :param unit: unit of the values in the position vector (Unit.RAW, Unit.DEG_PER_SECOND, Unit.REV_PER_MINUTE).
                     If None, RAW is selected. Raw values are ~ 0.229 rev/min (1.374 deg/s) per tick.
        :type unit: Unit, optional

        :return: the velocity values for all motors, converted to the desired unit.
        :rtype: int or float
        """
        velocities = [512]*self.n_motors

        if self._assert_connected():
            for i in range(self.n_motors):
                if self.motor_enabled[i]:
                    self.dynamixel_groupSyncRead_velocity.addParam(self.joint_motor_mapping[i])

            dxl_comm_result = self.dynamixel_groupSyncRead_velocity.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("ERROR: get_all_encoders %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))

            for i in range(self.n_motors):
                if self.motor_enabled[i]:
                    dxl_getdata_result = self.dynamixel_groupSyncRead_velocity.isAvailable(self.joint_motor_mapping[i],
                                                                                           ADDRESS_PRESENT_VELOCITY,
                                                                                           4)
                    if not dxl_getdata_result:
                        print("[ID:%03d] groupSyncRead getdata failed" % self.joint_motor_mapping[i])
                    else:
                        present_velocity = self.dynamixel_groupSyncRead_velocity.getData(self.joint_motor_mapping[i],
                                                                                         ADDRESS_PRESENT_VELOCITY,
                                                                                         4)
                        velocities[i] = present_velocity

            self.dynamixel_groupSyncRead_velocity.clearParam()

        unit = _convert_unit(unit)

        if unit == Unit.DEG_PER_SECOND:
            for i in range(self.n_motors):
                velocities[i] = float(velocities[i]) * 1.374
        elif unit == Unit.REV_PER_MINUTE:
            for i in range(self.n_motors):
                velocities[i] = float(velocities[i]) * 0.229
        return velocities

    def get_temperature_single(self, motor_id):
        """ Get the temperature reading for a single motor, expressed in degrees Celsius.

        :param motor_id: id of the motor (Finger.x)
        :type motor_id: int
        """
        if not self._assert_connected():
            return 0

        dxl_present_temperature, dxl_comm_result, dxl_error = \
            self.dynamixel_packetHandler.read1ByteTxRx(self.dynamixel_portHandler,
                                                       self.joint_motor_mapping[motor_id],
                                                       address=ADDRESS_PRESENT_TEMPERATURE)

        if dxl_comm_result != COMM_SUCCESS:
            print("ERROR: TEMPERATURE: %s" % self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
            return -1
        elif dxl_error != 0:
            print("ERROR: TEMPERATURE: %s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))
            return -1

        return dxl_present_temperature

    def set_torques(self, value=True):
        """ Turn on/off the torque for all motors. Note that this is called automatically in connect()/disconnect().
        Motors will ignore movement commands if torque is not enabled.

        :param value: True/False to enable/disable
        :type value: boolean or int
        """
        if not self._assert_connected():
            return

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                dxl_comm_result, dxl_error = self.dynamixel_packetHandler.write1ByteTxRx(self.dynamixel_portHandler,
                                                                                         self.joint_motor_mapping[i],
                                                                                         address=ADDRESS_TORQUE_ENABLE,
                                                                                         data=(1 if value else 0))
                if dxl_comm_result != COMM_SUCCESS:
                    print("ERROR: torque enable/disable ",
                          self.dynamixel_packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("ERROR: torque enable/disable %s" % self.dynamixel_packetHandler.getRxPacketError(dxl_error))

    def set_leds(self, value=True):
        """ Turn on/off the LEDs for all motors. Note that this is called automatically in connect()/disconnect().

        :param value: True/False to enable/disable
        :type value: boolean or int
        """
        if not self._assert_connected():
            return

        for i in range(self.n_motors):
            if self.motor_enabled[i]:
                dxl_comm_result, dxl_error = self.dynamixel_packetHandler.write1ByteTxRx(self.dynamixel_portHandler,
                                                                                         self.joint_motor_mapping[i],
                                                                                         address=ADDRESS_LED,
                                                                                         data=(1 if value else 0))

    def goto_zero_position(self):
        """ Utility method to move all motors to their default zero position.
        """
        if not self._assert_connected():
            return

        positions = self.motor_calib_zero_pos_ticks
        self.set_pos_vector(positions, Unit.RAW)
