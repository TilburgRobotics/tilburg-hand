"""
Simple demo to show how to set a pre-defined pose on the Tilburg Hand.
"""

import sys
import os

from time import sleep

sys.path.insert(0, "../libraries")
from tilburg_hand import TilburgHandMotorInterface, Finger, Unit


config_folder = os.path.abspath(os.path.dirname(__file__))
calibration_folder = os.path.join(os.path.expanduser("~"), 'tilburg_hand')
if len(sys.argv) != 2:
    print("Usage:\n\t", sys.argv[0], " <path-to-folder-with-config>")
    print("* Config folder should have a config.json file and a calibration.json"
          " file, as used/produced by motor_gui.")
    print("*   If no argument is supplied, the default config.json file will be "
          "used (", config_folder, "), and calibration will be saved in the "
          "user's home folder (", calibration_folder, ").")
    print("\n\n")

tilburg_hand_config_file_path = os.path.join(config_folder, "config.json")
tilburg_hand_calibration_file_path = os.path.join(calibration_folder, "calibration.json")

motors = TilburgHandMotorInterface(config_file=tilburg_hand_config_file_path,
                                   calibration_file=tilburg_hand_calibration_file_path,
                                   verbose=False)

ret = motors.connect()
if ret < 0:
    print("PROBLEMS CONNECTING TO THE MOTORS' BOARD")
    sys.exit()

pos_normalized = [0]*motors.n_motors

pos_normalized[Finger.INDEX_ABD] = 0.9
pos_normalized[Finger.MIDDLE_ABD] = 0.1

pos_normalized[Finger.INDEX_DIP] = 0.0
pos_normalized[Finger.MIDDLE_DIP] = 0.0
pos_normalized[Finger.RING_DIP] = 0.9

pos_normalized[Finger.INDEX_PIP] = 0
pos_normalized[Finger.MIDDLE_PIP] = 0
pos_normalized[Finger.RING_PIP] = 0.85

pos_normalized[Finger.INDEX_MCP] = 0
pos_normalized[Finger.MIDDLE_MCP] = 0
pos_normalized[Finger.RING_MCP] = 0.85

pos_normalized[Finger.THUMB_CMC] = 0.5
pos_normalized[Finger.THUMB_ABD] = 0.2
pos_normalized[Finger.THUMB_MCP] = 0.7
pos_normalized[Finger.THUMB_IP] = 0.9

motors.set_pos_vector(pos_normalized, unit=Unit.NORMALIZED)

sleep(3)

motors.goto_zero_position()
sleep(1)

motors.disconnect()
