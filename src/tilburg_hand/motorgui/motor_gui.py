"""
Motor control GUI for the Tilburg Hand v1.0.
"""

import sys
import os
import tkinter as tk
import json
from tkinter import Button, Label, Tk, DISABLED, StringVar, Scale

from tilburg_hand import TilburgHandMotorInterface, Finger, Wrist, Unit

SLIDERS_LENGTH = 100
SLIDERS_RESOLUTION = 0.01
UPDATE_EVERY_MS = 100

joint_rows_table = 4
column_space = "    "

# During calibration, movements are converted to [0, 4095]
calibrating_motor = [False]*18
recorded_motor_positions = [None]*18

enable_motors = True
motor_id_enabled = [True]*18


# GUI functions
def make_joint_element(window, motors, motors_initial_position, tilburg_hand_calibration_file_path,
                       name, row, col, joint, invert_scale=False):
    lbl = Label(window, text=name, anchor="center")
    lbl.grid(column=col, row=row)

    state_text = StringVar()
    state_text.set("[STATE]")
    state = Label(window, textvariable=state_text)
    state.grid(column=col, row=row+1)

    # slider shows feedback of current pos, and when moved sends new commands to the motors
    from_ = motors.motor_calib_max_range_ticks[joint]
    to_ = motors.motor_calib_min_range_ticks[joint]
    if invert_scale:
        from_ = motors.motor_calib_min_range_ticks[joint]
        to_ = motors.motor_calib_max_range_ticks[joint]

    slider = Scale(window, from_=from_, to=to_, resolution=SLIDERS_RESOLUTION, length=SLIDERS_LENGTH)
    slider['command'] = lambda value: move_motor(motors, joint, value)
    slider.set(motors_initial_position[joint])
    slider.grid(column=col, row=row+2)

    # btn = Button(window, text="c")
    # btn['command'] = lambda: calibrate_motor(motors, calibrating_motor, recorded_motor_positions,
    #                                          tilburg_hand_calibration_file_path, joint, btn, slider)
    # btn.grid(column=col, row=row+3)

    return state_text, slider


def calibrate_motor(motors, calibrating_motor, recorded_motor_positions, tilburg_hand_calibration_file_path,
                    joint, btn, slider):
    if not motor_id_enabled[joint]:
        return

    if btn['text'] == 'stop':
        calibrating_motor[joint] = False
        btn['text'] = "c"

        if len(recorded_motor_positions[joint]) > 0:
            motors.motor_calib_min_range_ticks[joint] = min(recorded_motor_positions[joint])
            motors.motor_calib_max_range_ticks[joint] = max(recorded_motor_positions[joint])

        slider['from_'] = motors.motor_calib_max_range_ticks[joint]
        slider['to'] = motors.motor_calib_min_range_ticks[joint]

        print("New calibrated ranges: ",
              motors.motor_calib_min_range_ticks[joint],
              motors.motor_calib_max_range_ticks[joint])

        motors.save_calibration(tilburg_hand_calibration_file_path)
    else:
        calibrating_motor[joint] = True
        recorded_motor_positions[joint] = []

        slider['from_'] = 4095
        slider['to'] = 0

        btn['text'] = "stop"


def set_new_initial_position(motors, tilburg_hand_calibration_file_path):
    motors.motor_calib_zero_pos_ticks = motors.get_encoder_vector(unit=Unit.RAW)
    motors.save_calibration(tilburg_hand_calibration_file_path)


def goto_initial_position(motors, all_sliders):
    for i in range(len(all_sliders)):
        all_sliders[i].set(motors.motor_calib_zero_pos_ticks[i])


def move_motor(motors, joint, value):
    position = int(float(value))
    if enable_motors and motor_id_enabled[joint]:
        motors.set_pos_single(joint, position, unit=Unit.RAW)


def update_loop(window, motors, motors_connected, calibrating_motor, recorded_motor_positions, all_states,
                motor_id_enabled):
    if motors_connected:
        # Get encoder value of any motor being calibrated
        for i in range(len(calibrating_motor)):
            if calibrating_motor[i]:
                pos = motors.get_encoder_single(i, unit=Unit.RAW)

                if len(recorded_motor_positions[i]) > 0 and pos != recorded_motor_positions[i][-1]:
                    print(pos)

                recorded_motor_positions[i].append(pos)

        currents = motors.get_current_vector()
        positions = motors.get_encoder_vector(unit=Unit.RAW)

        for i in range(len(all_states)):
            if motor_id_enabled[i]:
                all_states[i].set('C: '+str(currents[i])+'\nP: '+str(positions[i]))

    # update every k ms
    window.after(UPDATE_EVERY_MS, lambda: update_loop(window, motors, motors_connected, calibrating_motor,
                                                      recorded_motor_positions,
                                                      all_states, motor_id_enabled))


def quit(motors, motors_connected):
    print("Terminating...")
    if motors_connected:
        print("Disconnecting Dynamixel...")
        motors.disconnect()
    sys.exit()


def run():
    global enable_motors
    global motor_id_enabled

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
    else:
        config_folder = sys.argv[1]
        calibration_folder = sys.argv[1]

    tilburg_hand_config_file_path = os.path.join(config_folder, "config.json")
    tilburg_hand_calibration_file_path = os.path.join(calibration_folder, "calibration.json")

    calibration = None
    if os.path.exists(tilburg_hand_calibration_file_path):
        with open(tilburg_hand_calibration_file_path) as calib_file:
            calibration = json.load(calib_file)

    if not os.path.exists(tilburg_hand_config_file_path):
        print("ERROR: configuration file ", tilburg_hand_config_file_path, " not found.")
        sys.exit()

    global hand_orientation
    hand_orientation = ""
    if calibration is None or calibration.get('hand_orientation', '') == '':
        print("Calibration file [", tilburg_hand_calibration_file_path, "] does not exist: creating one.")
        popup = Tk()
        popup.title("No config found")
        popup.geometry('300x100')
        frame = tk.Frame(popup)
        frame.pack()
        lbl = Label(frame, text="No config found; generating new config:")
        lbl.pack(side=tk.TOP)

        def _assign(value, win):
            global hand_orientation
            hand_orientation = value
            win.destroy()
        btn1 = tk.Button(frame, text="Left Hand",
                         command=lambda: _assign("left", popup))
        btn1.pack(side=tk.LEFT)
        btn2 = tk.Button(frame, text="Right Hand",
                         command=lambda: _assign("right", popup))
        btn2.pack(side=tk.RIGHT)
        popup.mainloop()
    elif calibration is not None:
        hand_orientation = calibration.get('hand_orientation', '')

    motors = TilburgHandMotorInterface(config_file=tilburg_hand_config_file_path,
                                       calibration_file=tilburg_hand_calibration_file_path,
                                       hand_orientation=hand_orientation,
                                       verbose=False)

    if (not motors.device_port_found):
        enable_motors = False

    # Connect to the dynamixelsdk motor board (if enabled) and scan for all motors, to check which ones are connected
    motors_connected = False

    motor_id_enabled = [False]*motors.n_motors
    motors_initial_position = [2000]*motors.n_motors

    if enable_motors:
        ret = motors.connect()
        if ret > 0:
            motors_connected = True
            motor_id_enabled = motors.motor_enabled
            motors_initial_position = motors.motor_calib_zero_pos_ticks

            # from tilburg_hand import OperatingMode
            # motors.set_operating_mode(mode=OperatingMode.CURRENT_BASED_POSITION)

            print('Motors enabled and successful connection.\n\n\n')
        else:
            print('Problem connecting to the motor controller.\n\n\n')
            enable_motors = False

    # Build GUI
    window = Tk()
    window.title("Tilburg Hand Motor control GUI")
    window.geometry('1000x450')

    lbl = Label(window, text="Thumb", font=('Arial', 11, 'bold'))
    lbl.grid(column=1, row=0)

    thumb_ip_state, thumb_ip_slider = make_joint_element(window, motors, motors_initial_position,
                                                         tilburg_hand_calibration_file_path,
                                                         "ip-flex", 1, 0, Finger.THUMB_IP)
    thumb_mcp_state, thumb_mcp_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "mcp-flex", 1, 1, Finger.THUMB_MCP, invert_scale=False)
    thumb_abd_state, thumb_abd_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "abd", 1, 2, Finger.THUMB_ABD)
    thumb_cmc_state, thumb_cmc_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "cmc", 1, 3, Finger.THUMB_CMC)

    lbl = Label(window, text=column_space)
    lbl.grid(column=4, row=0)

    cur_col = 6
    lbl = Label(window, text="Index", font=('Arial', 11, 'bold'))
    lbl.grid(column=cur_col+1, row=0)

    index_dip_state, index_dip_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "dip-flex", 1, cur_col, Finger.INDEX_DIP)
    index_pip_state, index_pip_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "pip-flex", 1, cur_col+1, Finger.INDEX_PIP)
    index_mcp_state, index_mcp_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "mcp-flex", 1, cur_col+2, Finger.INDEX_MCP,
                                                           invert_scale=False)
    index_abd_state, index_abd_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "abd", 1, cur_col+3, Finger.INDEX_ABD)

    lbl = Label(window, text=column_space)
    lbl.grid(column=cur_col+4, row=0)

    cur_col = 12
    # thumb_col_start = cur_col
    lbl = Label(window, text="Middle", font=('Arial', 11, 'bold'))
    lbl.grid(column=cur_col+1, row=0)

    middle_dip_state, middle_dip_slider = make_joint_element(window, motors, motors_initial_position,
                                                             tilburg_hand_calibration_file_path,
                                                             "dip-flex", 1, cur_col, Finger.MIDDLE_DIP)
    middle_pip_state, middle_pip_slider = make_joint_element(window, motors, motors_initial_position,
                                                             tilburg_hand_calibration_file_path,
                                                             "pip-flex", 1, cur_col+1, Finger.MIDDLE_PIP)
    middle_mcp_state, middle_mcp_slider = make_joint_element(window, motors, motors_initial_position,
                                                             tilburg_hand_calibration_file_path,
                                                             "mcp-flex", 1, cur_col+2, Finger.MIDDLE_MCP,
                                                             invert_scale=False)
    middle_abd_state, middle_abd_slider = make_joint_element(window, motors, motors_initial_position,
                                                             tilburg_hand_calibration_file_path,
                                                             "abd", 1, cur_col+3, Finger.MIDDLE_ABD)

    cur_col = 0
    cur_row = joint_rows_table + 2

    lbl = Label(window, text=column_space)
    lbl.grid(column=cur_col, row=cur_row)
    cur_row += 1

    lbl = Label(window, text="Ring", font=('Arial', 11, 'bold'))
    lbl.grid(column=cur_col+1, row=cur_row)

    ring_dip_state, ring_dip_slider = make_joint_element(window, motors, motors_initial_position,
                                                         tilburg_hand_calibration_file_path,
                                                         "dip-flex", cur_row+1, cur_col, Finger.RING_DIP)
    ring_pip_state, ring_pip_slider = make_joint_element(window, motors, motors_initial_position,
                                                         tilburg_hand_calibration_file_path,
                                                         "pip-flex", cur_row+1, cur_col+1, Finger.RING_PIP)
    ring_mcp_state, ring_mcp_slider = make_joint_element(window, motors, motors_initial_position,
                                                         tilburg_hand_calibration_file_path,
                                                         "mcp-flex", cur_row+1, cur_col+2, Finger.RING_MCP,
                                                         invert_scale=False)
    ring_abd_state, ring_abd_slider = make_joint_element(window, motors, motors_initial_position,
                                                         tilburg_hand_calibration_file_path,
                                                         "abd", cur_row+1, cur_col+3, Finger.RING_ABD)

    lbl = Label(window, text=column_space)
    lbl.grid(column=cur_col+4, row=0)

    cur_col = 7
    lbl = Label(window, text="Wrist", font=('Arial', 11, 'bold'))
    lbl.grid(column=cur_col+1, row=cur_row)

    wrist_yaw_state, wrist_yaw_slider = make_joint_element(window, motors, motors_initial_position,
                                                           tilburg_hand_calibration_file_path,
                                                           "yaw", cur_row+1, cur_col, Wrist.YAW)
    wrist_pitch_state, wrist_pitch_slider = make_joint_element(window, motors, motors_initial_position,
                                                               tilburg_hand_calibration_file_path,
                                                               "pitch", cur_row+1, cur_col+1, Wrist.PITCH)

    lbl = Label(window, text="  ")
    lbl.grid(column=cur_col+2, row=0)

    all_sliders = [thumb_ip_slider, thumb_mcp_slider, thumb_abd_slider, thumb_cmc_slider, index_dip_slider,
                   index_pip_slider, index_mcp_slider, index_abd_slider, middle_dip_slider, middle_pip_slider,
                   middle_mcp_slider, middle_abd_slider, ring_dip_slider, ring_pip_slider, ring_mcp_slider,
                   ring_abd_slider, wrist_yaw_slider, wrist_pitch_slider]
    all_states = [thumb_ip_state, thumb_mcp_state, thumb_abd_state, thumb_cmc_state, index_dip_state,
                  index_pip_state, index_mcp_state, index_abd_state, middle_dip_state, middle_pip_state,
                  middle_mcp_state, middle_abd_state, ring_dip_state, ring_pip_state, ring_mcp_state,
                  ring_abd_state, wrist_yaw_state, wrist_pitch_state]

    for i in range(len(all_sliders)):
        if not motor_id_enabled[i]:
            all_sliders[i].config(state=DISABLED, takefocus=0)
            all_sliders[i]['bg'] = '#999999'

    btn = Button(window, text="Set zero", command=lambda: set_new_initial_position(motors,
                                                                                   tilburg_hand_calibration_file_path))
    btn.grid(column=12, row=9)

    btn = Button(window, text="Go to zero", command=lambda: goto_initial_position(motors, all_sliders))
    btn.grid(column=12, row=10)
    goto_initial_position(motors, all_sliders)

    window.protocol("WM_DELETE_WINDOW", lambda: quit(motors, motors_connected))
    window.after(10, lambda: update_loop(window, motors, motors_connected, calibrating_motor, recorded_motor_positions,
                                         all_states, motor_id_enabled))
    window.mainloop()


if __name__ == "__main__":
    run()
