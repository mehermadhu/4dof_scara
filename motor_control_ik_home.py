import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import time
import math
import matplotlib.pyplot as plt
import configparser

# Conversion factor based on 360 degrees = 4000 pulses
DEGREES_TO_PULSES = 4000 / 360

# Lengths of the robotic arm links
L1 = 137.0  # Length of the first link in mm
L2 = 121.0  # Length of the second link in mm
L3 = 40.0   # Length of the third link in mm

# Home position angles
home_theta1 = 0.0
home_theta2 = 0.0
home_theta3 = 0.0

# Configuration file path
CONFIG_FILE = 'robot_config.ini'

def load_home_positions():
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE)
    
    global home_theta1, home_theta2, home_theta3
    if 'HomePosition' in config:
        home_theta1 = float(config['HomePosition'].get('theta1', 0.0))
        home_theta2 = float(config['HomePosition'].get('theta2', 0.0))
        home_theta3 = float(config['HomePosition'].get('theta3', 0.0))
    else:
        config['HomePosition'] = {'theta1': home_theta1, 'theta2': home_theta2, 'theta3': home_theta3}
        with open(CONFIG_FILE, 'w') as configfile:
            config.write(configfile)

def save_home_positions():
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE)
    config['HomePosition'] = {'theta1': home_theta1, 'theta2': home_theta2, 'theta3': home_theta3}
    with open(CONFIG_FILE, 'w') as configfile:
        config.write(configfile)

def send_can_command(command):
    print(f"Sending CAN command: {command}")
    os.system(f'cansend can0 {command}')

def degrees_to_pulses(degrees):
    return int(degrees * DEGREES_TO_PULSES)

def convert_to_little_endian(value):
    hex_value = f"{value:08X}"
    little_endian_value = ''.join([hex_value[i:i+2] for i in range(6, -1, -2)])
    return little_endian_value

def convert_to_little_endian_signed(value):
    if value < 0:
        value = (1 << 32) + value
    hex_value = f"{value:08X}"
    little_endian_value = ''.join([hex_value[i:i+2] for i in range(6, -1, -2)])
    return little_endian_value

def inverse_kinematics(x, y, phi):
    # phi is the orientation angle of the end-effector
    wx = x - L3 * math.cos(math.radians(phi))
    wy = y - L3 * math.sin(math.radians(phi))
    
    D = (wx**2 + wy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if D > 1 or D < -1:
        raise ValueError("Position is unreachable")
    
    theta2 = math.atan2(math.sqrt(1 - D**2), D)
    theta1 = math.atan2(wy, wx) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta3 = math.radians(phi) - theta1 - theta2

    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)

def initialize_motors():
    try:
        load_home_positions()
        accel_time = int(entry_accel_time.get())
        decel_time = int(entry_decel_time.get())
        max_speed = int(entry_max_speed.get())

        accel_command_le = convert_to_little_endian(accel_time)
        decel_command_le = convert_to_little_endian(decel_time)
        speed_command_le = convert_to_little_endian(max_speed)
        
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B40600000000000")  # Initialization step 0
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600006000000")  # Initialization step 1
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600007000000")  # Initialization step 2
            time.sleep(0.5)
            send_can_command(f"{node}#2B4060000F000000")  # Initialization step 3
            time.sleep(0.5)
            send_can_command(f"{node}#23836000{accel_command_le}")  # Set acceleration time (index 6083)
            time.sleep(0.5)
            send_can_command(f"{node}#23846000{decel_command_le}")  # Set deceleration time (index 6084)
            time.sleep(0.5)
            send_can_command(f"{node}#23816000{speed_command_le}")  # Set maximum speed (index 6081)
            time.sleep(0.5)
            send_can_command(f"{node}#2F60600001000000")  # Switch to position mode (index 6060)
            time.sleep(0.5)
        
        messagebox.showinfo("Initialize Motors", "Motors have been initialized and are ready for operation.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for acceleration, deceleration, and speed.")

def shutdown_motors():
    try:
        # Move to home position
        pulses1 = degrees_to_pulses(home_theta1)
        pulses2 = degrees_to_pulses(home_theta2)
        pulses3 = degrees_to_pulses(home_theta3)

        pulse_command1_le = convert_to_little_endian_signed(pulses1)
        pulse_command2_le = convert_to_little_endian_signed(pulses2)
        pulse_command3_le = convert_to_little_endian_signed(pulses3)

        start_command = "0F"
        execute_command = "1F"

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.001)
            send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
            time.sleep(0.001)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.001)
        
        time.sleep(5)  # Wait for the robot to reach home position

        # Send shutdown command
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B40600000000000")  # Shutdown command
            time.sleep(0.5)
        messagebox.showinfo("Shutdown Motors", "Motors have been moved to home position and shut down properly.")
    except Exception as e:
        messagebox.showerror("Error", str(e))

def send_position_command():
    try:
        x = float(entry_x.get())
        y = float(entry_y.get())
        phi = float(entry_phi.get())
        
        theta1, theta2, theta3 = inverse_kinematics(x, y, phi)

        # Apply home position offsets
        theta1 += home_theta1
        theta2 += home_theta2
        theta3 += home_theta3

        pulses1 = degrees_to_pulses(theta1)
        pulses2 = degrees_to_pulses(theta2)
        pulses3 = degrees_to_pulses(theta3)

        pulse_command1_le = convert_to_little_endian_signed(pulses1)
        pulse_command2_le = convert_to_little_endian_signed(pulses2)
        pulse_command3_le = convert_to_little_endian_signed(pulses3)

        start_command = "0F"
        execute_command = "1F"

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.001)
            send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
            time.sleep(0.001)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.001)
        
        messagebox.showinfo("Send Position Command", "Position commands sent to all motors.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for positions.")
    except Exception as e:
        messagebox.showerror("Error", str(e))

def send_joint_angles_command():
    try:
        theta1 = float(entry_joint1.get())
        theta2 = float(entry_joint2.get())
        theta3 = float(entry_joint3.get())

        # Apply home position offsets
        theta1 += home_theta1
        theta2 += home_theta2
        theta3 += home_theta3

        pulses1 = degrees_to_pulses(theta1)
        pulses2 = degrees_to_pulses(theta2)
        pulses3 = degrees_to_pulses(theta3)

        pulse_command1_le = convert_to_little_endian_signed(pulses1)
        pulse_command2_le = convert_to_little_endian_signed(pulses2)
        pulse_command3_le = convert_to_little_endian_signed(pulses3)

        start_command = "0F"
        execute_command = "1F"

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.001)
            send_can_command(f"{node}#237A6000{pulse_command_le}")  # Set target position using index 607A
            time.sleep(0.001)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.001)
        
        messagebox.showinfo("Send Joint Angles Command", "Joint angles commands sent to all motors.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for joint angles.")
    except Exception as e:
        messagebox.showerror("Error", str(e))

def set_home_position():
    global home_theta1, home_theta2, home_theta3
    try:
        home_theta1 = float(entry_home_joint1.get())
        home_theta2 = float(entry_home_joint2.get())
        home_theta3 = float(entry_home_joint3.get())
        save_home_positions()
        messagebox.showinfo("Set Home Position", "Home position set successfully.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for home joint angles.")

def on_closing():
    if messagebox.askokcancel("Quit", "Do you want to quit and shut down all motors?"):
        shutdown_motors()
        root.destroy()

# GUI Setup
root = tk.Tk()
root.title("Motor Control Panel")

initialize_button = tk.Button(root, text="Initialize Motors", command=initialize_motors)
initialize_button.pack(pady=10)

shutdown_button = tk.Button(root, text="Shutdown Motors", command=shutdown_motors)
shutdown_button.pack(pady=10)

# Input fields for acceleration, deceleration, and maximum speed
tk.Label(root, text="Acceleration Time (ms):").pack()
entry_accel_time = tk.Entry(root)
entry_accel_time.insert(0, "100")
entry_accel_time.pack(pady=5)

tk.Label(root, text="Deceleration Time (ms):").pack()
entry_decel_time = tk.Entry(root)
entry_decel_time.insert(0, "100")
entry_decel_time.pack(pady=5)

tk.Label(root, text="Maximum Speed (rpm):").pack()
entry_max_speed = tk.Entry(root)
entry_max_speed.insert(0, "60")
entry_max_speed.pack(pady=5)

# Input fields for x, y positions and end-effector orientation
tk.Label(root, text="Enter x position (mm):").pack()
entry_x = tk.Entry(root)
entry_x.pack(pady=5)

tk.Label(root, text="Enter y position (mm):").pack()
entry_y = tk.Entry(root)
entry_y.pack(pady=5)

tk.Label(root, text="Enter end-effector orientation (degrees):").pack()
entry_phi = tk.Entry(root)
entry_phi.pack(pady=5)

send_absolute_position_button = tk.Button(root, text="Send Absolute Position", command=send_position_command)
send_absolute_position_button.pack(pady=10)

# Input fields for joint angles
tk.Label(root, text="Enter joint angle for Motor 1 (degrees):").pack()
entry_joint1 = tk.Entry(root)
entry_joint1.pack(pady=5)

tk.Label(root, text="Enter joint angle for Motor 2 (degrees):").pack()
entry_joint2 = tk.Entry(root)
entry_joint2.pack(pady=5)

tk.Label(root, text="Enter joint angle for Motor 3 (degrees):").pack()
entry_joint3 = tk.Entry(root)
entry_joint3.pack(pady=5)

send_absolute_joint_angles_button = tk.Button(root, text="Send Absolute Joint Angles", command=send_joint_angles_command)
send_absolute_joint_angles_button.pack(pady=10)

# Input fields for home position
tk.Label(root, text="Enter home position for Motor 1 (degrees):").pack()
entry_home_joint1 = tk.Entry(root)
entry_home_joint1.pack(pady=5)

tk.Label(root, text="Enter home position for Motor 2 (degrees):").pack()
entry_home_joint2 = tk.Entry(root)
entry_home_joint2.pack(pady=5)

tk.Label(root, text="Enter home position for Motor 3 (degrees):").pack()
entry_home_joint3 = tk.Entry(root)
entry_home_joint3.pack(pady=5)

set_home_position_button = tk.Button(root, text="Set Home Position", command=set_home_position)
set_home_position_button.pack(pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
