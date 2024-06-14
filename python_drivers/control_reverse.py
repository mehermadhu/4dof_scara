import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import time

# Conversion factor based on 360 degrees = 4000 pulses
DEGREES_TO_PULSES = 4000 / 360

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

def initialize_motors():
    try:
        accel_time = int(entry_accel_time.get())
        decel_time = int(entry_decel_time.get())
        max_speed = int(entry_max_speed.get())



        accel_command_le = convert_to_little_endian(accel_time)
        decel_command_le = convert_to_little_endian(decel_time)
        speed_command_le = convert_to_little_endian(max_speed)

        if reverse_var.get():
            max_speed = -max_speed
            speed_command_le = convert_to_little_endian_signed(max_speed)
        for node in ['602', '603', '604']:
            send_can_command(f"{node}#2B40600000000000")  # Initialize the state
            time.sleep(0.5)
            send_can_command(f"{node}#23836000{accel_command_le}")  # Set acceleration time
            time.sleep(0.5)
            send_can_command(f"{node}#23846000{decel_command_le}")  # Set deceleration time
            time.sleep(0.5)
            send_can_command(f"{node}#23816000{speed_command_le}")  # Set maximum speed
            time.sleep(0.5)
            send_can_command(f"{node}#2F60600001000000")  # Switch to position mode
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600006000000")  # Switch on
            time.sleep(0.5)
            send_can_command(f"{node}#2B40600007000000")  # Enable operation
            time.sleep(0.5)
            send_can_command(f"{node}#2B4060000F000000")  # Complete enable
            time.sleep(0.5)
        messagebox.showinfo("Initialize Motors", "Motors have been initialized and are ready for operation.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numerical values for acceleration, deceleration, and speed.")

def shutdown_motors():
    for node in ['602', '603', '604']:
        send_can_command(f"{node}#2B40600000000000")  # Shutdown command
        time.sleep(0.5)
    messagebox.showinfo("Shutdown Motors", "Motors have been shut down properly.")

def send_position_command(is_relative):
    try:
        degrees1 = float(entry_motor1.get())
        degrees2 = float(entry_motor2.get())
        degrees3 = float(entry_motor3.get())

        pulses1 = degrees_to_pulses(degrees1)
        pulses2 = degrees_to_pulses(degrees2)
        pulses3 = degrees_to_pulses(degrees3)

        pulse_command1_le = convert_to_little_endian(pulses1)
        pulse_command2_le = convert_to_little_endian(pulses2)
        pulse_command3_le = convert_to_little_endian(pulses3)

        if is_relative:
            start_command = "4F"
            execute_command = "5F"
        else:
            start_command = "0F"
            execute_command = "1F"

        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{start_command}000000")  # Prepare for motion
            time.sleep(0.1)
            send_can_command(f"{node}#237A0000{pulse_command_le}")  # Set target position using 237Ah register
            time.sleep(0.1)
        for node, pulse_command_le in zip(['602', '603', '604'], [pulse_command1_le, pulse_command2_le, pulse_command3_le]):
            send_can_command(f"{node}#2B406000{execute_command}000000")  # Execute motion
            time.sleep(0.1)
        messagebox.showinfo("Send Position Command", "Position commands sent to all motors.")
    except ValueError:
        messagebox.showerror("Invalid Input", "Please enter valid numbers for angles.")

def send_relative_position():
    send_position_command(is_relative=True)

def send_absolute_position():
    send_position_command(is_relative=False)

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

# Checkbox for reversing motor direction
reverse_var = tk.BooleanVar()
reverse_checkbox = tk.Checkbutton(root, text="Reverse Motor Direction", variable=reverse_var)
reverse_checkbox.pack(pady=5)

# Input fields for relative positions
tk.Label(root, text="Enter relative position for Motor 1 (degrees):").pack()
entry_motor1 = tk.Entry(root)
entry_motor1.pack(pady=5)

tk.Label(root, text="Enter relative position for Motor 2 (degrees):").pack()
entry_motor2 = tk.Entry(root)
entry_motor2.pack(pady=5)

tk.Label(root, text="Enter relative position for Motor 3 (degrees):").pack()
entry_motor3 = tk.Entry(root)
entry_motor3.pack(pady=5)

send_relative_position_button = tk.Button(root, text="Send Relative Position", command=send_relative_position)
send_relative_position_button.pack(pady=10)

send_absolute_position_button = tk.Button(root, text="Send Absolute Position", command=send_absolute_position)
send_absolute_position_button.pack(pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
