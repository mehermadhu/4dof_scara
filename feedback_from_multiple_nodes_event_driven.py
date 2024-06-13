import os
import time
import threading
import subprocess
import signal
import sys

# Define the CAN interface and node IDs
CAN_INTERFACE = "can0"
NODE_IDS = [2, 3, 4, 5, 6, 7]  # List of Node IDs for 6 motors
DELAY = 0.5  # Delay in seconds
monitoring_active = True

status_words = {node_id: "0000" for node_id in NODE_IDS}
position_values = {node_id: 0 for node_id in NODE_IDS}

# Function to send CAN message
def send_can_message(cob_id, data):
    command = f"cansend {CAN_INTERFACE} {cob_id}#{data}"
    os.system(command)

# Function to interpret the status word
def interpret_status_word(status_word):
    try:
        status_bits = bin(int(status_word, 16))[2:].zfill(16)
    except ValueError:
        print(f"Error: Invalid status word {status_word}")
        return "Invalid status word"
    
    state_descriptions = {
        "0000": "Not ready to switch on",
        "0001": "Switch on disabled",
        "0011": "Ready to switch on",
        "0111": "Switched on",
        "1111": "Operation enabled",
    }
    state = state_descriptions.get(status_bits[-4:], "Unknown state")
    driver_normal = "normal" if status_bits[-8] == '0' else "alarm"
    homing_completed = "not completed" if status_bits[-9] == '0' else "completed"
    bit4_state = "Bit4 state of 6040h is 0" if status_bits[-12] == '0' else "Bit4 state of 6040h is 1"
    motor_release = "released" if status_bits[-14] == '0' else "enabled"
    motor_running = "stopped" if status_bits[-15] == '0' else "running"
    motion_in_place = "not in place" if status_bits[-16] == '0' else "in place"

    return (f"State: {state}, Driver: {driver_normal}, Homing: {homing_completed}, {bit4_state}, "
            f"Motor: {motor_release}, Running: {motor_running}, Motion: {motion_in_place}")

# Function to interpret the position actual value
def interpret_position_value(position_value):
    try:
        position_pulses = int(position_value, 16)
        position_degrees = position_pulses * (360.0 / 4000.0)
    except ValueError:
        print(f"Error: Invalid position value {position_value}")
        return "Invalid position value"
    
    return f"Position: {position_pulses} pulses ({position_degrees:.2f} degrees)"

# Function to monitor CAN bus in real-time
def monitor_can_response():
    global status_words
    global position_values
    process = subprocess.Popen(['candump', CAN_INTERFACE], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    while monitoring_active:
        output = process.stdout.readline().strip()
        if output:
            print(f"Raw CAN response: {output}")  # Debugging output
            parts = output.split()
            if len(parts) > 3:
                response_cob_id = parts[1]
                response_data = parts[2:]
                for node_id in NODE_IDS:
                    if response_cob_id.startswith(f"{0x280 + node_id:03X}"):  # TPDO1 (0x280 + Node ID)
                        # Extract status word and position actual value from the response data
                        status_words[node_id] = ''.join(response_data[-4:-2])
                        position_values[node_id] = ''.join(response_data[-2:])
                        interpreted_status = interpret_status_word(status_words[node_id])
                        interpreted_position = interpret_position_value(position_values[node_id])
                        print(f"Node {node_id}: {status_words[node_id]} - {interpreted_status}, {position_values[node_id]} - {interpreted_position}")
        time.sleep(DELAY)
    process.terminate()

# Function to configure TPDO1 for a specific node
def configure_tpdo1(node_id):
    cob_id = f"{0x600 + node_id:03X}"
    # Disable TPDO1
    send_can_message(cob_id, "2B18010080000000")
    time.sleep(DELAY)
    # Set COB-ID for TPDO1 (0x280 + Node-ID)
    tpdo1_cob_id = f"{0x280 + node_id:03X}00"
    send_can_message(cob_id, f"2B18010100{tpdo1_cob_id}")
    time.sleep(DELAY)
    # Set transmission type for TPDO1 to event-driven (e.g., 0x01)
    send_can_message(cob_id, "2B18010201000000")
    time.sleep(DELAY)
    # Optionally set inhibit time to 100ms (0x0064)
    send_can_message(cob_id, "2B18010364000000")
    time.sleep(DELAY)
    # Clear existing mappings
    send_can_message(cob_id, "2F1A010000000000")
    time.sleep(DELAY)
    # Map status word (0x6041:00)
    send_can_message(cob_id, "2B1A010160410020")
    time.sleep(DELAY)
    # Map position actual value (0x6064:00)
    send_can_message(cob_id, "2B1A010260640020")
    time.sleep(DELAY)
    # Enable TPDO1 with new mapping
    send_can_message(cob_id, "2F1A010200000000")
    time.sleep(DELAY)
    send_can_message(cob_id, "2B18010000000000")
    time.sleep(DELAY)

# Signal handler for graceful exit
def signal_handler(sig, frame):
    global monitoring_active
    print('Exiting gracefully...')
    monitoring_active = False
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Start monitoring thread
monitoring_thread = threading.Thread(target=monitor_can_response, daemon=True)
monitoring_thread.start()

# Example usage to configure and monitor all nodes
if __name__ == "__main__":
    for node_id in NODE_IDS:
        print(f"Configuring TPDO1 for node {node_id}...")
        configure_tpdo1(node_id)
    print("Monitoring CAN bus for TPDOs...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)