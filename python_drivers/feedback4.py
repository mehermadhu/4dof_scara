import os
import time
import threading
import subprocess
import signal
import sys

# Define the CAN interface and node IDs
CAN_INTERFACE = "can0"
NODE_IDS = [2, 3, 4]  # List of Node IDs for 3 motors
DELAY = 0.1  # Delay in seconds for configuration
monitoring_active = True

status_words = {node_id: "0000" for node_id in NODE_IDS}
position_values = {node_id: "0000" for node_id in NODE_IDS}

# Function to send CAN message with padding
def send_can_message(cob_id, data):
    cob_id_str = f"{cob_id:03X}"
    data_padded = data.ljust(16, '0')
    command = f"cansend {CAN_INTERFACE} {cob_id_str}#{data_padded}"
    print(f"Sending CAN message: {command}")
    os.system(command)

# Function to configure RPDO1 for a specific node
def configure_rpdo1(node_id):
    cob_id = 0x600 + node_id
    # Disable RPDO1
    send_can_message(cob_id, "2B14000080000000")
    time.sleep(DELAY)
    # Set COB-ID for RPDO1 (0x200 + Node-ID)
    rpdo1_cob_id = f"{0x200 + node_id:03X}00"
    send_can_message(cob_id, f"2B140001{rpdo1_cob_id}")
    time.sleep(DELAY)
    # Set transmission type for RPDO1 to synchronous (0x01)
    send_can_message(cob_id, "2B14000201000000")
    time.sleep(DELAY)
    # Clear existing mappings
    send_can_message(cob_id, "2F16000000000000")
    time.sleep(DELAY)
    # Map target position (0x607A:00)
    send_can_message(cob_id, "2B160001607A0020")
    time.sleep(DELAY)
    # Enable RPDO1 with new mapping
    send_can_message(cob_id, "2F16000200000000")
    time.sleep(DELAY)
    send_can_message(cob_id, "2B14000000000000")
    time.sleep(DELAY)

# Function to configure TPDO1 for a specific node
def configure_tpdo1(node_id):
    cob_id = 0x600 + node_id
    # Disable TPDO1
    send_can_message(cob_id, "2B18010080000000")
    time.sleep(DELAY)
    # Set COB-ID for TPDO1 (0x280 + Node-ID)
    tpdo1_cob_id = f"{0x280 + node_id:03X}00"
    send_can_message(cob_id, f"2B180101{tpdo1_cob_id}")
    time.sleep(DELAY)
    # Set transmission type for TPDO1 to event-driven (0x01)
    send_can_message(cob_id, "2B18010201000000")
    time.sleep(DELAY)
    # Optionally set inhibit time to 100ms (0x0064)
    send_can_message(cob_id, "2B18010300640000")
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
        if position_pulses & 0x80000000:  # Check if the sign bit is set
            position_pulses -= 0x100000000  # Convert to signed 32-bit value
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
            print(f"Debug: parts: {parts}")  # Print parts for debugging
            if len(parts) > 2:
                response_cob_id = parts[1]
                response_data = ''.join(parts[3:])
                print(f"Debug: COB-ID: {response_cob_id}, Data: {response_data}")  # Additional debug output
                for node_id in NODE_IDS:
                    expected_cob_id = f"{0x580 + node_id:03X}"
                    print(f"Debug: Comparing {response_cob_id} with expected {expected_cob_id} for node {node_id}")
                    if response_cob_id == expected_cob_id:  # SDO response (0x580 + Node ID)
                        print("MATCHED-----")
                        # Extract status word and position actual value from the response data
                        if response_data.startswith("607A"):
                            position_value = response_data[8:16]
                            print(f"Debug: Node {node_id}, Position Value: {position_value}")  # Additional debug output
                            position_values[node_id] = position_value
                            interpreted_position = interpret_position_value(position_value)
                            print(f"Node {node_id}: Position {position_value} - {interpreted_position}")
                        elif response_data.startswith("6041"):
                            status_word = response_data[8:12]
                            print(f"Debug: Node {node_id}, Status Word: {status_word}")  # Additional debug output
                            status_words[node_id] = status_word
                            interpreted_status = interpret_status_word(status_word)
                            print(f"Node {node_id}: Status {status_word} - {interpreted_status}")
        time.sleep(DELAY)
    process.terminate()

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
        print(f"Configuring RPDO1 for node {node_id}...")
        configure_rpdo1(node_id)
    print("Monitoring CAN bus for TPDOs...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)
