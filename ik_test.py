import matplotlib.pyplot as plt
import math

# Lengths of the robotic arm links
L1 = 100.0  # Length of the first link
L2 = 100.0  # Length of the second link
L3 = 100.0  # Length of the third link

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

def plot_robot_arm(x, y, phi):
    try:
        theta1, theta2, theta3 = inverse_kinematics(x, y, phi)
        
        # Convert angles to radians
        theta1_rad = math.radians(theta1)
        theta2_rad = math.radians(theta2)
        theta3_rad = math.radians(theta3)
        
        # Calculate joint positions
        joint1_x = L1 * math.cos(theta1_rad)
        joint1_y = L1 * math.sin(theta1_rad)
        
        joint2_x = joint1_x + L2 * math.cos(theta1_rad + theta2_rad)
        joint2_y = joint1_y + L2 * math.sin(theta1_rad + theta2_rad)
        
        end_effector_x = joint2_x + L3 * math.cos(theta1_rad + theta2_rad + theta3_rad)
        end_effector_y = joint2_y + L3 * math.sin(theta1_rad + theta2_rad + theta3_rad)
        
        # Plotting
        plt.figure()
        plt.plot([0, joint1_x], [0, joint1_y], 'ro-', lw=2)
        plt.plot([joint1_x, joint2_x], [joint1_y, joint2_y], 'go-', lw=2)
        plt.plot([joint2_x, end_effector_x], [joint2_y, end_effector_y], 'bo-', lw=2)
        
        plt.scatter([0, joint1_x, joint2_x, end_effector_x], [0, joint1_y, joint2_y, end_effector_y])
        plt.text(0, 0, 'Base', fontsize=12, ha='right')
        plt.text(joint1_x, joint1_y, 'J1', fontsize=12, ha='right')
        plt.text(joint2_x, joint2_y, 'J2', fontsize=12, ha='right')
        plt.text(end_effector_x, end_effector_y, 'EE', fontsize=12, ha='right')
        
        plt.xlim(-L1-L2-L3-10, L1+L2+L3+10)
        plt.ylim(-L1-L2-L3-10, L1+L2+L3+10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.title(f'Robot Arm Configuration\n(x={x}, y={y}, φ={phi}°)')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
    except ValueError as e:
        print(f"Error: {e}")

# Test the inverse kinematics and plot the result
test_x = 150
test_y = 50
test_phi = 30  # End-effector orientation angle in degrees

plot_robot_arm(test_x, test_y, test_phi)