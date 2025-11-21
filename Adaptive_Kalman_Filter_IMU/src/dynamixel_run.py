################################################################
import smbus
# from time import sleep
from numpy import arctan2
import time
import math
from numpy import savetxt
import matplotlib.pyplot as plt
from dynamixel_sdk import *  # Uses Dynamixel SDK library
###############################################Dynamixel
# Control table address
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8

# Protocol version
PROTOCOL_VERSION = 1.0  # Dynamixel Protocol 1.0 for AX-12+

# Default setting
DXL_ID = 1  # Dynamixel ID (adjust to your motor's ID)
BAUDRATE = 1000000  # Dynamixel baudrate
DEVICENAME = '/dev/ttyUSB0'  # Port where your motor is connected (adjust as needed)
TORQUE_ENABLE = 1  # Enable motor torque
TORQUE_DISABLE = 0  # Disable motor torque

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not portHandler.openPort():
    print("Failed to open the port")
    quit()

# Set port baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

# Enable torque on the motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Torque Enable Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    quit()
elif dxl_error != 0:
    print(f"Torque Enable Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
    quit()

# Set CW and CCW angle limits to allow full range (0 to 1023)
cw_angle_limit_value = 0
ccw_angle_limit_value = 1023
packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CW_ANGLE_LIMIT, cw_angle_limit_value)
packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CCW_ANGLE_LIMIT, ccw_angle_limit_value)

# Generate sine wave pattern over time
time_step = 0.01  # Time step between updates (in seconds)
frequency = 0.3  # Frequency of the sine wave (in Hz)
amplitude = (ccw_angle_limit_value - cw_angle_limit_value) / 5 # Adjusted Amplitude within angle limits
offset = (ccw_angle_limit_value + cw_angle_limit_value) / 4 # Adjusted Offset to center the sine wave
#offset=0
run_time = 20 # Total duration of the sine wave movement (in seconds)

# Lists to store data for plotting
time_list = []
position_list = []

start_time = time.time()

############################################+

while time.time() - start_time < run_time:
    elapsed_time = time.time() - start_time
    ############################Dynamixel
    # Generate sine wave value and map it to a valid motor position
    #sine_value = math.sin(2 * math.pi * frequency * elapsed_time)
    #goal_position = int(amplitude * sine_value + offset)
    
    if elapsed_time <10:
    
        step_value=offset
        
    elif elapsed_time >=10:
    
        step_value=offset*2
    
    goal_position=step_value

    # Command the motor to move to the calculated position
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
# 
#     # Check communication result and error status
#     if dxl_comm_result != COMM_SUCCESS:
#         print(f"Failed to set goal position: {packetHandler.getTxRxResult(dxl_comm_result)}")
#     elif dxl_error != 0:
#         print(f"Goal Position Error: {packetHandler.getRxPacketError(dxl_error)}")

    # Read current position
    present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID,
                                                                               ADDR_PRESENT_POSITION)
# 
#     # Check if reading the position was successful
#     if dxl_comm_result != COMM_SUCCESS:
#         print(f"Failed to read present position: {packetHandler.getTxRxResult(dxl_comm_result)}")
#     elif dxl_error != 0:
#         print(f"Present Position Error: {packetHandler.getRxPacketError(dxl_error)}")

    # Append current time and position to lists
    time_list.append(elapsed_time)
    
    position_list.append(-(present_position-265)*360/1023)


    # Wait for the next time step
    #time.sleep(time_step)


# Close port
portHandler.closePort()
savetxt("time.csv",time_list, delimiter=',')
savetxt("Encoder.csv",position_list, delimiter=',')
# Plot the recorded data



plt.figure(1)
plt.plot(time_list, position_list,color='red', linewidth=2.5, linestyle=':')
plt.title('Motor Position Over Time (Sine Wave)')
plt.grid()
plt.legend('Encoder')
plt.ylabel('Roll (degree)')
plt.xlabel('Time (s)')

plt.show()

# plt.xlim(0,50)
# plt.ylim(-50,50)
# plt.figure(2)
#
# plt.subplot(2,2,1)
# plt.plot(t,PP,color='k')
# #plt.xlabel('Time(s)')
# plt.ylabel('$P_{Trace}$')
# plt.grid(True)
# plt.subplot(2,2,2)
# plt.plot(t,RR,color='k')
# #plt.xlabel('Time(s)')
# plt.ylabel('R')
# plt.grid(True)
# plt.subplot(2,2,3)
# plt.plot(t,Lambda,color='k')
# plt.xlabel('Time (s)')
# plt.ylabel(r'$/Lambda$')
# plt.grid(True)
# plt.subplot(2,2,4)
# plt.plot(t,QQ,color='k')
# plt.xlabel('Time (s)')
# plt.ylabel('$Q_{Trace}$')
# plt.grid(True)
# print('av tim')
# print(sum(tt)/len(tt))
# print('RMS error')
# print(sum(EE))

# Disable torque after the operation
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Torque Disable Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Torque Disable Packet Error: {packetHandler.getRxPacketError(dxl_error)}")



