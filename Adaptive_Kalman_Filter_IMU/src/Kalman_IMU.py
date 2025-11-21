from numpy.linalg import multi_dot
from numpy.linalg import norm
from numpy import dot
from numpy import mean
from numpy import roll
from numpy import power
from numpy import sqrt
from numpy import identity
from numpy import zeros
from numpy import array
from numpy import loadtxt
from numpy import size
from numpy import matmul
from numpy import reshape
from numpy import pi
from numpy import trace
from numpy import shape
import matplotlib.pyplot as plt
from numpy import arange
from numpy import savetxt
# filename = 'DATA.txt'

# arr= loadtxt('D:\DATAstep.txt',delimiter=' ')
################################################################
import smbus
# from time import sleep 
from numpy import arctan2
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
##########################
def MPU_Init():

	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    
    #concatenate higher and lower value
    value = ((high << 8) | low)
        
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

def MPU():
    #Read Accelerometer raw value
    #acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
        
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    #gyro_y = read_raw_data(GYRO_YOUT_H)
    #gyro_z = read_raw_data(GYRO_ZOUT_H)
        
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    # Ax = acc_x/16384.0
    # Ay = acc_y/16384.0
    # Az = acc_z/16384.0
        
    # Gx = gyro_x/131.0
    # Gy = gyro_y/131.0
    # Gz = gyro_z/131.0

    return acc_y,acc_z,gyro_x





##################################################
# from input
# u_k = arr[150:1000, 0]
# u_k=reshape(u_k,(size(u_k),1))
# z_k = arr[150:1000, 3]
# z_k=reshape(z_k,(size(z_k),1))
# print(size(u_k))
# from last step (ini)
i_old = 0
# constants
rho = 0.5
Ts = 0.01
i = 0

F = array([[1, Ts], [0, 1]])
G = array([[Ts], [0]])
H = array([[1, Ts]])

x_k_plus_a = array([[2], [0]])
P_k_plus_a = 0.0001 * identity(2)
Q_hat = 0.01 * identity(2)
R_hat = 0.05
norm_error = 0.001
k=float(0)
vector_error = array([[0.001],[0],[0],[0],[0],[0],[0],[0]])
EE=[]
XX=[]
PP=[]
Lambda=[]
RR=[]
QQ=[]
UU=[]
z_kk = []

##################
import time
time_list = []
l1=40
l2=5
# buffer1 = deque(maxlen=l1)
# buffer2 = deque(maxlen=l2)
start_time = time.time()
run_time = 20 # Total duration of the sine wave movement (in seconds)
###################
# KalmanFilter

while time.time() - start_time < run_time:
#while True:
    elapsed_time = time.time() - start_time
    acc_y,acc_z,gyro_x = MPU()
    
    #Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0

    u_k = gyro_x/131.0
#     Gy = gyro_y/131.0
#     Gz = gyro_z/131.0
    # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
    z_k = arctan2(Ay,Az)*180/pi

    ########################################################3
    z_kk.append((z_k))
    


    
    # print(i)
    x_hat_a = x_k_plus_a
    P_k_a = P_k_plus_a

    norm_error_old = norm_error
    vector_error_old = vector_error
    ht = H.T


    P_k_minus_a = F@P_k_a@F.T + Q_hat
    x_k_minus_a = dot(F, x_hat_a) + G*u_k

    K_k_a = dot(P_k_minus_a, ht) / ((H@P_k_minus_a@ht) + R_hat)

    P_k_plus_a = dot((identity(2) - dot(K_k_a,H)), P_k_minus_a)
    x_k_plus_a = x_k_minus_a + (K_k_a*(z_k - dot(H, x_k_minus_a)))

    error = z_k - dot(H, x_k_plus_a)
    # print(F)
    norm_error = norm_error_old + power(error, 2)
    ######################### forgetting factor##########################
    k = (mean(vector_error_old) + 1*power(error, 2)) / mean(vector_error_old)

    #k=3
    vector_error = roll(vector_error_old, 1)
    vector_error[0] = power(error, 2)
    norm_error2 = norm_error_old + dot(k, power(error, 2))
    landa = norm_error2 / norm_error_old
    #landa = 1
    

#     buffer1.append(vector_error[0])
#     buffer2.append(vector_error[0])
#     Landa = np.sum(buffer) / np.sum(buffer2)


    #P_k_plus_a = landa*P_k_plus_a
    
    ######################### forgetting factor##########################
    ##Adaption for Q and R
#     deltax = x_k_plus_a - x_k_minus_a
#     deltap = P_k_plus_a - P_k_minus_a
# 
#     sigmaQ = rho * (deltax * deltax.reshape(1, -1)) + dot((1 - rho), deltap)
#     C_hat = norm_error / (i + 1)
#     Q_tilda = P_k_plus_a - multi_dot([F, P_k_a, F.transpose()]) + C_hat * (dot(K_k_a, K_k_a.reshape(1, -1)))
#     alpha = dot(dot(H, (matmul(F, matmul(P_k_a, F.transpose())) + Q_tilda)), ht) / dot(
#         dot(H, (dot(F, dot(P_k_a, F.transpose())) + Q_hat)), ht)
# #    if norm(sigmaQ) >= 0.5:
# 
#  #       Q_hat_plus = (alpha*(dot(sigmaQ / (norm(sigmaQ)), Q_hat)))
# 
# 
# #    else:
#  #       Q_hat_plus = Q_hat
#         
#     Q_hat_plus = C_hat*dot(K_k_a,K_k_a.transpose())
#     ###for R
#     sigmaR = power(error, 2) + dot(H, dot(P_k_plus_a, ht))
#     gamma = 1 / sqrt(i+5)
    EE.append(error*error)
    XX.append((x_k_plus_a[0]))
    PP.append(float(trace(P_k_plus_a)))
#     Lambda.append(float(landa))
#     RR.append(float(R_hat_plus))
#     QQ.append(float(trace(Q_hat_plus)))
    UU.append(float(u_k))



#     if norm(sigmaR) >= 20:
#         R_hat_plus = gamma * R_hat + (1 - gamma) * sigmaR
#     
#     else:
#         R_hat_plus = R_hat
    i = i + 1
    time_list.append(elapsed_time)


    
    #if i==3000:
        #break

savetxt('KF.csv',XX,delimiter=',')
savetxt('P_KF.csv',PP,delimiter=',')
#savetxt('Acc_roll_estimation.csv',z_kk,delimiter=',')








plt.figure(4)
plt.title("KF")
plt.plot(time_list,z_kk,color='0.7',linewidth=2.5,linestyle='dashed')
plt.plot(time_list,XX,color='k')
plt.grid()
plt.legend(['Sensor data','AKF Estimated'])
plt.ylabel('Roll (degree)')
plt.xlabel('Time (s)')

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
plt.show()
# print('av tim')
# print(sum(tt)/len(tt))
# print('RMS error')
# print(sum(EE))
