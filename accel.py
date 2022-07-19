# Implementing Invensense's ICM-20948, as configured in SparkFun's 9DoF IMU Breakout (SEN-15335)

import argparse
import pandas as pd
import time
import smbus
import numpy as np

# Add some defines for the various register settings
ADDRESS = 0x69
BANK_SELECT = 0x7f
_bank_select_0 = 0x00
_bank_select_1 = 0x10
_bank_select_2 = 0x20
_bank_select_3 = 0x30
#bank 0
POWER_MANAGEMENT_1 = 0x06
POWER_MANAGEMENT_2 = 0x07

ACCEL_X_HIGH = 0x2d
ACCEL_X_LOW = 0x2e
ACCEL_Y_HIGH = 0x2f
ACCEL_Y_LOW = 0x30
ACCEL_Z_HIGH = 0x31
ACCEL_Z_LOW = 0x32

GYRO_X_HIGH = 0x33
GYRO_X_LOW = 0x34
GYRO_Y_HIGH = 0x35
GYRO_Y_LOW = 0x36
GYRO_Z_HIGH = 0x37
GYRO_Z_LOW = 0x38

#bank 2
GYRO_RANGE = 0x01 # 00000110: 00+-250, 01+-500, 10+-1000, 11+-2000
_gyro_range_250 = 0x00
_gyro_range_500 = 0x02
_gyro_range_1000 = 0x04
_gyro_range_2000 = 0x06

ACCELERATION_RANGE =  0x14 # 00000110: 00+-2, 01+-4, 10+-8, 11+-16
_acceleration_range_2 = 0x00
_acceleration_range_4 = 0x02
_acceleration_range_8 = 0x04
_acceleration_range_16 = 0x06
_acceleration_enable_lpf = 0x01
_acceleration_lpf_config_low_freq = 0x30
_acceleration_lpf_config_mid_freq = 0x18
_acceleration_lpf_config_high_freq = 0x38

GYRO_RANGE = 0x01
_gyro_range_250 = 0x00
_gyro_range_500 = 0x02
_gyro_range_1000 = 0x04
_gyro_range_2000 = 0x06
_gyro_enable_lpf = 0x01
_gyro_lpf_config_low_noise = 0x30
_gyro_lpf_config_high_speed = 0x38

class IMU():
    def __init__(self):
        self.devAddress = 0x69
        # create the i2c smbus so we can communicate
        self.busI2c = smbus.SMBus(1)
        # make sure we're on the right bank
        self.busI2c.write_byte_data(ADDRESS, BANK_SELECT, _bank_select_0)
        # wake up the accelerometer
        self.busI2c.write_byte_data(ADDRESS, POWER_MANAGEMENT_1, 0x01)
        # disable lpf by default and set to mid freq
        self.lpfA_config = _acceleration_lpf_config_mid_freq
        self.lpfA_enable = 0
        # set the acceleration range to +-16 for default
        self.setAccelRange(_acceleration_range_16)
        # set the gyro to +- 2000deg/sec for default
        self.setGyroRange(_gyro_range_2000)
        
    def setAccelLPF(self, bEnable, freq=_acceleration_lpf_config_mid_freq):
        if bEnable:
            self.lpfA_enable = _acceleration_enable_lpf
        else:
            self.lpfA_enable = 0
        self.lpfA_config = freq
        self.setAccelRange(self.accelRange)
        
    def setAccelRange(self, range):
        self.accelRange = range
        # first set the divider
        if range == _acceleration_range_2:
            self.accelDivider = 16384
        elif range == _acceleration_range_4:
            self.accelDivider = 8192
        elif range == _acceleration_range_8:
            self.accelDivider = 4096
        elif range == _acceleration_range_16:
            self.accelDivider = 2048
        # then set the appropriate register
        self.busI2c.write_byte_data(ADDRESS, BANK_SELECT, _bank_select_2)
        #make sure to turn on the lpf and set it to lowest possible noise
        range += self.lpfA_config + self.lpfA_enable
        self.busI2c.write_byte_data(ADDRESS, ACCELERATION_RANGE, range)
        self.busI2c.write_byte_data(ADDRESS, BANK_SELECT, _bank_select_0)
            
    def setGyroRange(self, range):
        # first set the divider
        if range == _gyro_range_250:
            self.gyroDivider = 131.072 #+-250degrees, so 32768/250
        elif range == _gyro_range_500:
            self.gyroDivider = 65.536
        elif range == _gyro_range_1000:
            self.gyroDivider = 32.768
        elif range == _gyro_range_2000:
            self.gyroDivider = 16.384
        # then set the register
        self.busI2c.write_byte_data(ADDRESS, BANK_SELECT, _bank_select_2)
        #make sure to turn on the lpf and set it to lowest possible noise
        range += _gyro_lpf_config_low_noise + _gyro_enable_lpf
        self.busI2c.write_byte_data(ADDRESS, GYRO_RANGE, range)
        self.busI2c.write_byte_data(ADDRESS, BANK_SELECT, _bank_select_0)
        
    def getAccel_x(self):
        high = self.busI2c.read_byte_data(ADDRESS, ACCEL_X_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, ACCEL_X_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def getAccel_y(self):
        high = self.busI2c.read_byte_data(ADDRESS, ACCEL_Y_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, ACCEL_Y_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def getAccel_z(self):
        #read 0x31 High byte, 0x32 low byte
        high = self.busI2c.read_byte_data(ADDRESS, ACCEL_Z_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, ACCEL_Z_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def getGyro_x(self):
        high = self.busI2c.read_byte_data(ADDRESS, GYRO_X_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, GYRO_X_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def getGyro_y(self):
        high = self.busI2c.read_byte_data(ADDRESS, GYRO_Y_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, GYRO_Y_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def getGyro_z(self):
        high = self.busI2c.read_byte_data(ADDRESS, GYRO_Z_HIGH)
        low = self.busI2c.read_byte_data(ADDRESS, GYRO_Z_LOW)
        out = (high << 8) + low
        return np.int16(out)
    
    def accel_g(self, value):
        return value / self.accelDivider
    
    def accel_ftsec(self, value):
        return (value / self.accelDivider) * 32.17405
    
    def gyro_deg(self, value):
        return value / self.gyroDivider

# Get command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("time", help="Time to run in seconds[int]", type=int)
parser.add_argument("file", help="File name for output csv file")
parser.add_argument("-a", "--accel_range", help="Max range in +-g", type=int, choices=[2,4,8,16])
parser.add_argument("-g", "--gyro_range", help="Max range in +-degrees per second", type=int, choices=[250,500,1000,2000])
parser.add_argument("-d", "--delayMS", help="Specify additional delay (ms) in data collection loop [float]", type=float)
parser.add_argument("--ms", help="Output time index in milliseconds instead of microseconds", action="store_true")
parser.add_argument("-k", "--keypress", help="After zeroing will wait to start data collection until Enter is pressed", action="store_true")
parser.add_argument("-l", "--lpf", help="Low Pass Filter frequency settings: low, mid, high", type=int, choices=[1,2,3])
args = parser.parse_args()
accelerometer = IMU()

if args.accel_range == 2:
    accelerometer.setAccelRange(_acceleration_range_2)
elif args.accel_range == 4:
    accelerometer.setAccelRange(_acceleration_range_4)
elif args.accel_range == 8:
    accelerometer.setAccelRange(_acceleration_range_8)
elif args.accel_range == 16:
    accelerometer.setAccelRange(_acceleration_range_16)
    
if args.lpf == 1:
    accelerometer.setAccelLPF(True, _acceleration_lpf_config_low_freq)
elif args.lpf == 2:
    accelerometer.setAccelLPF(True, _acceleration_lpf_config_mid_freq)
elif args.lpf == 3:
    accelerometer.setAccelLPF(True, _acceleration_lpf_config_high_freq)
    
if args.gyro_range == 250:
    accelerometer.setGyroRange(_gyro_range_250)
elif args.gyro_range == 500:
    accelerometer.setGyroRange(_gyro_range_500)
elif args.gyro_range == 1000:
    accelerometer.setGyroRange(_gyro_range_1000)
elif args.gyro_range == 2000:
    accelerometer.setGyroRange(_gyro_range_2000)

# Calculate offsets (pretty sure the accelerometer has an integrated method for this)
# Loop of 500 is 4 sec at default rate of 125hz (probably should make the how long an option)
print("Zeroing Accelerometer (do not move it!)...\n")
time.sleep(1) # Wait just a bit
axOff = 0
ayOff = 0
azOff = 0
rxOff = 0
ryOff = 0
rzOff = 0
for i in range(500):
    axOff += accelerometer.getAccel_x()
    ayOff += accelerometer.getAccel_y()
    azOff += accelerometer.getAccel_z()
    rxOff += accelerometer.getGyro_x()
    ryOff += accelerometer.getGyro_y()
    rzOff += accelerometer.getGyro_z()
axOff /= 500
ayOff /= 500
azOff /= 500
rxOff /= 500
ryOff /= 500
rzOff /= 500

frameList = []
# Use time as the first column, index/timestamp
zero = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
# Figure out how many frames and any additional delay
totalFrames = 161 * args.time
delaySec = 0
if args.delayMS is not None:
    totalFrames = int(float(1000 * args.time) / (args.delayMS + 6.2))
    delaySec = args.delayMS * 0.001
    
print("Starting data collection: %d Sec at a sample rate of %dHz\n" % (args.time, 1000 / (6.2 + (delaySec / 1000))))
if args.keypress:
    input("Press Enter to begin...")
for i in range(totalFrames): 
    ax = accelerometer.getAccel_x()
    ay = accelerometer.getAccel_y()
    az = accelerometer.getAccel_z()
    rx = accelerometer.getGyro_x()
    ry = accelerometer.getGyro_y()
    rz = accelerometer.getGyro_z()
    frameList += [[time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                  ax, 
                  ay,
                  az,
                  rx,
                  ry,
                  rz]]
    if args.delayMS is not None:
        time.sleep(delaySec)

print("Saving data to " + args.file)

# Apply offsets and convert to g and deg/s
for frame in frameList:
    frame[0] -= zero
    if args.ms:
        frame[0] *= 0.000001
    else:
        frame[0] *= 0.001
    for i in range(3):
        frame[i + 1] = accelerometer.accel_g(frame[i + 1])
    frame[1] -= accelerometer.accel_g(axOff)
    frame[2] -= accelerometer.accel_g(ayOff)
    frame[3] -= accelerometer.accel_g(azOff)
    for i in range(3):
        frame[i + 4] = accelerometer.gyro_deg(frame[i + 4])
    frame[4] -= accelerometer.gyro_deg(rxOff)
    frame[5] -= accelerometer.gyro_deg(ryOff)
    frame[6] -= accelerometer.gyro_deg(rzOff)
# Create the dataframe, set time index to int (truncate to ms/us)
dataFrame = pd.DataFrame(frameList, columns=['Time','AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ'])
dataFrame.Time = dataFrame.Time.astype(int)
dataFrame.set_index('Time', inplace=True)
dataFrame.to_csv(args.file)
