# SenseCam.ino
## Structs
### IMUdata
Stores the acceleration and gyro sensor data from IMU in `ax`,`ay`,`az` and `gx`,`gy`,`gz`.
Stores the angular speed in `wx`,`wy`,`wz`.
Stores the net acceleration, net angular speed, and net torque in `NetAccel`,`NetOmega`,`NetTorque`.
Stores the last fetched time epoch in `timeflag`.
Stores a boolean value that is true if physical impact detected in `onImpact`.
### GSRdata
Stores the sensor value in `v`, and the time derivative of `v` in `dvdt`.
Stores the last fetched time epoch in `timeflag`.
Stores a boolean value that is true if GSR value impact detected in `onImpact`.
## Global Variables
### Constants
`BVPpin`, `GSRpin` is the pin number of each sensors signal pin.
`BVPsampleSize` is the buffer size of the array that the values of BVP sensor stored.
`LoopDelay` is the time of interval between each single iteration of loop in milliseconds.
`CameraCooldown` is the cooldown of the camera taking pictures in milliseconds.
### Handler Objects
`bleKeyboard` is a emulates bluetooth keyboard.
`imu` is a handler of IMU sensor.
## Functions
### void setup()
Set up pins, IMU(with custom SDA pin), and bluetooth keyboard.
### void loop()
Check if any of three impacts happened every `LoopDelay` milliseconds.
If any, call `TakePicture()` to take picture.
### UpdateIMUdata
Fetch new sensor values from IMU.
Calculate angular velocity from latest two sensor values.
Calculate net acceleration, net angular speed, and net torque.
Check if impact happened.
Stores all data to `static IMUdata *data`.
Returns `data`.
### UpdateGSRdata
Fetch new sensor values from GSR.
Calculate dv/dt from latest two sensor values.
Check if impact happened.
Stores all data to `static GSRdata *data`.
Returns `data`.
### GetHeartRate
Fetch new sensor values from BVP and append it to array.
Calculate FFT to get frequency of beat.
Pick major frequency and returns it in bpm.
### TakePicture
Check cooldown and send `0x20` via bluetooth keyboard.
## Libraries
- [SparkFunLSM9DS1](https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library) - Library for IMU sensor
- [BleKeyboard](https://github.com/T-vK/ESP32-BLE-Keyboard) - ESP32 bluetooth keyboard library
- [arduinoFFT](https://github.com/kosme/arduinoFFT) - FFT library for arduino
