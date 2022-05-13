#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <BleKeyboard.h>
#include <arduinoFFT.h>

struct IMUdata {
  float ax, ay, az,
        gx, gy, gz,
        wx, wy, wz;
  float NetAccel, NetOmega, NetTorque;
  unsigned int timeflag;
  bool onImpact;
};

struct GSRdata {
  int v;
  float dvdt;
  unsigned int timeflag;
  bool onImpact;
};

const PROGMEM unsigned short BVPpin = 14, GSRpin = 15;
const PROGMEM unsigned int BVPsampleSize = 512, LoopDelay = 20, CameraCooldown = 3000;

BleKeyboard bleKeyboard;
LSM9DS1 imu;

void setup() {
  pinMode(BVPpin, INPUT);
  pinMode(GSRpin, INPUT);
  Wire.begin(12, 22);
  while (!imu.begin());
  bleKeyboard.begin();
}

void loop() {
  static unsigned int LastLoop = 0;
  unsigned int timeflag = millis();
  if (timeflag - LastLoop > LoopDelay) {
    LastLoop = timeflag;
    bool onImpact = UpdateIMUdata()->onImpact;
    bool onPulse = GetHeartRate() > 105;
    bool onSurprise = UpdateGSRdata()->onImpact;
    if (onImpact || onPulse || onSurprise) TakePicture();
  }
}

IMUdata* UpdateIMUdata() {
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.accelAvailable()) imu.readAccel();
  static IMUdata *data = (struct IMUdata*) calloc(1, sizeof(IMUdata));
  unsigned int timegap = millis() - data->timeflag;
  data->ax = imu.calcAccel(imu.ax);
  data->ay = imu.calcAccel(imu.ay);
  data->az = imu.calcAccel(imu.az);
  data->wx = (imu.calcAccel(imu.gx) - data->gx) / timegap;
  data->wy = (imu.calcAccel(imu.gy) - data->gy) / timegap;
  data->wz = (imu.calcAccel(imu.gz) - data->gz) / timegap;
  data->gx = imu.calcAccel(imu.gx);
  data->gy = imu.calcAccel(imu.gy);
  data->gz = imu.calcAccel(imu.gz);
  data->timeflag += timegap;
  data->NetAccel = sqrt(pow(data->ax, 2) + pow(data->ay, 2) + pow(data->az, 2));
  data->NetOmega = sqrt(pow(data->gx, 2) + pow(data->gy, 2) + pow(data->gz, 2));
  data->NetTorque = sqrt(pow(data->wx, 2) + pow(data->wy, 2) + pow(data->wz, 2));
  data->onImpact = data->NetAccel > 3 && data->NetTorque > 0.7;
  return data;
}

GSRdata* UpdateGSRdata() {
  static GSRdata *data = (struct GSRdata*) calloc(1, sizeof(GSRdata));
  unsigned int timegap = millis() - data->timeflag;
  int nv = analogRead(GSRpin);
  data->dvdt = double(nv - data->v) / timegap;
  data->v = nv;
  data->timeflag += timegap;
  data->onImpact = data->dvdt > 3;
  return data;
}

unsigned int GetHeartRate() {
  static arduinoFFT FFT = arduinoFFT();
  static int *vRaw = (int *) calloc(BVPsampleSize, sizeof(int));
  memmove(vRaw, vRaw + 1, (BVPsampleSize - 1) * sizeof(int));
  vRaw[BVPsampleSize - 1] = analogRead(BVPpin);
  double *vReal = (double *) malloc(sizeof(double) * BVPsampleSize);
  double *vImag = (double *) calloc(BVPsampleSize, sizeof(double));
  for (int i = 0; i < BVPsampleSize; i++) vReal[i] = vRaw[i];
  FFT.Windowing(vReal, BVPsampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BVPsampleSize, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BVPsampleSize);
  double x = FFT.MajorPeak(vReal, BVPsampleSize, 1000.0 / LoopDelay);
  free(vReal), free(vImag);
  return round(60 * x);
}

void TakePicture() {
  static unsigned int LastTaken = -CameraCooldown;
  if (millis() - LastTaken < CameraCooldown) return;
  LastTaken = millis();
  if (bleKeyboard.isConnected()) bleKeyboard.print(char(0x20));
}
