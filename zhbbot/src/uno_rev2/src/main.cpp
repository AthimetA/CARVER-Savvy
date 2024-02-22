#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  Serial.println("Hi this is edited by me");
}

long pre_millis = 0;
void loop() {
  float wx, wy, wz, ax, ay, az;

  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable() && (millis() - pre_millis >= 20) ) 
  {
    pre_millis = millis();

    IMU.readGyroscope(wx, wy, wz); // Read gyroscope values degrees/second
    IMU.readAcceleration(ax, ay, az); // Read acceleration values g

    //Convert degrees/second to radian/second
    wx = wx * (PI / 180.0);
    wy = wy * (PI / 180.0);
    wz = wz * (PI / 180.0);

    //Convert g to m/s^2
    ax = ax * 9.81;
    ay = ay * 9.81;
    az = az * 9.81;

    Serial.print(wx);
    Serial.print('\t');
    Serial.print(wy);
    Serial.print('\t');
    Serial.print(wz);
    Serial.print('\t');
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.println(az);
  }
}
