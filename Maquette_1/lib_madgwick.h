#import <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

float beta;  // compute beta : GyroMeasError = PI * (40.0f / 180.0f)
float GyroMeasDrift;      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float zeta; // GyroMeasDrift = PI * (2.0f / 180.0f)
                                                 

extern float q[4];
extern float deltat;

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);

#ifdef __cplusplus
}
#endif
