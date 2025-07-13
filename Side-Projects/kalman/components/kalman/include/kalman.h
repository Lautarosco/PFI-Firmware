#pragma once

typedef struct {
    // float z;                /* State: Altitude [m] */
    // float vz;               /* State: Altitude velocity [m/s] */
    float Pz;               /* Covariance in altitude */
    float Pvz;              /* Covariance in altitude velocity */
    float Pzvz;             /* Cross covariance between altitude and altitude velocity */
    float Qz;               /* Noise in prediction for altitude */
    float Qvz;              /* Noise in prediction for altitude velocity */
} kalman_altitude_t;

void kalman_alt_init(kalman_altitude_t *kf, float Qz, float Qvz, float Pz, float Pvz);
void kalman_predict(kalman_altitude_t *kf, float acc_z, float dt, float *z, float *vz);
void kalman_update(kalman_altitude_t *kf, float z_sensor, float R, float *z, float *vz);