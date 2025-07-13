#include <stdio.h>
#include <kalman.h>
#include <string.h>

void kalman_alt_init(kalman_altitude_t *kf, float Qz, float Qvz, float Pz, float Pvz) {
    memset(kf, 0, sizeof(kalman_altitude_t));

    kf->Pz = Pz;
    kf->Pvz = Pvz;
    kf->Qz = Qz;
    kf->Qvz = Qvz;
}

void kalman_predict(kalman_altitude_t *kf, float acc_z, float dt, float *z, float *vz) {
    *z += ((*vz) * dt) + (0.5f * acc_z * dt * dt);
    *vz += acc_z * dt;

    kf->Pz += (dt * ((2.0f * kf->Pzvz) + (dt * kf->Pvz))) + kf->Qz;
    kf->Pzvz += dt * kf->Pvz;
    kf->Pvz += kf->Qvz;
}

void kalman_update(kalman_altitude_t *kf, float z_sensor, float R, float *z, float *vz) {
    float err = z_sensor - (*z);

    float Kz = kf->Pz / (kf->Pz + R);
    float Kvz = kf->Pzvz / (kf->Pz + R);

    (*z) += Kz * err;
    (*vz) += Kvz * err;

    kf->Pz = (1.0f - Kz) * kf->Pz;
    kf->Pzvz = (1.0f - Kz) * kf->Pzvz;
    kf->Pvz += - (Kvz * kf->Pzvz);
}
