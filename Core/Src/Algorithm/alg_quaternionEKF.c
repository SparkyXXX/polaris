/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : alg_quaternionEKF.c
 *  Description  : attitude update with gyro bias estimate and chi-square test
 *  LastEditors  : Polaris
 *  Date         : 2023-02-06 00:50:38
 *  LastEditTime : 2023-08-12 01:00:13
 */


#include "alg_quaternionEKF.h"
#include "sys_const.h"

QEKF_INS_t QEKF_INS;

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf) {
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1) {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    Kalman_FilterInit(&QEKF_INS.QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    QEKF_INS.QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.QuaternionEKF.xhat_data[3] = 0;

    QEKF_INS.QuaternionEKF.User_Func0_f = QuaternionEKF_Observe;
    QEKF_INS.QuaternionEKF.User_Func1_f = QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.QuaternionEKF.User_Func2_f = QuaternionEKF_SetH;
    QEKF_INS.QuaternionEKF.User_Func3_f = QuaternionEKF_xhatUpdate;

    QEKF_INS.QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.QuaternionEKF.F_data, QuaternionEKF_F, sizeof(QuaternionEKF_F));
    memcpy(QEKF_INS.QuaternionEKF.P_data, QuaternionEKF_P, sizeof(QuaternionEKF_P));
}


/**
 * @brief           Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    // 0.5(Ohm-Ohm^bias)*deltaT
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!QEKF_INS.Initialized) {
        QuaternionEKF_Init(10, 0.001f, 1000000 * 10, 0.9996f * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    memcpy(QEKF_INS.QuaternionEKF.F_data, QuaternionEKF_F, sizeof(QuaternionEKF_F));

    QEKF_INS.QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.QuaternionEKF.F_data[20] = -halfgxdt;
	QuaternionEKF_Observe(&QEKF_INS.QuaternionEKF);
    if (QEKF_INS.UpdateCount == 0) {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    accelInvNorm = Math_InvSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;
    }

    // get body state
    QEKF_INS.gyro_norm = 1.0f / Math_InvSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                             QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                             QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f) {
        QEKF_INS.StableFlag = 1;
    }
    else {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R
    QEKF_INS.QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.QuaternionEKF.R_data[8] = QEKF_INS.R;

    Kalman_FilterUpdate(&QEKF_INS.QuaternionEKF);

    QEKF_INS.q[0] = QEKF_INS.QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; 

    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) * 57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;

    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++;
}


/**
 * @brief It is used to update a 4x2 block matrix in the upper right corner of 
 *        the linearized state transition matrix F. Later, it is used to update 
 *        the covariance matrix P and limit the variance of the zero drift to prevent 
 *        over-convergence and limit the amplitude to prevent divergence
 *
 * @param kf
 */
static void QuaternionEKF_F_Linearization_P_Fading(Kalman_KalmanTypeDef *kf) {
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = Math_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++) {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    if (kf->P_data[28] > 10000) {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000) {
        kf->P_data[35] = 10000;
    }
}


/**
 * @brief Calculate the Jacobi matrix H of the observation function h (x) at the working point
 *
 * @param kf
 */
static void QuaternionEKF_SetH(Kalman_KalmanTypeDef *kf) {
    static float doubleq0, doubleq1, doubleq2, doubleq3;
		uint8_t sizeof_float = sizeof(float);
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}


/**
 * @brief Using observations and prior estimates to obtain the optimal posterior estimate
 *        Chi-square test is added to judge whether the conditions of fusion acceleration are met
 *        At the same time, the divergence protection is introduced to ensure the necessary 
 *        measurement update under bad conditions
 *
 * @param kf
 */
static void QuaternionEKF_xhatUpdate(Kalman_KalmanTypeDef *kf) {
    static float q0, q1, q2, q3;
		uint8_t sizeof_float = sizeof(float);

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;

    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
        if (QEKF_INS.StableFlag) {
            QEKF_INS.ErrorCount++;
        }
        else {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50) {
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else {
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else { // if divergent or rk is not that big/acceptable,use adaptive gain
        // scale adaptive
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++) {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    if (QEKF_INS.ConvergeFlag) {
        for (uint8_t i = 4; i < 6; i++) {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}


/**
 * @brief EKF observe
 *
 * @param kf 
 */
static void QuaternionEKF_Observe(Kalman_KalmanTypeDef *kf) {
    memcpy(QuaternionEKF_P, kf->P_data, sizeof(QuaternionEKF_P));
    memcpy(QEKF_INS.QuaternionEKF_K, kf->K_data, sizeof(QEKF_INS.QuaternionEKF_K));
    memcpy(QEKF_INS.QuaternionEKF_H, kf->H_data, sizeof(QEKF_INS.QuaternionEKF_H));
}
