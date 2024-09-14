/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_ins.c
 *  Description  : Attitude solution
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:43:30
 *  LastEditTime : 2023-08-15 01:28:01
 */


#include "app_ins.h"
#include "sys_dwt.h"
#include "alg_quaternionEKF.h"
#include "alg_math.h"
#include "periph_bmi088.h"
#include "periph_bmp280.h"
#include "periph_magnetic.h"


INS_INSTypeDef INS;

void Ins_Task(void const * argument) {
    BMI088_DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();
    INS_INSTypeDef* ins = INS_GetINSPtr();
	static uint32_t tick;
    const float gravity[3] = {0, 0, 9.81f};
    const float install_comp[3][3] = {
                                    {0, 1, 0},
                                    {1, 0, 0},
                                    {0, 0, 1}};
    forever {        
        static uint32_t count = 0;
        ins->update_dt = DWT_GetDeltaT(&ins->last_update_tick);

        if (((count % 1) == 0)) {
            BMI088_DecodeData();            

            ins->Accel[X_INS] = bmi088->accel.x;
            ins->Accel[Y_INS] = bmi088->accel.y;
            ins->Accel[Z_INS] = bmi088->accel.z;
            ins->Gyro[X_INS] = bmi088->gyro.pitch;
            ins->Gyro[Y_INS] = bmi088->gyro.row;
            ins->Gyro[Z_INS] = bmi088->gyro.yaw;

            // demo function
            Param_Correction(&ins->Param, ins->Gyro, ins->Accel);

            // Calculate the angle between the gravity acceleration vector and the XY axis of the b system, 
            // which can be used as a function extension.
            ins->atanxz = -atan2f(ins->Accel[X_INS], ins->Accel[Z_INS]) * 180 / PI;
            ins->atanyz = atan2f(ins->Accel[Y_INS], ins->Accel[Z_INS]) * 180 / PI;

            // Core function, EKF update quaternion
            QuaternionEKF_Update(ins->Gyro[X_INS], ins->Gyro[Y_INS], ins->Gyro[Z_INS], ins->Accel[X_INS], ins->Accel[Y_INS], ins->Accel[Z_INS], ins->update_dt);

            memcpy(ins->q, QEKF_INS.q, sizeof(QEKF_INS.q));

            // The base vector of the airframe system is converted to the navigation coordinate system. In this example, 
            // the inertial system is selected as the navigation system
            BodyFrameToEarthFrame(install_comp[X_INS], ins->xn, ins->q);
            BodyFrameToEarthFrame(install_comp[Y_INS], ins->yn, ins->q);
            BodyFrameToEarthFrame(install_comp[Z_INS], ins->zn, ins->q);

            // Convert the gravity from the navigation coordinate system n to the aircraft system b, 
            // and then calculate the motion acceleration according to the accelerometer data
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, ins->q);
            for (uint8_t i = 0; i < 3; i++) {
                ins->MotionAccel_b[i] = (ins->Accel[i] - gravity_b[i]) * ins->update_dt / (ins->AccelLPF + ins->update_dt) + ins->MotionAccel_b[i] * ins->AccelLPF / (ins->AccelLPF + ins->update_dt);
            }
            BodyFrameToEarthFrame(ins->MotionAccel_b, ins->MotionAccel_n, ins->q);
						
            Ins_InstallCompAngle(ins, install_comp);
			
        }
        count++;
      osDelay(1);
    }
}


INS_INSTypeDef *INS_GetINSPtr() {
    return &INS;
}


void INS_Init() {
    INS_INSTypeDef* ins = INS_GetINSPtr();
    ins->Param.scale[X_INS] = 1;
    ins->Param.scale[Y_INS] = 1;
    ins->Param.scale[Z_INS] = 1;
    ins->Param.Yaw = 0;
    ins->Param.Pitch = 0;
    ins->Param.Roll = 0;
    ins->Param.flag = 1;

    QuaternionEKF_Init(10, 0.01f, 10000000, 1, 0.0085f);
    ins->AccelLPF = 0.0085f;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}


/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


/**
 * @brief reserved.It is used to correct IMU installation error and scale factor error, 
 *        i.e. the installation deviation of gyroscope axis and PTZ axis
 * @param param 
 * @param gyro  
 * @param accel 
 */
static void Param_Correction(INS_DataTypeDef *param, float gyro[3], float accel[3]) {
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X_INS] = c_11 * gyro_temp[X_INS] +
              c_12 * gyro_temp[Y_INS] +
              c_13 * gyro_temp[Z_INS];
    gyro[Y_INS] = c_21 * gyro_temp[X_INS] +
              c_22 * gyro_temp[Y_INS] +
              c_23 * gyro_temp[Z_INS];
    gyro[Z_INS] = c_31 * gyro_temp[X_INS] +
              c_32 * gyro_temp[Y_INS] +
              c_33 * gyro_temp[Z_INS];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X_INS] = c_11 * accel_temp[X_INS] +
               c_12 * accel_temp[Y_INS] +
               c_13 * accel_temp[Z_INS];
    accel[Y_INS] = c_21 * accel_temp[X_INS] +
               c_22 * accel_temp[Y_INS] +
               c_23 * accel_temp[Z_INS];
    accel[Z_INS] = c_31 * accel_temp[X_INS] +
               c_32 * accel_temp[Y_INS] +
               c_33 * accel_temp[Z_INS];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}


void Ins_InstallCompAngle(INS_INSTypeDef *ins, const float correc_mat[3][3]) {
    ins->Pitch = QEKF_INS.Pitch * correc_mat[0][0] + QEKF_INS.Roll * correc_mat[0][1];
    ins->Roll = QEKF_INS.Pitch * correc_mat[1][0] + QEKF_INS.Roll * correc_mat[1][1];
    ins->Yaw = QEKF_INS.Yaw;
    ins->YawTotalAngle = QEKF_INS.YawTotalAngle;
}


void Ins_ClearYawTotalCounter() {
    QEKF_INS.YawRoundCount = 0;
}


/**
  * @brief          Ins Gpio Exit Callbcak
  * @param          GPIO_Pin :Specifies the pins connected EXTI line
  * @retval         NULL
  */
void Ins_GPIOExitCallback(GPIO_GPIOTypeDef *gpio) {
    if (gpio == BMI_INT1) {

    }
    else if (gpio == BMI_INT2) {

    }
    
    else if (gpio == IST8310_DRDY) {
        MAG_MAGUpdate();
    }
}
