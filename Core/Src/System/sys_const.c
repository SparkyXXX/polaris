/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.c
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:20:29
 *  LastEditTime : 2023-08-24 17:24:01
 */


#include "sys_const.h"

const float Const_SERVO_INIT_OFFSET = 10.0f;

const float Const_LEFT_WHEEL_MOTOR_INIT_OFFSET          = 0.0f;
const float Const_RIGHT_WHEEL_MOTOR_INIT_OFFSET         = 0.0f;   
const float Const_BODY_LEFT_FRONT_MOTOR_INIT_OFFSET     = 104.679993f;       
const float Const_BODY_RIGHT_FRONT_MOTOR_INIT_OFFSET    = 73.890015f;       
const float Const_BODY_LEFT_BACK_MOTOR_INIT_OFFSET      = -135.589996f;   
const float Const_BODY_RIGHT_BACK_MOTOR_INIT_OFFSET     = -45.3899994f;       
const float Const_CRO_LEFT_FRONT_MOTOR_INIT_OFFSET      = 32.679993f;   
const float Const_CRO_RIGHT_FRONT_MOTOR_INIT_OFFSET     = 32.669983f;       
const float Const_CRO_LEFT_BACK_MOTOR_INIT_OFFSET       = -168.690002f;   
const float Const_CRO_RIGHT_BACK_MOTOR_INIT_OFFSET      = 143.479996f;   
const float Const_KNEE_LEFT_FRONT_MOTOR_INIT_OFFSET     = 0.0f;       
const float Const_KNEE_RIGHT_FRONT_MOTOR_INIT_OFFSET    = 0.0f;       
const float Const_KNEE_LEFT_BACK_MOTOR_INIT_OFFSET      = 0.0f;   
const float Const_KNEE_RIGHT_BACK_MOTOR_INIT_OFFSET     = 0.0f;       
const float Const_GIMBAL_YAW_MOTOR_INIT_OFFSET          = 53.0f;   
const float Const_GIMBAL_PITCH_MOTOR_INIT_OFFSET        = -120.0f; 

const float Const_HOP_LEFT_FOUNT_OFFSET                 = 389.28f;
const float Const_HOP_LEFT_BACK_OFFSET                  = -94.13f;
const float Const_HOP_RIGHT_FOUNT_OFFSET                = -202.929993f;
const float Const_HOP_RIGHT_BACK_OFFSET                 = 206.580002f;

const float Const_QUAD_LEFT_FOUNT_1_OFFSET              = 0.0f;
const float Const_QUAD_LEFT_FOUNT_2_OFFSET              = -276.0f;
const float Const_QUAD_LEFT_FOUNT_3_OFFSET              = 180.0f;
const float Const_QUAD_LEFT_BACK_1_OFFSET               = 0.0f;
const float Const_QUAD_LEFT_BACK_2_OFFSET               = -386.0f;
const float Const_QUAD_LEFT_BACK_3_OFFSET               = -18.0f;
const float Const_QUAD_RIGHT_FOUNT_1_OFFSET             = 0.0f;
const float Const_QUAD_RIGHT_FOUNT_2_OFFSET             = 0.0f;
const float Const_QUAD_RIGHT_FOUNT_3_OFFSET             = 0.0f;
const float Const_QUAD_RIGHT_BACK_1_OFFSET              = 180.0f;
const float Const_QUAD_RIGHT_BACK_2_OFFSET              = -30.0f;
const float Const_QUAD_RIGHT_BACK_3_OFFSET              = 240.0f;

const float Const_WHEELLEG_REMOTE_YAW_GAIN              = 0.0003f;
float Const_WHEELLEG_REMOTE_X_GAIN                      = 0.0000022f;
const float Const_WHEELLEG_REMOTE_LEN_GAIN              = 0.00000007f;

const float Const_WHEELLEG_DEFAULT_LEG_LEN              = 0.21f;
const float Const_WHEELLEG_NORMAL_LEG_LEN				= 0.24f;
const float Const_WHEELLEG_STABLE_TIME                  = 2000.0f;
const float Const_WHEELLEG_FALL_CONTINUE_TIME           = 4000.0f;

const float Const_WHEELLEG_FALL_SPEED_THRESHOLD         = 5.0f;
const float Const_WHEELLEG_STABLE_PITCH_ANGLE           = 10.0f;
const float Const_WHEELLEG_FALL_PHI_ANGLE               = 20.0f;
const uint32_t Const_WHEELLEG_PHI_ANG_FALL_TICK			= 600;
const uint32_t Const_WHEELLEG_SPEED_FALL_TICK           = 800;

float Const_WHEELLEG_LQR_LEG_T_GAIN                     = 0.8f;
float Const_WHEEELEG_LQR_LEG_TP_GAIN                    = 0.6f;
        
const float REMOTE_YAW_ANGLE_TO_REF                     = 0.0006f;
const float REMOTE_PITCH_ANGLE_TO_REF                   = 0.0003f;
float Const_PITCH_UMAXANGLE                             = 22.0f; 
float Const_PITCH_DMAXANGLE                             = -24.0f;
float Const_YAW_MAXANGLE                                = 40.0f;
float Const_Chassis_Gyrp_Spd                            = 4.0f;

const float Const_HopLeftLenParam[4][5] = {
    {480.0f, 1.8f, 0.0f, 10, 40}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_HopRightLenParam[4][5] = {
    {480.0f, 1.8f, 0.0f, 10, 40}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_HopLeftAngParam[4][5] = {
    {0.3f, 0, 0.0f, 20, 10}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_HopRightAngParam[4][5] = {
    {0.3f, 0, 0.0f, 20, 10}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_BalRollParam[2][4][5] = {
    {{6.0f, 0.0f, 0.0f, 20.0f, 60.0f}, {0.1f, -1}, {0.0f, 0.0f}, {-1, -1}},     // ang pid
    {{3.0f, 0.0f, 0.0f, 20.0f, 50.0f}, {0.1f, -1}, {0.0f, 0.0f}, {-1, -1}}};    // gyro pid

const float Const_BalYawParam[2][4][5] = {
    {{0.18f, 0, 0, 0, 8}, {0.1f, -1}, {0, 0}, {-1, -1}},                        // ang pid
    {{3.0f, 0.0f, 0, 3, 8}, {0.1f, -1}, {0, 0}, {-1, -1}}};                       // gyro pid

const float Const_WheelMotorCurParam[4][5] = {
    {0.0f, 0.0f, 0.0f, 10, 40}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_GimbalYawParam[2][4][5] = {
    {{0.1f, 0.0f, 0, 10, 15}, {0.1f, -1}, {0, 0}, {-1, -1}},                        // ang pid
    {{3.5f, 0.1f, 0, 5, 15}, {0.1f, -1}, {0, 0}, {-1, -1}}};                       // gyro pid

const float Const_GimbalPitchParam[2][4][5] = {
    {{0.8f, 0.0f, 0, 20.0f, 6.0f}, {0.1f, -1}, {0, 0}, {-1, -1}},                        // ang pid
    {{4.0f, 0.1f, 0, 10.0f, 15.0f}, {0.1f, -1}, {0, 0}, {-1, -1}}};                       // gyro pid

const float QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1};

float QuaternionEKF_P[36] = {100000,    0.1,    0.1,    0.1,    0.1,    0.1,
                                0.1, 100000,    0.1,    0.1,    0.1,    0.1,
                                0.1,    0.1, 100000,    0.1,    0.1,    0.1,
                                0.1,    0.1,    0.1, 100000,    0.1,    0.1,
                                0.1,    0.1,    0.1,    0.1,    100,    0.1,
                                0.1,    0.1,    0.1,    0.1,    0.1,    100};

float LQR_PolyfitK[4][12] = {
{   65.5135f,   39.2983f,  -71.4362f,  -34.3490f,  109.8385f,   27.3984f,
   278.8675f,   15.2160f,   68.9978f,   43.9194f,  460.0334f,   90.2805f,},
{  -20.5502f,  -59.6722f,  124.3063f,   73.2886f,  -55.7614f,  -25.4322f,
  -414.1953f,  -37.5535f,  -34.1635f,  -22.8636f, -799.8604f, -166.1015f,},
{ -153.0773f,  -14.5429f,  -80.6106f,  -63.9552f,  -77.1027f,   -6.3383f,
   205.4244f,   34.7675f,  -49.2628f,  -33.7821f,  519.1542f,  117.1530f,},
{   -2.2500f,   -0.5085f,   -1.0924f,   -3.4896f,   66.1016f,   18.5243f,
    12.7453f,    1.6321f,   47.2452f,   37.3260f,    4.4854f,  -14.8093f,}
};

const char* Shell_WelComMessage = 
"        Polaris robot embedded control service.\r\n"
" /$$$$$$$   /$$$$$$  /$$        /$$$$$$  /$$$$$$$  /$$$$$$  /$$$$$$ \r\n"
"| $$__  $$ /$$__  $$| $$       /$$__  $$| $$__  $$|_  $$_/ /$$__  $$\r\n"
"| $$  \\ $$| $$  \\ $$| $$      | $$  \\ $$| $$  \\ $$  | $$  | $$  \\__/\r\n"
"| $$$$$$$/| $$  | $$| $$      | $$$$$$$$| $$$$$$$/  | $$  |  $$$$$$ \r\n"
"| $$____/ | $$  | $$| $$      | $$__  $$| $$__  $$  | $$   \\____  $$\r\n"
"| $$      | $$  | $$| $$      | $$  | $$| $$ \\ $$  | $$   /$$  \\ $$\r\n"
"| $$      |  $$$$$$/| $$$$$$$$| $$  | $$| $$  | $$ /$$$$$$|  $$$$$$/\r\n"
"|__/       \\______/ |________/|__/  |__/|__/  |__/|______/ \\______/ \r\n"
"Type Help to view a list of registered commands.\r\n>Please enter the password:\r\n";

const char* Shell_EndOfMessage = "[Enter help to obtain the Polaris command help prompt]\r\n";
const char* Shell_Password = "Polar";
const char* Shell_LoginSuccess = "\r\nLogin successful\r\npolar>";
const char* Shell_LoginFailed = "\r\nLogin failed!\r\n>";                 
const char* Shell_PolarRoot = "polar:root";

// Peripheral interface mapping
SPI_HandleTypeDef* Const_BMI088_SPI_HANDLER         = &hspi1;
const float Const_BMI088_OFFLINE_TIME               = 0.3f;
I2C_HandleTypeDef* Const_BMP280_I2C_HANDLER         = &hi2c1;
const float Const_BMP280_OFFLINE_TIME               = 30.0f;
I2C_HandleTypeDef* Const_MAGNETIC_I2C_HANDLER       = &hi2c2;
const float Const_MAG_MAG_OFFLINE_TIME              = 0.3f;
const float Const_Motor_MOTOR_OFFLINE_TIME          = 0.05f;
I2C_HandleTypeDef* Const_OLED_I2C_HANDLER           = &hi2c1;
UART_HandleTypeDef* Const_Remote_UART_HANDLER       = &huart1;
const float Const_Remote_REMOTE_OFFLINE_TIME        = 1.0f;
const float Const_Protocol_OFFLINE_TIME             = 0.5f;
UART_HandleTypeDef* Const_LOG_UART_HANDLER          = &huart6;
const float Const_GimbalFdb_OFFLINE_TIME            = 0.5f;
UART_HandleTypeDef* Const_Shell_UART_HANDLER        = &huart2;
TIM_HandleTypeDef* Const_FreeRTOS_RUNTIME_TIMER_HANDLER = &htim2;
UART_HandleTypeDef* Const_Gimbal_UART_HANDLER       = &huart4;
