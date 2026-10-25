/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Omni_IK.c
  * @brief          : 全向底盘运动学逆解算模块
  *                   支持随动模式与小陀螺模式两种控制策略
  * @author         : Chen
  * @date           : 2026/10/23
  * @version        : v1.1
  ******************************************************************************
  * @attention      vx、vy 遵循右手坐标系，w 以逆时针为正方向
  *                      ^ vx     /_   w
  *                 fl   |   fr   \  \
  *            vy     /  |  \         \
  *            <---------|---------   |
  *                   \  |  /         /
  *                 rl   |   rr      /
  *                      |
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Omni_IK.h"

/* Parameters ----------------------------------------------------------------*/
/**
 * @brief 底盘物理参数常量
 * @note  用户需要根据自己的底盘物理参数进行修改
 */
static const Omni_Chassis_Info_t Omni_Chassis_Info = {
    .chassis_radius  = 0.2f,                      /* 底盘半径 (m)，轮心到几何中心的水平距离 */
    .wheel_radius    = 0.152f,                    /* 全向轮半径 (m) */
    .torque_ratio    = (3591.0f / 187.0f),        /* DJI M3508 电机减速比 */
};

/* Functions ----------------------------------------------------------------*/
/**
 * @brief  随动模式运动学逆解算
 * @note   底盘跟随云台指向，保持底盘与云台坐标系一致。
 * @param  vx            底盘坐标系 X 轴目标速度 (m/s)
 * @param  vy            底盘坐标系 Y 轴目标速度 (m/s)
 * @param  w             底盘坐标系目标旋转角速度 (rad/s),需要外部进行pid计算
 * @param  output[4]     输出各轮电机目标角速度 (rad/s)，需要根据自己实际电机id排放位置进行修改，
 *
 * 逆解算公式（四轮全向底盘）：
 *   v_front_left  = -vx + vy + vw * R
 *   v_front_right =  vx + vy + vw * R
 *   v_rear_left   = -vx - vy + vw * R
 *   v_rear_right  =  vx - vy + vw * R
 *   其中 R = chassis_radius
 */
void Omni_Follow_IK(
    float vx,
    float vy,
    float w,
    float output[4])
{
    /* ---------- 全向轮运动学逆解算：目标速度 → 各轮线速度 (m/s) ---------- */
    float v_front_left  = -vx + vy + w * Omni_Chassis_Info.chassis_radius;
    float v_front_right =  vx + vy + w * Omni_Chassis_Info.chassis_radius;
    float v_rear_left   = -vx - vy + w * Omni_Chassis_Info.chassis_radius;
    float v_rear_right  =  vx - vy + w * Omni_Chassis_Info.chassis_radius;

    /* ---------- 转速转换系数: v/r * torque_ratio ---------- */
    float rad_s_factor = Omni_Chassis_Info.torque_ratio / Omni_Chassis_Info.wheel_radius;

    /* ---------- 线速度 → 电机转速 (rad/s) ---------- */
    output[0] = v_front_left * rad_s_factor;
    output[1] = v_rear_left  * rad_s_factor;
    output[2] = v_rear_right * rad_s_factor;
    output[3] = v_front_right * rad_s_factor;
}

/**
 * @brief  小陀螺模式运动学逆解算
 * @note   底盘绕自身中心高速自旋，同时在云台坐标系下完成平移运动。
 * @param  vx            云台坐标系 X 轴目标速度 (m/s)
 * @param  vy            云台坐标系 Y 轴目标速度 (m/s)
 * @param  w             底盘坐标系目标旋转角速度 (rad/s)
 * @param  gimbal_encoder 云台当前编码器值，6020电机编码器值范围[0, 8191]
 * @param  fixed_chassis_forward_encoder 底盘正方向编码器值，是一个固定值
 * @param  output[4]     输出各轮电机目标角速度 (rad/s)，需要根据自己实际电机id排放位置进行修改，
 *
 * 处理流程：
 *   1. 计算云台与底盘正方向的编码器值差 relative_encoder = gimbal_encoder - fixed_chassis_forward_encoder
 *   2. 将编码器值转换为角度 relative_angle = relative_encoder * (2.0f * 3.1415926f / 8191.0f)
 *   3. 通过二维旋转矩阵将 (vx, vy) 变换到底盘坐标系 (vx_chassis, vy_chassis)
 *   4. 在底盘坐标系下执行四轮全向运动学逆解算
 */
void Omni_Spin_IK(
    float vx,
    float vy,
    float w,
    int16_t gimbal_encoder,
    int16_t fixed_chassis_forward_encoder,
    float output[4])
{
    /* ---------- 计算云台与底盘的编码器值差 ---------- */
    int16_t relative_encoder = gimbal_encoder - fixed_chassis_forward_encoder;

    /* ---------- 编码器值转换为角度(rad) ---------- */
    float relative_angle = relative_encoder * (2.0f * 3.1415926f / 8191.0f);

    /* ---------- 向量旋转变换：云台坐标系 → 底盘坐标系 ---------- */
    float vx_chassis = vx * cosf(relative_angle) - vy * sinf(relative_angle);
    float vy_chassis = vx * sinf(relative_angle) + vy * cosf(relative_angle);

    /* ---------- 全向轮运动学逆解算：目标速度 → 各轮线速度 (m/s) ---------- */
    float v_front_left  = -vx_chassis + vy_chassis + w * Omni_Chassis_Info.chassis_radius;
    float v_front_right =  vx_chassis + vy_chassis + w * Omni_Chassis_Info.chassis_radius;
    float v_rear_left   = -vx_chassis - vy_chassis + w * Omni_Chassis_Info.chassis_radius;
    float v_rear_right  =  vx_chassis - vy_chassis + w * Omni_Chassis_Info.chassis_radius;

    /* ---------- 转速转换系数: v/r * torque_ratio ---------- */
    float rad_s_factor = Omni_Chassis_Info.torque_ratio / Omni_Chassis_Info.wheel_radius;
    
    /* ---------- 线速度 → 电机转速 (rad/s) ---------- */
    output[0] = v_front_left * rad_s_factor;
    output[1] = v_rear_left  * rad_s_factor;
    output[2] = v_rear_right * rad_s_factor;
    output[3] = v_front_right * rad_s_factor;
}
