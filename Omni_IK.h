/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Omni_IK.h
  * @brief          : 全向底盘运动学
  * @author         : Chen
  * @date           : 2026/10/7
  * @version        : v1.0
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OMNI_IK_H
#define OMNI_IK_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Structs ------------------------------------------------------------------*/
typedef struct
{
    float chassis_radius;
    float wheel_radius;
    float torque_ratio;
} Omni_Chassis_Info_t;

/* Externs ------------------------------------------------------------------*/
extern void Omni_Follow_IK(
    float vx,
    float vy,
    float w,
    float output[4]);

extern void Omni_Spin_IK(
    float vx,
    float vy,
    float w,
    float gimbal_angle,
    float fixed_chassis_forward_angle,
    float output[4]);

#endif /* OMNI_IK_H */