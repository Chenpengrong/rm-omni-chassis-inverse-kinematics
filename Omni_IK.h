/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Omni_IK.h
  * @brief          : 全向底盘运动学
  * @author         : Chen
  * @date           : 2026/10/23
  * @version        : v1.1
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
    int16_t gimbal_encoder,
    int16_t fixed_chassis_forward_encoder,
    float output[4]);

#endif /* OMNI_IK_H */
