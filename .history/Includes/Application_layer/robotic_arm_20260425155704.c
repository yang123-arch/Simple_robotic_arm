#include "robotic_arm.h"
#include "pid.h"

RoboticArmState robotic_arm_state;

Motor joint_1;
DoubleBuffer_t joint_2;
DoubleBuffer_t joint_3;
DoubleBuffer_t joint_4;
DoubleBuffer_t gripper;

void Robotic_Arm_Init(void) {
  // 在这里添加机械臂初始化的代码
}