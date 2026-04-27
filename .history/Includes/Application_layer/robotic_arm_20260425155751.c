#include "robotic_arm.h"
#include "define.h"
#include "pid.h"

RoboticArmState robotic_arm_state;

Motor[2] joint_1;
Motor[2] joint_2;
Motor[2] joint_3;
Motor[2] joint_4;
Motor[2] gripper;

void Robotic_Arm_Init(void) {
  // 在这里添加机械臂初始化的代码
}