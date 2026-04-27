#include "robotic_arm.h"
#include "Double_buffer.h"
#include "define.h"
#include "pid.h"

RoboticArmState robotic_arm_state;

Motor joint_1[2];
Motor joint_2[2];
Motor joint_3[2];
Motor joint_4[2];
Motor gripper[2];

void Robotic_Arm_Init(void) {
  // 在这里添加机械臂初始化的代码
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.joint1,
                                    (uint8_t *)&joint_1[0],
                                    (uint8_t *)&joint_1[1], sizeof(joint_1));
}