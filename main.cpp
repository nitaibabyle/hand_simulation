#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char const *argv[])
{
  // 声明kernel对象，参数为配置文件
  bitbot::MujocoKernel<UserData> kernel("../../../bitbot_hand.xml");

  // 注册ExtraData的标头
  kernel.RegisterExtraDataHeaders({"time", "sin"});

  // 注册Config函数
  kernel.RegisterConfigFunc(&ConfigFunc);
  // 注册Finish函数
  kernel.RegisterFinishFunc(&FinishFunc);

  // 注册event
  kernel.RegisterEvent("waiting", static_cast<bitbot::StateId>(Events::Wait), &EventWait, true);

  // 注册state
  kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting, {});

  // 设置用户的第一个state
  kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));

  // 运行
  kernel.Run();

  return 0;
}
