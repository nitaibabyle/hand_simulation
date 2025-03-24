#pragma once

#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#define _USE_MATH_DEFINES
#include <math.h>

enum Events
{
  Wait = 1001,

};

enum class States : bitbot::StateId
{
  Waiting = 1001,

};

extern bitbot::MujocoJoint *l_joint1, *l_joint11, *l_joint2, *l_joint22, *l_joint3, *l_joint33, *l_joint4, *l_joint44, *l_joint5, *l_joint51, *l_joint52, *l_joint53, *r_joint1, *r_joint11, *r_joint2, *r_joint22, *r_joint3, *r_joint33, *r_joint4, *r_joint44, *r_joint5, *r_joint51, *r_joint52, *r_joint53;

struct UserData
{
  double sin = 10;
};

void ConfigFunc(const bitbot::MujocoBus &bus, UserData &);

std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData &user_data);

void StateWaiting(const bitbot::KernelInterface &kernel, bitbot::ExtraData &extra_data, UserData &user_data);

void FinishFunc(UserData &);
