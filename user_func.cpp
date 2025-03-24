#include "user_func.h"

#include <chrono>
#include <memory>

bitbot::MujocoJoint *l_joint1, *l_joint11, *l_joint2, *l_joint22, *l_joint3, *l_joint33, *l_joint4, *l_joint44, *l_joint5, *l_joint51, *l_joint52, *l_joint53, *r_joint1, *r_joint11, *r_joint2, *r_joint22, *r_joint3, *r_joint33, *r_joint4, *r_joint44, *r_joint5, *r_joint51, *r_joint52, *r_joint53;

void ConfigFunc(const bitbot::MujocoBus &bus, UserData &)
{
  l_joint1 = bus.GetDevice<bitbot::MujocoJoint>(1).value();
  l_joint11 = bus.GetDevice<bitbot::MujocoJoint>(2).value();
  l_joint2 = bus.GetDevice<bitbot::MujocoJoint>(3).value();
  l_joint22 = bus.GetDevice<bitbot::MujocoJoint>(4).value();
  l_joint3 = bus.GetDevice<bitbot::MujocoJoint>(5).value();
  l_joint33 = bus.GetDevice<bitbot::MujocoJoint>(6).value();
  l_joint4 = bus.GetDevice<bitbot::MujocoJoint>(7).value();
  l_joint44 = bus.GetDevice<bitbot::MujocoJoint>(8).value();
  l_joint5 = bus.GetDevice<bitbot::MujocoJoint>(9).value();
  l_joint51 = bus.GetDevice<bitbot::MujocoJoint>(10).value();
  l_joint52 = bus.GetDevice<bitbot::MujocoJoint>(11).value();
  l_joint53 = bus.GetDevice<bitbot::MujocoJoint>(12).value();

  r_joint1 = bus.GetDevice<bitbot::MujocoJoint>(21).value();
  r_joint11 = bus.GetDevice<bitbot::MujocoJoint>(22).value();
  r_joint2 = bus.GetDevice<bitbot::MujocoJoint>(23).value();
  r_joint22 = bus.GetDevice<bitbot::MujocoJoint>(24).value();
  r_joint3 = bus.GetDevice<bitbot::MujocoJoint>(25).value();
  r_joint33 = bus.GetDevice<bitbot::MujocoJoint>(26).value();
  r_joint4 = bus.GetDevice<bitbot::MujocoJoint>(27).value();
  r_joint44 = bus.GetDevice<bitbot::MujocoJoint>(28).value();
  r_joint5 = bus.GetDevice<bitbot::MujocoJoint>(29).value();
  r_joint51 = bus.GetDevice<bitbot::MujocoJoint>(30).value();
  r_joint52 = bus.GetDevice<bitbot::MujocoJoint>(31).value();
  r_joint53 = bus.GetDevice<bitbot::MujocoJoint>(32).value();
}

std::optional<bitbot::StateId> EventWait(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::Waiting);
}

void StateWaiting(const bitbot::KernelInterface &kernel, bitbot::ExtraData &extra_data, UserData &user_data)
{
  constexpr double deg2rad = M_PI / 180.0;
  constexpr double rad2deg = 180.0 / M_PI;

  static double start_time = 0;
  static bool init = false;

  double t = kernel.GetPeriodsCount() * 0.004 - start_time;

  l_joint1->SetTargetPosition(1 * user_data.sin * deg2rad * (-1 + sin(0.2 * M_PI * t)));
  l_joint11->SetTargetPosition(1 * user_data.sin * deg2rad * (-1 + cos(0.2 * M_PI * t)));
  l_joint2->SetTargetPosition(1.5 * user_data.sin * deg2rad * (-0.8 + sin(0.2 * M_PI * t + 1)));
  l_joint22->SetTargetPosition(1.5 * user_data.sin * deg2rad * (-0.8 + cos(0.2 * M_PI * t + 0.5)));
  l_joint3->SetTargetPosition(3.2 * user_data.sin * deg2rad * (-0.5 + sin(0.2 * M_PI * t + 2)));
  l_joint33->SetTargetPosition(3.2 * user_data.sin * deg2rad * (-0.5 + cos(0.2 * M_PI * t + 1.5)));
  l_joint4->SetTargetPosition(4 * user_data.sin * deg2rad * (-0.7 + sin(0.2 * M_PI * t + 3)));
  l_joint44->SetTargetPosition(4 * user_data.sin * deg2rad * (-0.7 + cos(0.2 * M_PI * t + 1.7)));
  l_joint5->SetTargetPosition(1 * user_data.sin * deg2rad * (0 + sin(0.2 * M_PI * t)));
  l_joint51->SetTargetPosition(1.5 * user_data.sin * deg2rad * (1.5 + cos(0.2 * M_PI * t)));
  l_joint52->SetTargetPosition(user_data.sin * deg2rad * (0 + sin(0.2 * M_PI * t)));
  l_joint53->SetTargetPosition(user_data.sin * deg2rad * (0 + cos(0.2 * M_PI * t)));

  l_joint1->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 + cos(0.2 * M_PI * t)));
  l_joint11->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 - sin(0.2 * M_PI * t)));
  l_joint2->SetTargetVelocity(1.5 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.8 + cos(0.2 * M_PI * t + 1)));
  l_joint22->SetTargetVelocity(1.5 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.8 - sin(0.2 * M_PI * t + 0.5)));
  l_joint3->SetTargetVelocity(3.2 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.5 + cos(0.2 * M_PI * t + 2)));
  l_joint33->SetTargetVelocity(3.2 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.5 - sin(0.2 * M_PI * t + 1.5)));
  l_joint4->SetTargetVelocity(4 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.7 + cos(0.2 * M_PI * t + 3)));
  l_joint44->SetTargetVelocity(4 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.7 - sin(0.2 * M_PI * t + 1.7)));
  // l_joint5->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 + cos(0.2 * M_PI * t)));
  // l_joint51->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 - sin(0.2 * M_PI * t)));
  // l_joint52->SetTargetVelocity(user_data.sin * deg2rad * 0.2 * M_PI * (1 + cos(0.2 * M_PI * t)));
  // l_joint53->SetTargetVelocity(user_data.sin * deg2rad * 0.2 * M_PI * (1 - sin(0.2 * M_PI * t)));

  r_joint1->SetTargetPosition(1 * user_data.sin * deg2rad * (-1 + sin(0.2 * M_PI * t)));
  r_joint11->SetTargetPosition(1 * user_data.sin * deg2rad * (-1 + cos(0.2 * M_PI * t)));
  r_joint2->SetTargetPosition(1.5 * user_data.sin * deg2rad * (-0.8 + sin(0.2 * M_PI * t + 1)));
  r_joint22->SetTargetPosition(1.5 * user_data.sin * deg2rad * (-0.8 + cos(0.2 * M_PI * t + 0.5)));
  r_joint3->SetTargetPosition(3.2 * user_data.sin * deg2rad * (-0.5 + sin(0.2 * M_PI * t + 2)));
  r_joint33->SetTargetPosition(3.2 * user_data.sin * deg2rad * (-0.5 + cos(0.2 * M_PI * t + 1.5)));
  r_joint4->SetTargetPosition(4 * user_data.sin * deg2rad * (-0.7 + sin(0.2 * M_PI * t + 3)));
  r_joint44->SetTargetPosition(4 * user_data.sin * deg2rad * (-0.7 + cos(0.2 * M_PI * t + 1.7)));
  r_joint5->SetTargetPosition(-1 * user_data.sin * deg2rad * (0 + sin(0.2 * M_PI * t)));
  r_joint51->SetTargetPosition(-1.5 * user_data.sin * deg2rad * (1.5 + cos(0.2 * M_PI * t)));
  r_joint52->SetTargetPosition(user_data.sin * deg2rad * (0 + sin(0.2 * M_PI * t)));
  r_joint53->SetTargetPosition(user_data.sin * deg2rad * (0 + cos(0.2 * M_PI * t)));

  r_joint1->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 + cos(0.2 * M_PI * t)));
  r_joint11->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (-1 - sin(0.2 * M_PI * t)));
  r_joint2->SetTargetVelocity(1.5 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.8 + cos(0.2 * M_PI * t + 1)));
  r_joint22->SetTargetVelocity(1.5 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.8 - sin(0.2 * M_PI * t + 0.5)));
  r_joint3->SetTargetVelocity(3.2 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.5 + cos(0.2 * M_PI * t + 2)));
  r_joint33->SetTargetVelocity(3.2 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.5 - sin(0.2 * M_PI * t + 1.5)));
  r_joint4->SetTargetVelocity(4 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.7 + cos(0.2 * M_PI * t + 3)));
  r_joint44->SetTargetVelocity(4 * user_data.sin * deg2rad * 0.2 * M_PI * (-0.7 - sin(0.2 * M_PI * t + 1.7)));
  // r_joint5->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (1 + cos(0.2 * M_PI * t)));
  // r_joint51->SetTargetVelocity(1 * user_data.sin * deg2rad * 0.2 * M_PI * (1 - sin(0.2 * M_PI * t)));
  // r_joint52->SetTargetVelocity(user_data.sin * deg2rad * 0.2 * M_PI * (1 + cos(0.2 * M_PI * t)));
  // r_joint53->SetTargetVelocity(user_data.sin * deg2rad * 0.2 * M_PI * (1 - sin(0.2 * M_PI * t)));

  extra_data[0] = t;
  extra_data[1] = user_data.sin;
}
// zhangjintao

void FinishFunc(UserData &)
{
  std::cout << "finish" << std::endl;
}
