#pragma once
#include <franka/robot.h>

#include <atomic>

class TorqueController {
 public:
  virtual franka::Torques step(const franka::RobotState& robot_state,
                               franka::Duration& duration) = 0;
  virtual void start(const franka::RobotState& robot_state,
                     std::shared_ptr<franka::Model> model) = 0;
  virtual void stop(const franka::RobotState& robot_state,
                    std::shared_ptr<franka::Model> model) = 0;
  virtual bool isRunning() = 0;
  virtual const std::string name() = 0;
  /// Called after step() with the torque actually sent to the robot, after
  /// the virtual walls, the torque rate limit and the clipping.
  virtual void commanded(const franka::RobotState& robot_state,
                         const franka::Torques& torques) {}

  void setTime(double time) { time_ = time; }

  double getTime() { return time_; }

 private:
  std::atomic<double> time_;
};
