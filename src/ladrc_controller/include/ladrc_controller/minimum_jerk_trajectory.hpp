#ifndef LADRC_CONTROLLER__MINIMUM_JERK_TRAJECTORY_HPP_
#define LADRC_CONTROLLER__MINIMUM_JERK_TRAJECTORY_HPP_

#include <cmath>

namespace ladrc_controller
{

/**
 * @brief 点到点 Minimum Jerk (5 次多项式) 轨迹生成器
 *
 * s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 * 边界条件: pos(0)=p0, vel(0)=0, acc(0)=0, pos(T)=pT, vel(T)=0, acc(T)=0
 *
 * 解析系数:
 *   a0 = p0,  a1 = 0,  a2 = 0
 *   a3 = 10*dp/T^3,  a4 = -15*dp/T^4,  a5 = 6*dp/T^5
 */
class MinimumJerkTrajectory
{
public:
  struct TrajectoryPoint
  {
    double position;
    double velocity;
    double acceleration;
  };

  MinimumJerkTrajectory()
    : p0_(0.0), pT_(0.0), T_(0.0),
      a0_(0.0), a1_(0.0), a2_(0.0),
      a3_(0.0), a4_(0.0), a5_(0.0),
      initialized_(false)
  {}

  void initialize(double start_pos, double end_pos, double duration)
  {
    p0_ = start_pos;
    pT_ = end_pos;
    T_ = std::max(duration, 1e-3);  // 防止除零

    double dp = pT_ - p0_;
    double T2 = T_ * T_;
    double T3 = T2 * T_;
    double T4 = T3 * T_;
    double T5 = T4 * T_;

    a0_ = p0_;
    a1_ = 0.0;
    a2_ = 0.0;
    a3_ = 10.0 * dp / T3;
    a4_ = -15.0 * dp / T4;
    a5_ = 6.0 * dp / T5;

    initialized_ = true;
  }

  TrajectoryPoint evaluate(double t) const
  {
    TrajectoryPoint pt{};
    if (!initialized_)
    {
      pt.position = p0_;
      pt.velocity = 0.0;
      pt.acceleration = 0.0;
      return pt;
    }

    // 钳位到 [0, T]
    double tc = std::max(0.0, std::min(t, T_));
    double tc2 = tc * tc;
    double tc3 = tc2 * tc;
    double tc4 = tc3 * tc;

    pt.position     = a0_ + a1_*tc + a2_*tc2 + a3_*tc3 + a4_*tc4 + a5_*tc4*tc;
    pt.velocity     = a1_ + 2.0*a2_*tc + 3.0*a3_*tc2 + 4.0*a4_*tc3 + 5.0*a5_*tc4;
    pt.acceleration = 2.0*a2_ + 6.0*a3_*tc + 12.0*a4_*tc2 + 20.0*a5_*tc3;

    return pt;
  }

  bool isFinished(double t) const
  {
    return t >= T_;
  }

  double getDuration() const { return T_; }
  double getStartPosition() const { return p0_; }
  double getEndPosition() const { return pT_; }
  bool isInitialized() const { return initialized_; }

private:
  double p0_, pT_, T_;
  double a0_, a1_, a2_, a3_, a4_, a5_;
  bool initialized_;
};

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__MINIMUM_JERK_TRAJECTORY_HPP_
