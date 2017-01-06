#ifndef Pose_hpp
#include <math.h>
#include <iostream>

class Pose
{
private:
  double m_x;
  double m_y;
  double m_z;
  double m_yaw;
  double m_ptc;
  double m_rol;

public:
  Pose(double x, double y, double z, double yaw, double ptc, double rol);
  Pose();
  double dist(Pose posn);
  Pose diff(Pose posn);

  void print();

  void setPose(double x, double y, double z, double yaw, double ptc, double rol);

  double x();
  double y();
  double z();
  double yaw();
  double ptc();
  double rol();
};
#endif
