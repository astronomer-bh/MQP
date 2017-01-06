#ifndef Robot_hpp
#include "Pose.hpp"

class Robot
{
private:
  int m_id;
  Pose m_curPose;
  Pose m_initPose;
public:
  Robot(int id);
  Robot(int id, Pose pose);
  Robot(int id, double x, double y, double z, double yaw, double ptc, double rol);
  int dist();
  Pose diff();

  void printID();
  void printPose();

  int id();
  void setPose(Pose pose);
  void setPose(double x, double y, double z, double yaw, double ptc, double rol);
  Pose getPose();

};
#endif
