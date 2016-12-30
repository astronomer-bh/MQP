#include "Robot.hpp"

// initializes robot if given Pose directly
Robot::Robot(int id, Pose pose)
  : m_id(id), m_initPose(pose), m_curPose(pose){}

// initializes robot if given pose as set of doubles
Robot::Robot(int id, double x, double y, double z, double yaw, double ptc, double rol)
  : m_id(id), m_initPose(x, y, z, yaw, ptc, rol), m_curPose(x, y, z, yaw, ptc, rol) {}

// give current dist from origin
int Robot::dist(){
  return m_curPose.dist(m_initPose);
}

// returns diff between intial pose and cur pose
Pose Robot::diff(){
  return m_curPose.diff(m_initPose);
}

// prints robot ID
void Robot::printID(){
  std::cout << "ID: " << m_id << std::endl;
}

// prints current pose relative to initial pose
void Robot::printPose(){
  diff().print();
}

//
// Getters and Setters
// ony define needed ones as to not overclutte
// or overwrite
//
int Robot::id(){
  return m_id;
}

void Robot::setPose(Pose pose){
  this->m_curPose = pose;
}

void Robot::setPose(double x, double y, double z, double yaw, double ptc, double rol){
  m_curPose.setPose(x, y, z, yaw, ptc, rol);
}

Pose Robot::getPose(){
  return m_curPose;
}
