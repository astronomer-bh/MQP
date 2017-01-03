#include "Pose.hpp"

// Pose constructor made from coordinate/rotational input
Pose::Pose(double x, double y, double z, double yaw, double ptc, double rol)
  : m_x(x), m_y(y), m_z(z), m_yaw(yaw), m_ptc(ptc), m_rol(rol)
{}

// distance between two Poses via coordinates
double Pose::dist(Pose posn){
  return sqrt(pow((x() - posn.x()), 2) + pow((y() - posn.y()), 2) + pow((z() - posn.z()), 2));
}

// literal difference between two poses
Pose Pose::diff(Pose posn){
  return Pose(x() - posn.x(),
              y() - posn.y(),
              z() - posn.z(),
              yaw() - posn.yaw(),
              ptc() - posn.ptc(),
              rol() - posn.rol());
}

// printing all members of the Pose function
void Pose::print(){
  std::cout << "x: " << m_x
            << " y: " << m_y
            << " z: " << m_z
            << " yaw: " << m_yaw
            << " ptc: " << m_ptc
            << " rol: " << m_rol << std::endl;
}

// change pose information, basically jsut an update of pose
void Pose::setPose(double x, double y, double z, double yaw, double ptc, double rol){
  this->m_x = x;
  this->m_y = y;
  this->m_z = z;
  this->m_yaw = yaw;
  this->m_ptc = ptc;
  this->m_rol = rol;
}


//
// Getters and Setters
// ony define needed ones as to not overclutte
// or overwrite
//
double Pose::x(){
  return m_x;
}

double Pose::y(){
  return m_y;
}

double Pose::z(){
  return m_z;
}

double Pose::yaw(){
  return m_yaw;
}

double Pose::ptc(){
  return m_ptc;
}

double Pose::rol(){
  return m_rol;
}
