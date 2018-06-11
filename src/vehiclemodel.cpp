/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>
#include <cmath>
#include <thread>
#include "vehiclemodel.hpp"
#include <chrono>

VehicleModel::VehicleModel(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4),
  m_groundAccelerationMutex{},
  m_groundSteeringAngleMutex{},
  m_groundAcceleration{0.0},
  m_groundSteeringAngle{0.0},
  m_longitudinalSpeed{0.01},
  m_longitudinalSpeedDot{0.0},
  m_lateralSpeed{0.0},
  m_yawRate{0.0},
  m_prevSteerAngle{0.0},
  m_g{9.82}
{
 setUp(commandlineArguments);
}
VehicleModel::~VehicleModel()
{
}
void VehicleModel::setUp(std::map<std::string, std::string> commandlineArguments)
{
  m_senderStamp=(commandlineArguments["senderStamp"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["senderStamp"]))) : (m_senderStamp);
  m_frontToCog=(commandlineArguments["frontToCog"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["frontToCog"]))) : (m_frontToCog);

  m_mass=(commandlineArguments["mass"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["mass"]))) : (m_mass);
  m_momentOfInertiaZ=(commandlineArguments["momentOfInertiaZ"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["momentOfInertiaZ"]))) : (m_momentOfInertiaZ);
  m_length=(commandlineArguments["length"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["length"]))) : (m_length);
  m_frictionCoefficient=(commandlineArguments["frictionCoefficient"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["frictionCoefficient"]))) : (m_frictionCoefficient);
  m_rearToCog = m_length - m_frontToCog;
  m_magicFormulaCAlpha=(commandlineArguments["magicFormulaCAlpha"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["magicFormulaCAlpha"]))) : (m_magicFormulaCAlpha);
  m_magicFormulaC=(commandlineArguments["magicFormulaC"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["magicFormulaC"]))) : (m_magicFormulaC);
  m_magicFormulaE=(commandlineArguments["magicFormulaE"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["magicFormulaE"]))) : (m_magicFormulaE);
  double F =(commandlineArguments["freq"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["freq"]))) : (0.05);
  m_dt = 1.0/F;
  std::cout<<"VehicleModel set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    std::cout<<it->first<<" "<<it->second<<std::endl;
  }
}

void VehicleModel::tearDown()
{
}

void VehicleModel::nextContainer(cluon::data::Envelope &a_container)
{
  if (a_container.dataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    std::unique_lock<std::mutex> l(m_groundAccelerationMutex);
    auto groundDeceleration = cluon::extractMessage<opendlv::proxy::GroundDecelerationRequest>(std::move(a_container));
    m_groundAcceleration = -groundDeceleration.groundDeceleration();
    //std::cout<<"VehicleModel recieved groundDeceleration = "<<m_groundAcceleration<<"\n";
    //m_newAcc = true;
  } else if (a_container.dataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    std::unique_lock<std::mutex> l(m_groundAccelerationMutex);
    auto groundAcceleration = cluon::extractMessage<opendlv::proxy::GroundAccelerationRequest>(std::move(a_container));
    m_groundAcceleration = groundAcceleration.groundAcceleration();
    //std::cout<<"VehicleModel recieved groundAcceleration = "<<m_groundAcceleration<<"\n";
    //m_newAcc = true;
  } else if (a_container.dataType() == opendlv::logic::action::AimPoint::ID()) {
    std::unique_lock<std::mutex> m(m_groundSteeringAngleMutex);
    auto groundSteeringAngle = cluon::extractMessage<opendlv::logic::action::AimPoint>(std::move(a_container));
    m_groundSteeringAngle = groundSteeringAngle.azimuthAngle();
    //std::cout<<"VehicleModel recieved AimPoint = "<<m_groundSteeringAngle<<"\n";
  }
}

void VehicleModel::body(){
  double groundAccelerationCopy;
  double groundSteeringAngleCopy;
  {
    std::unique_lock<std::mutex> l(m_groundAccelerationMutex);
    std::unique_lock<std::mutex> m(m_groundSteeringAngleMutex);
    groundAccelerationCopy = m_groundAcceleration;
    groundSteeringAngleCopy = m_groundSteeringAngle;
  }
  //std::cout<<"VehicleModel body: "<<"groundAccelerationCopy: "<<groundAccelerationCopy<<" AimPoint: "<<m_groundSteeringAngle<<std::endl;
  if (std::abs(groundSteeringAngleCopy-m_prevSteerAngle)/m_dt>(80.0*3.14159265/180.0)){
    if (groundSteeringAngleCopy > m_prevSteerAngle) {
      groundSteeringAngleCopy = m_dt*80.0*3.14159265/180.0 + m_prevSteerAngle;
    }
    else{
      groundSteeringAngleCopy = -m_dt*80.0*3.14159265/180.0 + m_prevSteerAngle;
    }
  }
  //std::cout<<"grounSteering: "<<groundSteeringAngleCopy<<std::endl;

  /*if (//m_newAcc) {
    ei = 0.0;
    ePrev = 0.0;
    //m_newAcc = false;
  }
  e = groundAccelerationCopy-m_longitudinalSpeedDot;
  ei += e*m_dt;
  ed = (e-ePrev)/m_dt;
  double acceleration = kp*e+kd*ed+ki*ei;
  std::cout<<"groundAccelerationCopy: "<<groundAccelerationCopy<<endl;
  std::cout<<"m_longitudinalSpeedDot: "<<m_longitudinalSpeedDot<<endl;
  std::cout<<"ACCELERATION: "<<acceleration<<endl;
  std::cout<<"e: "<<e<<endl;
  std::cout<<"ePrev: "<<ePrev<<endl;
  std::cout<<"m_dt: "<<m_dt<<endl;
  std::cout<<"ed: "<<ed<<endl;
  std::cout<<"ei: "<<ei<<endl;
  std::cout<<"kp*e: "<<kp*e<<endl;
  std::cout<<"kd*ed: "<<kd*ed<<endl;
  std::cout<<"ki*ei: "<<ki*ei<<endl;
  if (acceleration > 5.0) {
    acceleration = 5.0;
  }else if(acceleration < -5.0){
    acceleration = -5.0;
  }*/

  double slipAngleFront = groundSteeringAngleCopy - std::atan(
      (m_lateralSpeed + m_frontToCog * m_yawRate) / std::abs(m_longitudinalSpeed));
  double slipAngleRear = -std::atan((m_lateralSpeed - m_rearToCog * m_yawRate) /
      std::abs(m_longitudinalSpeed));

  double forceFrontZ = m_mass * m_g * (m_frontToCog / (m_frontToCog + m_length));
  double forceRearZ = m_mass * m_g * (m_length / (m_frontToCog + m_length));

  double forceFrontY = magicFormula(slipAngleFront, forceFrontZ,
      m_frictionCoefficient, m_magicFormulaCAlpha, m_magicFormulaC, m_magicFormulaE);
  double forceRearY = magicFormula(slipAngleRear, forceRearZ,
      m_frictionCoefficient, m_magicFormulaCAlpha, m_magicFormulaC, m_magicFormulaE);

  double rollResistance;
  if (m_longitudinalSpeed>0) {rollResistance = -9.81*0.02;}
  else if (m_longitudinalSpeed<0){rollResistance = 9.81*0.02;}
  else {rollResistance = 0.0;}

  m_longitudinalSpeedDot = groundAccelerationCopy - std::sin(groundSteeringAngleCopy)*forceFrontY/m_mass + m_yawRate * m_lateralSpeed + rollResistance;

  double lateralSpeedDot =
    (forceFrontY * std::cos(groundSteeringAngleCopy) + forceRearY) / m_mass -
    m_yawRate * m_longitudinalSpeed;

  double yawRateDot = (m_frontToCog * forceFrontY *
      std::cos(groundSteeringAngleCopy) - m_rearToCog * forceRearY) /
    m_momentOfInertiaZ;

  m_longitudinalSpeed += m_longitudinalSpeedDot * m_dt;
  if (m_longitudinalSpeed<0) {
    m_longitudinalSpeed = 0.01;
  }
  m_lateralSpeed += lateralSpeedDot * m_dt;
  m_yawRate += yawRateDot * m_dt;

  //ePrev = e;
  m_prevSteerAngle = groundSteeringAngleCopy;
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
  opendlv::sim::KinematicState kinematicState;
  kinematicState.vx(static_cast<float>(m_longitudinalSpeed));
  kinematicState.vy(static_cast<float>(m_lateralSpeed));
  kinematicState.yawRate(static_cast<float>(m_yawRate));
  kinematicState.vz(static_cast<float>(0.0));
  kinematicState.rollRate(static_cast<float>(0.0));
  kinematicState.pitchRate(static_cast<float>(0.0));
  m_od4.send(kinematicState, sampleTime, m_senderStamp);
  float groundSpeed = static_cast<float>(sqrt(pow(m_longitudinalSpeed,2)+pow(m_lateralSpeed,2)));
  opendlv::proxy::GroundSpeedReading groundSpeedReading;
  groundSpeedReading.groundSpeed(groundSpeed);
  m_od4.send(groundSpeedReading, sampleTime, m_senderStamp);
  //std::cout<<"VehicleModel sends: "<<" vx: "<<m_longitudinalSpeed<<" vy: "<<m_lateralSpeed<<" yawRate: "<<m_yawRate<<" groundSpeed: "<<groundSpeed<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime)<< " senderStamp: "<<m_senderStamp<<"\n";
}

double VehicleModel::magicFormula(double const &a_slipAngle, double const &a_forceZ,
    double const &a_frictionCoefficient, double const &a_cAlpha, double const &a_c,
    double const &a_e) const
{
  double const b = a_cAlpha / (a_c * a_frictionCoefficient * a_forceZ);
  double const forceY = a_frictionCoefficient * a_forceZ * std::sin(a_c *
     std::atan(b * a_slipAngle - a_e * (b * a_slipAngle - std::atan(b * a_slipAngle))));
  return forceY;
}
