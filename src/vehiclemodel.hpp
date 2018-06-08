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

#ifndef OPENDLV_SIM_LYNX_VEHICLEMODEL_HPP
#define OPENDLV_SIM_LYNX_VEHICLEMODEL_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <map>
#include <chrono>

class VehicleModel {
 public:
  VehicleModel(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4);
  VehicleModel(VehicleModel const &) = delete;
  VehicleModel &operator=(VehicleModel const &) = delete;
  virtual ~VehicleModel();
  void nextContainer(cluon::data::Envelope &);
  void body();

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);
  void tearDown();
  double magicFormula(double const &, double const &, double const &,
      double const &, double const &, double const &) const;

  /* commandlineArguments */
  cluon::OD4Session &m_od4;
  int m_senderStamp{203};
  double m_frontToCog{0.765f};
  double m_mass{188.0};
  double m_momentOfInertiaZ{105.0};
  double m_length{1.53};
  double m_frictionCoefficient{0.9};
  double m_rearToCog{0.0};
  double m_magicFormulaCAlpha{25229.0};
  double m_magicFormulaC{1.0};
  double m_magicFormulaE{-2.0};
  double m_dt{0.05};
  /* Member variables */
  std::mutex m_groundAccelerationMutex;
  std::mutex m_groundSteeringAngleMutex;
  double m_groundAcceleration;
  double m_groundSteeringAngle;
  double m_longitudinalSpeed;
  double m_longitudinalSpeedDot;
  double m_lateralSpeed;
  double m_yawRate;
  double m_prevSteerAngle;
  double m_g;
};

#endif
