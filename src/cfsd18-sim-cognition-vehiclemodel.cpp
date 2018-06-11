/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "vehiclemodel.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<=0) {
    std::cerr << argv[0] << " is a NEAT driver implementation for the CFSD18 project." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=120 --maxSteering=25.0 --maxAcceleration=5.0 --maxDeceleration=5.0" <<  std::endl;
    retCode = 1;
  } else {
    const float freq{(commandlineArguments["freq"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["freq"])) : (50.0f)};

    // Interface to a running OpenDaVINCI session.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    VehicleModel vehiclemodel(commandlineArguments, od4);

    auto bodyEnvelope{[&driver = vehiclemodel]() -> bool
    {
      driver.body();
      return true;
    }
    };
    auto dataEnvelope{[&driver = vehiclemodel](cluon::data::Envelope &&envelope)
      {
          driver.nextContainer(envelope);
      }
    };

    od4.dataTrigger(opendlv::proxy::GroundDecelerationRequest::ID(),dataEnvelope);
    od4.dataTrigger(opendlv::proxy::GroundAccelerationRequest::ID(),dataEnvelope);
    od4.dataTrigger(opendlv::logic::action::AimPoint::ID(),dataEnvelope);
    od4.timeTrigger(freq, bodyEnvelope);

  }
  return retCode;
}
