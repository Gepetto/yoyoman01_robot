/*
 * Copyright 2011,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of RomeoController.
 */
/*
#ifndef _SOT_RomeoController_H_
#define _SOT_RomeoController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

#include "sot-romeo-device.hh"

namespace dgsot=dynamicgraph::sot;

class SoTRomeoController: public 
  dgsot::AbstractSotExternalInterface
{
 public:

  static const std::string LOG_PYTHON;
  
  SoTRomeoController();
  virtual ~SoTRomeoController();

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);
  
  void runPython(std::ostream& file,
		 const std::string& command,
		 dynamicgraph::Interpreter& interpreter);
  
  virtual void startupPython();
    

  SoTRomeoDevice device_;
};

#endif // _SOT_RomeoController_H_ 
*/

#ifndef _SOT_YOYOMAN01_Controller_H_
#define _SOT_YOYOMAN01_Controller_H_

#include "sot-yoyoman-controller.hh"
namespace dgsot=dynamicgraph::sot;

class SoTYoyoman01Controller: public SoTYoyomanController
{
 public:
  static const std::string LOG_PYTHON_YOYOMAN01;

  SoTYoyoman01Controller();
  virtual ~SoTYoyoman01Controller() {};


 protected:

  virtual void startupPython();
  

};

#endif // _SOTYoyoman01Controller_H_ 

