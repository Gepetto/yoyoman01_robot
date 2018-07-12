
/*
 * Copyright 2018,
 *
 * Nicolas Testard
 *
 * LAAS, CNRS
 *
 * This file is part of Yoyoman01Controller.
 */

#ifndef _SOT_YoyomanController_H_
#define _SOT_YoyomanController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>

#include "sot-yoyoman01-device.hh"
#include <dynamic_graph_bridge/ros_interpreter.hh>
namespace dgsot=dynamicgraph::sot;

class SoTYoyomanController: public 
  dgsot::AbstractSotExternalInterface
{
 public:

  static const std::string LOG_PYTHON;
  
  SoTYoyomanController();
  SoTYoyomanController(const char robotName[]);
  SoTYoyomanController(std::string robotName);
  virtual ~SoTYoyomanController();

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void setNoIntegration(void);
  void setSecondOrderIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);

  /// Run a python command 
  void runPython(std::ostream& file,
		 const std::string& command,
		 dynamicgraph::Interpreter& interpreter);
  
  virtual void startupPython();
    
  void init();

  SoTYoyoman01Device device_;
};

#endif /* _SOT_YoyomanController_H_ */

