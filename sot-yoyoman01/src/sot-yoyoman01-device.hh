/*
 * Copyright 2018,
 *
 * Nicolas Testard
 *
 * LAAS, CNRS
 *
 * This file is part of RomeoController.
 * RomeoController is not a free software, 
 * it contains information related to Romeo which involves
 * that you either purchased the proper license to havec access to
 * those informations, or that you signed the appropriate
 * Non-Disclosure agreement.
 *
 *
 */

#ifndef _SOT_Yoyoman01Device_H_
#define _SOT_Yoyoman01Device_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/matrix-geometry.hh>
//#include <sot/core/matrix-rotation.hh>

namespace dgsot=dynamicgraph::sot;
namespace dg=dynamicgraph;

class SoTYoyoman01Device: public 
dgsot::Device
{
 public:

  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string& getClassName () const		
  {  
    return CLASS_NAME;							    
  }
  
  SoTYoyoman01Device(std::string RobotName);
  virtual ~SoTYoyoman01Device();
  
  void setSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void timeStep(double ts) { timestep_ =ts;}

protected:
  // Update output port with the control computed from the
  // dynamic graph.
  

  /// \brief Current integration step.
  double timestep_;
  
  /// \brief Previous robot configuration.
  
  dg::Vector previousState_;
  
  /// \brief Robot state provided by OpenHRP.
  ///
  /// This corresponds to the real encoders values and take into
  /// account the stabilization step. Therefore, this usually
  /// does *not* match the state control input signal.
  ///
  

  /// Intermediate variables to avoid allocation during control


  
  /// Intermediate variables to avoid allocation during control
  std::vector<double> baseff_;

  /// Accelerations measured by accelerometers
  dynamicgraph::Signal <dg::Vector, int> accelerometerSOUT_;
  /// Rotation velocity measured by gyrometers
  dynamicgraph::Signal <dg::Vector, int> gyrometerSOUT_;
  /// joint angles
  dynamicgraph::Signal <dg::Vector, int> joint_anglesSOUT_;
  /// motor angles
  //dynamicgraph::Signal <dg::Vector, int> motor_anglesSOUT_;


  /// Protected methods for internal variables filling
  void setSensorsIMU(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsEncoders(std::map<std::string,dgsot::SensorValues> &SensorsIn, int t);
  
  /// Intermediate variables to avoid allocation during control
  //dg::Vector dgforces_;
  //dg::Vector dgRobotState_; // motor-angles
  dg::Vector joint_angles_; // joint-angles
  //dg::Vector motor_angles_; // motor-angles
  //dg::Vector dgRobotVelocity_; // motor velocities
  //dg::Vector velocities_; // motor velocities
  dgsot::MatrixRotation pose;
  dg::Vector accelerometer_;
  dg::Vector gyrometer_;

  
};
#endif /* _SOT_RomeoDevice_H_*/
