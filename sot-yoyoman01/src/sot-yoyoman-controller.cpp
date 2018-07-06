/* 
 * Copyright 2016,
 *
 * Rohan Budhiraja
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of TALOSController.
 * TALOSController is a free software, 
 *
 */

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

#include "sot-yoyoman-controller.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <ros/console.h>

const std::string SoTYoyomanController::LOG_PYTHON="/tmp/YoyomanController_python.out";

using namespace std;

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;


void workThread(SoTYoyomanController *aSoTYoyoman)
{
  
  dynamicgraph::Interpreter aLocalInterpreter(dynamicgraph::rosInit(false,true));

  aSoTYoyoman->interpreter_ = 
    boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);
  std::cout << "Going through the thread." << std::endl;
  {
    boost::lock_guard<boost::mutex> lock(mut);
    data_ready=true;
  }
  cond.notify_all();
  ros::waitForShutdown();
}

SoTYoyomanController::SoTYoyomanController(std::string RobotName):
  device_(RobotName)
{
  init();
}

SoTYoyomanController::SoTYoyomanController(const char robotName[]):
  device_(robotName)
{
  init();
}

void SoTYoyomanController::init()
{
  std::cout << "Going through SoTYoyomanController." << std::endl;
  boost::thread thr(workThread,this);
  sotDEBUG(25) << __FILE__ << ":" 
	       << __FUNCTION__ <<"(#" 
	       << __LINE__ << " )" << std::endl;

  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);

  double ts = ros::param::param<double> ("/sot_controller/dt", SoTYoyoman01Device::TIMESTEP_DEFAULT);
  device_.timeStep(ts);
}

SoTYoyomanController::~SoTYoyomanController()
{
}

void SoTYoyomanController::
setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.setupSetSensors(SensorsIn);
}


void SoTYoyomanController::
nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.nominalSetSensors(SensorsIn);
}

void SoTYoyomanController::
cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  device_.cleanupSetSensors(SensorsIn);
}


void SoTYoyomanController::
getControl(map<string,dgsot::ControlValues> &controlOut)
{
  try 
    {
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
      device_.getControl(controlOut);
      sotDEBUG(25) << __FILE__ << __FUNCTION__ << "(#" << __LINE__ << ")" << endl;
    }
  catch ( dynamicgraph::sot::ExceptionAbstract & err)
    {

      std::cout << __FILE__ << " " 
		<< __FUNCTION__ << " (" 
		<< __LINE__ << ") " 
		<< err.getStringMessage() 
		<<  endl;
      throw err;
    }
}

void SoTYoyomanController::
setNoIntegration(void)
{
  device_.setNoIntegration();
}

void SoTYoyomanController::
setSecondOrderIntegration(void)
{
  device_.setSecondOrderIntegration();
}

void SoTYoyomanController::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string lres(""),lout(""),lerr("");
  interpreter.runCommand(command,lres,lout,lerr);

  if (lres != "None")
    {
      if (lres=="<NULL>")
	{
	  file << lout << std::endl;
	  file << "------" << std::endl;
	  file << lerr << std::endl;
	  ROS_INFO(lout.c_str());
	  ROS_ERROR(lerr.c_str());
	}
      else
	{
	  file << lres << std::endl;
	  ROS_INFO(lres.c_str());
	}
    }
}

void SoTYoyomanController::
startupPython()
{
  std::ofstream aof(LOG_PYTHON.c_str());
  runPython (aof, "import sys, os", *interpreter_);
  runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
  runPython (aof, "path = []", *interpreter_);
  runPython (aof,
	     "for p in pythonpath.split(':'):\n"
	     "  if p not in sys.path:\n"
	     "    path.append(p)", *interpreter_);
  runPython (aof, "path.extend(sys.path)", *interpreter_);
  runPython (aof, "sys.path = path", *interpreter_);

  // Calling again rosInit here to start the spinner. It will
  // deal with topics and services callbacks in a separate, non
  // real-time thread. See roscpp documentation for more
  // information.
  dynamicgraph::rosInit (true);
  aof.close();
}
