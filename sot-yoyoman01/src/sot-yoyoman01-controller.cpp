/*
 * Copyright 2011,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of romeoController.
 */
/*
#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>
#include <dynamic_graph_bridge/ros_init.hh>
#include <dynamic_graph_bridge/ros_interpreter.hh>

#include "sot-romeo-controller.hh"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
const std::string SoTRomeoController::LOG_PYTHON="/tmp/RomeoController_python.out";

using namespace std;

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

#define ROBOTNAME std::string("ROMEO")

void workThread(SoTRomeoController *aSoTRomeo)
{
  
  dynamicgraph::Interpreter aLocalInterpreter(dynamicgraph::rosInit(false,true));

  aSoTRomeo->interpreter_ = 
    boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);
  std::cout << "Going through the thread." << std::endl;
  {
    boost::lock_guard<boost::mutex> lock(mut);
    data_ready=true;
  }
  cond.notify_all();
  ros::waitForShutdown();
}

SoTRomeoController::SoTRomeoController():
  device_(ROBOTNAME)
{

  std::cout << "Going through SoTRomeoController." << std::endl;
  boost::thread thr(workThread,this);
  sotDEBUG(25) << __FILE__ << ":" 
	       << __FUNCTION__ <<"(#" 
	       << __LINE__ << " )" << std::endl;

  boost::unique_lock<boost::mutex> lock(mut);
  cond.wait(lock);

  startupPython();
  interpreter_->startRosService ();
}

SoTRomeoController::~SoTRomeoController()
{
}

void SoTRomeoController::
setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.setupSetSensors(SensorsIn);
}


void SoTRomeoController::
nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  device_.nominalSetSensors(SensorsIn);
}

void SoTRomeoController::
cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  device_.cleanupSetSensors(SensorsIn);
}


void SoTRomeoController::
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

void SoTRomeoController::
runPython(std::ostream& file,
	  const std::string& command,
	  dynamicgraph::Interpreter& interpreter)
{
  file << ">>> " << command << std::endl;
  std::string lerr(""),lout(""),lres("");
  interpreter.runCommand(command,lres,lout,lerr);
  if (lres != "None")
    {
      if (lres=="<NULL>")
	{
	  file << lout << std::endl;
	  file << "------" << std::endl;
	  file << lerr << std::endl;
	}
      else
	file << lres << std::endl;
    }
}

void SoTRomeoController::
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

  runPython
    (aof,
     "from dynamic_graph.sot.romeo.prologue import robot",
     *interpreter_);
  aof.close();
}


extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTRomeoController;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}

*/

#include <sot/core/debug.hh>

/* yoyoman01 is inspired of yoyoman  but he is cheating because he is not passive at all (you can blame florenc for that)*/

#define ROBOTNAME std::string("YOYOMAN01")

#include "sot-yoyoman01-controller.hh"

const std::string SoTYoyoman01Controller::LOG_PYTHON_YOYOMAN01="/tmp/Yoyoman01Controller_python.out";

SoTYoyoman01Controller::SoTYoyoman01Controller():
  SoTYoyomanController(ROBOTNAME)
{
  startupPython();
  interpreter_->startRosService ();
}

void SoTYoyoman01Controller::startupPython()
{
  SoTYoyomanController::startupPython();
  std::ofstream aof(LOG_PYTHON_YOYOMAN01.c_str());
  
  runPython
    (aof,
     "from dynamic_graph.sot.yoyoman01.prologue import robot",
     *interpreter_);
  aof.close();
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTYoyoman01Controller;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}

