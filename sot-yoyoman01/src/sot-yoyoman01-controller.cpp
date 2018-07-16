/*
 * Copyright 2018,
 *
 * Nicolas Testard
 *
 * LAAS, CNRS
 *
 * This file is part of Yoyoman01Controller.
 */


#include <sot/core/debug.hh>

/* yoyoman01 is inspired of yoyoman  but he is cheating because he is not passive at all (you can blame FCAM1 for that)*/

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

