/*
 * Copyright 2011,
 *
 * Nicolas Testard
 *
 * LAAS, CNRS
 *
 * This file is part of Yoyoman01Controller.
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

