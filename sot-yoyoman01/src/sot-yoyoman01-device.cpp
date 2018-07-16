/*
 * Copyright 2018,
 *
 * Nicolas Testard
 *
 * LAAS, CNRS
 *
 * This file is part of Yoyoman01Controller.
 *
 */

#include <fstream>
#include <map>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/sot-yoyoman-device.txt"

#if 1
#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}

#else
// Void the macro
#define RESETDEBUG5()
#define ODEBUG5FULL(x)
#define ODEBUG5(x)
#endif


#include <sot/core/debug.hh>

#include "sot-yoyoman01-device.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace std;

const double SoTYoyoman01Device::TIMESTEP_DEFAULT = 0.001;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SoTYoyoman01Device,"Device");

SoTYoyoman01Device::SoTYoyoman01Device(std::string RobotName):
  dgsot::Device(RobotName),
  timestep_(TIMESTEP_DEFAULT),
  previousState_ (),
  baseff_ (),
  accelerometerSOUT_("StackOfTasks(" + RobotName + ")::output(vector)::accelerometer"),
  gyrometerSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::gyrometer"),
  //motor_anglesSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::motor_angles"),
  joint_anglesSOUT_ ("StackOfTasks(" + RobotName + ")::output(vector)::joint_angles"),
  accelerometer_ (3),
  gyrometer_ (3) 
{
  RESETDEBUG5();
  sotDEBUGIN(25) ;
  for( int i=0;i<4;++i ) { withForceSignals[i] = true; } //probably useless
  signalRegistration (accelerometerSOUT_ 
                   << gyrometerSOUT_
                   << joint_anglesSOUT_ );
                   //<< motor_anglesSOUT_);
  dg::Vector data (3); data.setZero ();
  accelerometerSOUT_.setConstant (data);
  gyrometerSOUT_.setConstant (data);
  baseff_.resize(12); // a mediter
  dg::Vector dataForces(6); dataForces.setZero ();
  for(int i=0;i<4;i++)
    forcesSOUT[i]->setConstant(dataForces);


  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  docstring =
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand("increment",
	     makeCommandVoid1((Device&)*this,
			      &Device::increment, docstring));
  
  sotDEBUGOUT(25);
}

SoTYoyoman01Device::~SoTYoyoman01Device()
{ }


void SoTYoyoman01Device::setSensorsIMU(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  //TODO: Confirm if this can be made quaternion
  it = SensorsIn.find("attitude");
  if (it!=SensorsIn.end())
  {
    const vector<double>& attitude = it->second.getValues ();
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        pose (i, j) = attitude [i * 3 + j];
    attitudeSOUT.setConstant (pose);
    attitudeSOUT.setTime (t);
  }

  it = SensorsIn.find("accelerometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& accelerometer =
        SensorsIn ["accelerometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      accelerometer_ (i) = accelerometer [i];
    accelerometerSOUT_.setConstant (accelerometer_);
    accelerometerSOUT_.setTime (t);
  }

  it = SensorsIn.find("gyrometer_0");
  if (it!=SensorsIn.end())
  {
    const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
    for (std::size_t i=0; i<3; ++i)
      gyrometer_ (i) = gyrometer [i];
    gyrometerSOUT_.setConstant (gyrometer_);
    gyrometerSOUT_.setTime (t);
  }
}

void SoTYoyoman01Device::setSensorsEncoders(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  map<string,dgsot::SensorValues>::iterator it;
  /*
  it = SensorsIn.find("motor-angles");
  if (it!=SensorsIn.end())
  {
    const vector<double>& anglesIn = it->second.getValues();
    dgRobotState_.resize (anglesIn.size () + 6);
    motor_angles_.resize(anglesIn.size ());
    for (unsigned i = 0; i < 6; ++i)
      dgRobotState_ (i) = 0.;
    for (unsigned i = 0; i < anglesIn.size(); ++i)
      {
	dgRobotState_ (i + 6) = anglesIn[i];
	motor_angles_(i)= anglesIn[i];
      }
    robotState_.setConstant(dgRobotState_);
    robotState_.setTime(t);
    motor_anglesSOUT_.setConstant(motor_angles_);
    motor_anglesSOUT_.setTime(t);
  }
*/

  it = SensorsIn.find("joint-angles");
  if (it!=SensorsIn.end())
  {
    const vector<double>& joint_anglesIn = it->second.getValues();
    joint_angles_.resize (joint_anglesIn.size () );
    for (unsigned i = 0; i < joint_anglesIn.size(); ++i)
      joint_angles_ (i) = joint_anglesIn[i];
    joint_anglesSOUT_.setConstant(joint_angles_);
    joint_anglesSOUT_.setTime(t);
  }

}



void SoTYoyoman01Device::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  map<string,dgsot::SensorValues>::iterator it;
  int t = stateSOUT.getTime () + 1;

  setSensorsIMU(SensorsIn,t);
  setSensorsEncoders(SensorsIn,t);
  sotDEBUGOUT(25);
}

void SoTYoyoman01Device::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}


void SoTYoyoman01Device::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTYoyoman01Device::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void SoTYoyoman01Device::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;
  vector<double> anglesOut;
  anglesOut.resize(state_.size());

  // Integrate control

  
  increment(timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << ((previousState_.size() == state_.size())?
				  (state_ - previousState_) : state_ ) 
		<< std::endl;
  ODEBUG5FULL("state = "<< state_);
  ODEBUG5FULL("diff  = " << ((previousState_.size() == state_.size())?
			     (state_ - previousState_) : state_ ) );
  previousState_ = state_;



  // Specify the joint values for the controller.
  if ((int)anglesOut.size()!=state_.size()-6)
    anglesOut.resize(state_.size()-6);

  for(unsigned int i=6; i < state_.size();++i)
    anglesOut[i-6] = state_(i);
  //controlOut["joints"].setValues(anglesOut);
  controlOut["control"].setValues(anglesOut);  
  
  // Read zmp reference from input signal if plugged
  int time = controlSIN.getTime ();
  zmpSIN.recompute (time + 1);
  // Express ZMP in free flyer reference frame
  dg::Vector zmpGlobal (4);
  for (unsigned int i = 0; i < 3; ++i)
    zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;
  dgsot::MatrixHomogeneous inversePose;
  

  
  inversePose = freeFlyerPose().inverse(Eigen::Affine);
  dg::Vector localZmp(4); localZmp = inversePose.matrix() * zmpGlobal;
  
  vector<double> ZMPRef(3);
  for(unsigned int i=0;i<3;++i)
    ZMPRef[i] = localZmp(i);

  controlOut["zmp"].setName("zmp");
  controlOut["zmp"].setValues(ZMPRef);
  
  // Update position of freeflyer in global frame
  /*
  for (int i = 0;i < 3; ++i)
    baseff_[i*4+3] = freeFlyerPose () (i, 3);
  for(unsigned i = 0;i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      baseff_[i * 4 + j] = freeFlyerPose () (i, j);*/
      
  Eigen::Vector3d transq_(freeFlyerPose().translation());
  dg::sot::VectorQuaternion qt_(freeFlyerPose().linear());

  //translation
  for(int i=0; i<3; i++) baseff_[i] = transq_(i);
  
  //rotation: quaternion
  baseff_[3] = qt_.w();
  baseff_[4] = qt_.x();
  baseff_[5] = qt_.y();
  baseff_[6] = qt_.z();


  controlOut["baseff"].setValues(baseff_);
  ODEBUG5FULL("end"); 
  sotDEBUGOUT(25) ;
}

using namespace dynamicgraph::sot;

namespace dynamicgraph { 
	namespace sot {
#ifdef WIN32
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "c:/tmp/sot-core-traces.txt";
#else	// WIN32
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "/tmp/sot-core-traces.txt";
#endif	// WIN32

#ifdef VP_DEBUG
# ifdef WIN32
  std::ofstream debugfile("C:/tmp/sot-core-traces.txt",
			  std::ios::trunc&std::ios::out);
# else	// WIN32
  std::ofstream debugfile("/tmp/sot-core-traces.txt",
			  std::ios::trunc&std::ios::out);
# endif	// WIN32
#else // VP_DEBUG

  std::ofstream debugfile;


#endif // VP_DEBUG

} /* namespace sot */
} /* namespace dynamicgraph */




