
/*Sources : 
* https://github.com/jhu-lcsr-attic/barrett_control/blob/libbarrett/barrett_hw/src/wam_server.cpp
* http://www.ensta-bretagne.fr/lemezo/files/teaching/ROS/TP1.pdf
* http://homepages.laas.fr/ostasse/Teaching/ROS/rosintro.pdf
* https://github.com/ros-controls/ros_control/wiki/hardware_interface
# To compile :g++ ros_hardware.cpp -o ros_harware -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
*/

#include <ros/ros.h>
//~ #include <native/task.h>
//~ #include <sys/mman.h>
//~ #include <cmath>
//~ #include <time.h>
#include <controller_manager/controller_manager.h>
//~ #include <signal.h>
//~ #include <realtime_tools/realtime_publisher.h>
//~ #include <control_toolbox/filters.h>
//~ #include <control_toolbox/pid.h>
//~ #include <std_msgs/Duration.h>

#include <iostream>
#include <sstream>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//**************************THREADS****************************
#include <pthread.h>
#define NUM_THREADS 2
//**************************SPI****************************
//~ #include "/home/fcaminad/Documents/GIT/catkin_ws/src/yoyoman01_robot/yoyoman01_hw/variables.h"
#include "variables.h"
#include "mraa/spi.h"

/* SPI declaration */
#define SPI_BUS 0
#define MSB_FIRST 0
#define SPI_FREQ 3125000 //3.125Mhz (max 10Mhz)

//volatile sig_atomic_t flag = 1;

//Pointer for I/O SPI buffer
struct TrameRead *ptr_rbuffer = &rbuffer;
struct TrameWrite *ptr_wbuffer = &wbuffer;
//Pointer for imu scaled data
struct rate_scaled *ptr_rate = &ratescaled;
struct acc_scaled *ptr_acc = &accscaled;
struct mag_scaled *ptr_mag = &magscaled;

//******************************************************

///Attention Ã  bien inclure chaque type de message !
#include <std_msgs/Float64.h>

//void quitRequested(int sig) {
//g_quit = true;
//}

//~ namespace barrett_hw
//~ {
class Yoyoman01Class : public hardware_interface::RobotHW
{
  public:
    int ReadWrite();
    int SpiInit();
    void SPI_check_connection();
    int UpdateImu();
    int UpdateCmd();
    int UpdateSensor();

    //~ bool SPI_check_connection();

    Yoyoman01Class() //ros::NodeHandle nh);
    {
        /// connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_base("base_link", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_base);

        hardware_interface::JointStateHandle state_handle_Head("HeadLink", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_Head);

        hardware_interface::JointStateHandle state_handle_Neck("NeckLink", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_Neck);

        hardware_interface::JointStateHandle state_handle_RArm("RArmLink", &pos[3], &vel[3], &eff[3]);
        jnt_state_interface.registerHandle(state_handle_RArm);

        hardware_interface::JointStateHandle state_handle_LArm("LArmLink", &pos[4], &vel[4], &eff[4]);
        jnt_state_interface.registerHandle(state_handle_LArm);

        hardware_interface::JointStateHandle state_handle_RHip("RHipLink", &pos[5], &vel[5], &eff[5]);
        jnt_state_interface.registerHandle(state_handle_RHip);

        hardware_interface::JointStateHandle state_handle_LHip("LHipLink", &pos[6], &vel[6], &eff[6]);
        jnt_state_interface.registerHandle(state_handle_LHip);

        registerInterface(&jnt_state_interface);

        /// connect and register the joint position interface
        hardware_interface::JointHandle pos_handle_base(jnt_state_interface.getHandle("base_link"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_handle_base);

        hardware_interface::JointHandle pos_handle_Head(jnt_state_interface.getHandle("HeadLink"), &cmd[1]);
        jnt_pos_interface.registerHandle(pos_handle_Head);

        hardware_interface::JointHandle pos_handle_Neck(jnt_state_interface.getHandle("NeckLink"), &cmd[2]);
        jnt_pos_interface.registerHandle(pos_handle_Neck);

        hardware_interface::JointHandle pos_handle_RArm(jnt_state_interface.getHandle("RArmLink"), &cmd[3]);
        jnt_pos_interface.registerHandle(pos_handle_RArm);

        hardware_interface::JointHandle pos_handle_LArm(jnt_state_interface.getHandle("LArmLink"), &cmd[4]);
        jnt_pos_interface.registerHandle(pos_handle_LArm);

        hardware_interface::JointHandle pos_handle_RHip(jnt_state_interface.getHandle("RHipLink"), &cmd[5]);
        jnt_pos_interface.registerHandle(pos_handle_RHip);

        hardware_interface::JointHandle pos_handle_LHip(jnt_state_interface.getHandle("LHipLink"), &cmd[6]);
        jnt_pos_interface.registerHandle(pos_handle_LHip);

        registerInterface(&jnt_pos_interface);

        ///IMU
        hardware_interface::ImuSensorHandle IMU_handle("name", "frame_id", &imuregister[0], &imuregister[1], &imuregister[2], &imuregister[3], &imuregister[4], &imuregister[5]);
        imu_state_interface.registerHandle(IMU_handle);

        registerInterface(&imu_state_interface);
    }

    double imuregister[6];

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::ImuSensorInterface imu_state_interface;
    double cmd[7];
    double pos[7];
    double vel[7];
    double eff[7];
    mraa_spi_context spi;
};

int Yoyoman01Class::ReadWrite()
{
    /* SPI TRANSFER */
    mraa_spi_transfer_buf(spi, (uint8_t *)ptr_wbuffer, (uint8_t *)ptr_rbuffer, SIZE_BUFFER); //TX,RX,size
    /* SPI VERIF */
    SPI_check_connection();
}

int Yoyoman01Class::UpdateImu()
{
    /* Mise Ãƒ  l'echelle DATA IMU */
    ptr_rate->rXrate_scaled = (float)rbuffer.rate[0] / FACTOR_RATE;
    ptr_rate->rYrate_scaled = (float)rbuffer.rate[1] / FACTOR_RATE;
    ptr_rate->rZrate_scaled = (float)rbuffer.rate[2] / FACTOR_RATE;
    ptr_acc->rXacc_scaled = (float)rbuffer.acc[0] / FACTOR_ACC;
    ptr_acc->rYacc_scaled = (float)rbuffer.acc[1] / FACTOR_ACC;
    ptr_acc->rZacc_scaled = (float)rbuffer.acc[2] / FACTOR_ACC;
    ptr_mag->rXmag_scaled = (float)rbuffer.mag[0] / FACTOR_MAG;
    ptr_mag->rYmag_scaled = (float)rbuffer.mag[1] / FACTOR_MAG;
    ptr_mag->rZmag_scaled = (float)rbuffer.mag[2] / FACTOR_MAG;

    hardware_interface::ImuSensorHandle::Data ImuData;
    ImuData.angular_velocity = (double *)ptr_rate;
    ImuData.linear_acceleration = (double *)ptr_acc;
}
float ph = 0;
int cAx1_pos, cAx2_pos, cXm1_pos, cXm2_pos, cOd0_pos, cOd1_pos;

/* get cmd from ROS */
int Yoyoman01Class::UpdateCmd()
{
    cmd [5]= (0.6 * sin(ph));
    cmd [6]= (0.6 * sin(ph));
    /* Convert radians to HW position */
    cAx1_pos = cmd[1] * (1023 / (2*3.1415));
    cAx2_pos = cmd[2] * (1023 / (2*3.1415));
    cXm1_pos = cmd[3] * (4095 / (2*3.1415));
    cXm2_pos = cmd[4] * (4095 / (2*3.1415)); 
    cOd0_pos = cmd[5] * (8192 / (2*3.1415))*(48/10);//wh*4.8=wm
    cOd1_pos = cmd[6] * (8192 / (2*3.1415))*(48/10);

    cAx1_pos = 512;
    cAx2_pos = 512;
    cXm1_pos = 2048;
    cXm2_pos = 2048;
    cOd0_pos = 0;//(8192 * (0.2 * cos(ph)));
    cOd1_pos = 0;

    /* Check the limits for AX */
    if (cAx1_pos < 500 || cAx1_pos > 522 || cAx2_pos < 500 || cAx2_pos > 522)
    {
        ROS_WARN("AX cmd out of range");
        return false;
    }
    /* Check the limits for XM */
    if (cXm1_pos < 1948 || cXm1_pos > 2148 || cXm2_pos < 1948 || cXm2_pos > 2148)
    {
        ROS_WARN("Xm cmd out of range");
        return false;
    }
    /* Check the limits for ODrive */
    if (cOd0_pos < -4000 || cOd0_pos > 4000 || cOd1_pos < -4000 || cOd1_pos > 4000)
    {
        ROS_WARN("ODrive cmd out of range");
        return false;
    }
    /* Write data */
    else
    {
        ph = ph + 0.005;
        ptr_wbuffer->wAx1_pos = cAx1_pos; //head
        ptr_wbuffer->wAx2_pos = cAx2_pos; //neck
        ptr_wbuffer->wXm1_pos = cXm1_pos; 
        ptr_wbuffer->wXm2_pos = cXm2_pos;
        ptr_wbuffer->wOd0_pos = cOd0_pos;
        ptr_wbuffer->wOd1_pos = cOd1_pos;
        return true;
    }
}
/* get position from STM32 */
int Yoyoman01Class::UpdateSensor()
{
    pos[1] = ptr_rbuffer->rAx1_pos;
    pos[2] = ptr_rbuffer->rAx2_pos;
    pos[3] = ptr_rbuffer->rXm1_pos;
    pos[4] = ptr_rbuffer->rXm2_pos;
    pos[5] = ptr_rbuffer->rOd0_pos;
    pos[6] = ptr_rbuffer->rOd1_pos;
}

int Yoyoman01Class::SpiInit()
{
    //////////SPI INITIALISATION///////////////////////////////////////////
    /* initialize mraa for the platform (not needed most of the times) */
    mraa_init();

    mraa_result_t status = MRAA_SUCCESS;

    /* initialize SPI bus */
    spi = mraa_spi_init(SPI_BUS);

    if (spi == NULL)
    {
        fprintf(stderr, "Failed to initialize SPI\n");
        mraa_deinit();
        return EXIT_FAILURE;
    }
    /* set SPI frequency */
    status = mraa_spi_frequency(spi, SPI_FREQ);
    if (status != MRAA_SUCCESS)
    {
        goto err_exit;
    }
    /* set spi mode */
    status = mraa_spi_mode(spi, MRAA_SPI_MODE0); //or (int) 0
    if (status != MRAA_SUCCESS)
    {
        goto err_exit;
    }
    /* set big endian mode */
    status = mraa_spi_lsbmode(spi, MSB_FIRST);
    if (status != MRAA_SUCCESS)
    {
        goto err_exit;
    }
    /* MAX7219/21 chip needs the data in word size */
    status = mraa_spi_bit_per_word(spi, 8);
    if (status != MRAA_SUCCESS)
    {
        fprintf(stdout, "Failed to set SPI Device to 8Bit mode\n");
        goto err_exit;
    }
    else
    {
        return 0;
    }

err_exit:
    mraa_result_print(status);
    /* stop spi */
    mraa_spi_stop(spi);
    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();
    return EXIT_FAILURE;
}

void Yoyoman01Class::SPI_check_connection()
{
    ptr_wbuffer->wspi_test = 36055; //number sent

    if (wbuffer.wspi_test != rbuffer.rspi_test)
    {
        while (wbuffer.wspi_test != rbuffer.rspi_test)
        {
            mraa_spi_transfer_buf(spi, (uint8_t *)ptr_wbuffer, (uint8_t *)ptr_rbuffer, SIZE_BUFFER); //S/R until goods values
            ROS_ERROR("SPI connection FAULT *Maybe Reset stm32* %d ", rbuffer.rspi_test);            // Au premier démarrage un reset du STm2 est parfois necessaire
            sleep(1);                                                                                //wait 1s
        }
    }
    else
    {
        //ROS_INFO("SPI connection OK");
        ptr_rbuffer->rspi_test = 0; //reset reception
    }
}

struct arg_struct
{
    bool rosOk;
} args;

pthread_mutex_t mutx = PTHREAD_MUTEX_INITIALIZER; //mutex initialisation 

void *RTloop(void *argument);
void *ROSloop(void *argument);

void *RTloop(void *argument)
{
    Yoyoman01Class yoyoman01;
    yoyoman01.SpiInit(); //Initialisation matérielle

    while (args.rosOk) /// Boucle tant que le master existe (ros::ok())
    {
        //ROS_INFO("----while hwloop----\n");
        pthread_mutex_lock(&mutx); /* On verrouille les variables pour ce thread */
        yoyoman01.ReadWrite();
        yoyoman01.UpdateImu();
        yoyoman01.UpdateCmd();
        yoyoman01.UpdateSensor();
        pthread_mutex_unlock(&mutx); /* On deverrouille les variables */
        /// Pause
        usleep(10000); //100HZ
        //usleep(100000); //10HZ
        //ros::spinOnce(); // will block while a callback is performed
    }
    pthread_exit(EXIT_SUCCESS);
}

void *ROSloop(void *argument)
{
    ros::Rate loop_rate(1); /// Frequence boucle en Hz

    while (args.rosOk) // NOT real time loop
    {
        //Affichage Reception
        ROS_INFO("----Reception Position----");
        ROS_INFO(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d \n", rbuffer.rOd0_pos, rbuffer.rOd1_pos, rbuffer.rAx1_pos, rbuffer.rAx2_pos, rbuffer.rXm1_pos, rbuffer.rXm2_pos);
        ROS_INFO("----Envoie----\n");
        ROS_INFO(" OD0 %d | OD1 %d | XM1 %d | XM2 %d \n", ptr_wbuffer->wOd0_pos,ptr_wbuffer->wOd1_pos,ptr_wbuffer->wXm1_pos, ptr_wbuffer->wXm2_pos);
        ROS_INFO("----IMU Scaled----\n");
        ROS_INFO("Received rate X %f | Y %f | Z %f\n", ptr_rate->rXrate_scaled, ptr_rate->rYrate_scaled, ptr_rate->rZrate_scaled);
        ROS_INFO("Received accel X %f | Y %f | Z %f\n", ptr_acc->rXacc_scaled, ptr_acc->rYacc_scaled, ptr_acc->rZacc_scaled);
        ROS_INFO("Received mag  X %f | Y %f | Z %f\n", ptr_mag->rXmag_scaled, ptr_mag->rYmag_scaled, ptr_mag->rZmag_scaled);

        /* ROS_WARN("Entrer cmd ODrive [0;8000] : ");
        int od;
        scanf("%d", &od);
        wbuffer.wOd0_pos = od;
        wbuffer.wOd1_pos = od;*/

        /*         ROS_WARN("Entrer cmd XM [0;4095]: ");
        int test;
        scanf("%d", &test);
        wbuffer.wXm1_pos = test;
        wbuffer.wXm2_pos = test;  */

        loop_rate.sleep();
    }
    pthread_exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
    Yoyoman01Class yoyoman01;

    /*---------------- ROS Stuff ------------------ */
    //controller_manager::ControllerManager cm(&yoyoman01);

    /* Initialisation du node : le troisieme argument est son nom */
    ros::init(argc, argv, "new_node");

    /* Connexion au master et initialisation du NodeHandle qui permet d avoir acces aux topics et services */
    ros::NodeHandle yoyoman01_nh;

    /*---------------- Validation fonctionnement SPI ------------------ */
    yoyoman01.SpiInit();              //Initialisation matérielle
    yoyoman01.SPI_check_connection(); //Verifivation envoie/reception

    /*---------------- Gestion des threads ------------------ */
    /* 
    Thread "number 1" : Main  
    Thread "number 2" : Rosloop
    Thread "number 3" : RTloop
    Thread "number 4" : spinner
    */
   ROS_INFO("Creation thread");
    args.rosOk = (ros::ok);

    pthread_t thread1;
    pthread_t thread2;
    int error_return;

    struct sched_param params1;
    params1.sched_priority = 90; // 1(low) to 99(high)
    error_return = pthread_setschedparam(thread1, SCHED_RR, &params1); // function sets the scheduling policy and parameters of the thread
    error_return = pthread_create(&thread1, NULL, RTloop, &args);  // create a new thread
    error_return = pthread_create(&thread2, NULL, ROSloop, &args); // create a new thread

    if (error_return)
    {
        ROS_ERROR("return code from pthread_create() is %d\n", error_return);
        exit(-1);
    }

    /* Spinners */
    ros::MultiThreadedSpinner spinner(1); //unspecified (or set to 0), it will use a thread for each CPU core
    spinner.spin();                       // spin() will not return until the node has been shutdown

    /*CrÃ©ation du publisher avec
    le type du message
    le nom du topic
    la taille du buffer de message Ã  conserver en cas de surchage */
    //ros::Publisher  yoyoman01_pub = yoyoman01_nh.advertise<std_msgs::Float64>("/yoyoman01/Head_position_controller/command", 10);

    pthread_join(thread1, NULL); // Wait the end of the thread before exit
    pthread_join(thread2, NULL); // Wait the end of the thread before exit
    /* Last thing that main() should do */
    return 0;
}

/* //~ bool Yoyoman01Class::SPI_check_connection()//const ros::Time time, const ros::Duration period)
//~ {
//~ mraa_spi_transfer_buf(spi,(uint8_t*)ptr_wbuffer ,(uint8_t*)ptr_rbuffer,SIZE_BUFFER);   //TX,RX,size
//~ return true;
//~ }
//~ bool Yoyoman01Class::read()//const ros::Time time, const ros::Duration period)
//~ {
// Iterate over all devices
//~ for(Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); ++it) {
//~ this->read_wam(time, period, it->second);
//~ }
//~ for(Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); ++it) {
//~ this->read_wam(time, period, it->second);
//~ }
//~ return true;
//~ }
//~ void Yoyoman01Class::write()//const ros::Time time, const ros::Duration period)
//~ {
// Iterate over all devices
//~ for(Wam4Map::iterator it = wam4s_.begin(); it != wam4s_.end(); ++it) {
//~ this->write_wam(time, period, it->second);
//~ }
//~ for(Wam7Map::iterator it = wam7s_.begin(); it != wam7s_.end(); ++it) {
//~ this->write_wam(time, period, it->second);
//~ }
//~ }
//~ template <size_t DOF>
//~ bool Yoyoman01Class::read_wam()//const ros::Time time, const ros::Duration period,boost::shared_ptr<Yoyoman01Class::WamDevice<DOF> > device)
//~ {
//~ // Poll the hardware
//~ try {
//~ device->interface->update();
//~ } catch (const std::runtime_error& e) {
//~ if (device->interface->getSafetyModule() != NULL  &&
//~ device->interface->getSafetyModule()->getMode(true) == barrett::SafetyModule::ESTOP)
//~ {
//~ ROS_ERROR_STREAM("systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot communicate with Pucks.");
//~ return false;
//~ } else {
//~ throw;
//~ }
//~ } */

//-----------------------------------
//--------PARTIE INSPIREE SOT POUR IMU ---------
/*
namespace sot_controller{
    namespace lhi = hardware_interface;
    namespace lci = controller_interface;
  
  class RCSotController : public lci::ControllerBase {
    
  protected:
    /// Robot nb dofs.
    size_t nbDofs_;
    /// Data log.
    rc_sot_system::DataToLog DataOneIter_;
    /// Initialize the hardware interface accessing the IMU.
    bool initIMU();
    /// In the map sensorsIn_ creates the key "name_IMUNb"
    /// and associate to this key the vector data.
    void setSensorsImu(std::string &name,int IMUNb,std::vector<double> &data);
    /// Read the imus and set the interface to the SoT.
    void fillImu();
        /// Initialize the roscontrol interfaces
    bool initInterfaces(lhi::RobotHW * yoyoman01,
			ros::NodeHandle &,
			ros::NodeHandle &,
    ClaimedResources & claimed_resources);
  private:
    /// \brief Vector towards the IMU.
    std::vector<lhi::ImuSensorHandle> imu_sensor_;
    /// \brief Interface to the sensors (IMU).
    lhi::ImuSensorInterface* imu_iface_;
    
  };
  
  bool RCSotController::
  initInterfaces(lhi::RobotHW * yoyoman01,
		 ros::NodeHandle &,
		 ros::NodeHandle &,
		 ClaimedResources & claimed_resources)
  {
    std::string lns;
    lns="hardware_interface";
    // Get a pointer to the IMU sensor interface
    imu_iface_ = yoyoman01->get<ImuSensorInterface>();
    if (! imu_iface_)
      {
	ROS_ERROR("This controller requires a hardware interface of type '%s'."
		  " Make sure this is registered in the %s::RobotHW class.",
		  internal :: demangledTypeName<ImuSensorInterface>().c_str(),lns.c_str());
	return false ;
      }
    if (! init ())
      {
	ROS_ERROR("Failed to initialize sot-controller" );
	std :: cerr << "FAILED LOADING SOT CONTROLLER" << std::endl;
	return false ;
      }
    
    return true;
}
}
//-----------------------------------
//-----------------------------------
*/
