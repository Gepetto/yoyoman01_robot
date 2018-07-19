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

//*****************SPI**********************
//~ #include "/home/fcaminad/Documents/GIT/catkin_ws/src/yoyoman01_robot/yoyoman01_hw/variables.h"
#include "variables.h"
#include "mraa/spi.h"
int SpiFunction();

#define TAU 6.2831

//SPI declaration
#define SPI_BUS 0
#define MSB_FIRST 0
#define SPI_FREQ 3125000 //3.125Mhz (max 10Mhz)

//volatile sig_atomic_t flag = 1;

//Intermediate variables for cmd
int cAx1_pos, cAx2_pos, cXm1_pos, cXm2_pos, cOd0_pos, cOd1_pos;
int currentFlag;

//Pointer for I/O SPI buffer
struct TrameRead *ptr_rbuffer = &rbuffer;
struct TrameWrite *ptr_wbuffer = &wbuffer;
//Pointer for imu scaled data
struct rate_scaled *ptr_rate = &ratescaled;
struct acc_scaled *ptr_acc = &accscaled;
struct mag_scaled *ptr_mag = &magscaled;

//Threads
void *RTloop(void *argument);
void *ROSloop(void *argument);
pthread_mutex_t mutx = PTHREAD_MUTEX_INITIALIZER; //mutex initialisation
//******************************************************

///Attention à bien inclure chaque type de message !
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
    int InitMotorsXM();
    int InitMotorsAX();

    //~ bool SPI_check_connection();

    Yoyoman01Class() //ros::NodeHandle nh);
    {
        /// connect and register the joint state interface

        hardware_interface::JointStateHandle state_handle_Head("Head", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_Head);

        hardware_interface::JointStateHandle state_handle_Neck("Neck", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_Neck);

        hardware_interface::JointStateHandle state_handle_RArm("Rarm", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_RArm);

        hardware_interface::JointStateHandle state_handle_LArm("Larm", &pos[3], &vel[3], &eff[3]);
        jnt_state_interface.registerHandle(state_handle_LArm);

        hardware_interface::JointStateHandle state_handle_RHip("RHip", &pos[4], &vel[4], &eff[4]);
        jnt_state_interface.registerHandle(state_handle_RHip);

        hardware_interface::JointStateHandle state_handle_LHip("LHip", &pos[5], &vel[5], &eff[5]);
        jnt_state_interface.registerHandle(state_handle_LHip);

        registerInterface(&jnt_state_interface);

        /// connect and register the joint position interface

        hardware_interface::JointHandle pos_handle_Head(jnt_state_interface.getHandle("Head"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_handle_Head);

        hardware_interface::JointHandle pos_handle_Neck(jnt_state_interface.getHandle("Neck"), &cmd[1]);
        jnt_pos_interface.registerHandle(pos_handle_Neck);

        hardware_interface::JointHandle pos_handle_RArm(jnt_state_interface.getHandle("Rarm"), &cmd[2]);
        jnt_pos_interface.registerHandle(pos_handle_RArm);

        hardware_interface::JointHandle pos_handle_LArm(jnt_state_interface.getHandle("Larm"), &cmd[3]);
        jnt_pos_interface.registerHandle(pos_handle_LArm);

        hardware_interface::JointHandle pos_handle_RHip(jnt_state_interface.getHandle("RHip"), &cmd[4]);
        jnt_pos_interface.registerHandle(pos_handle_RHip);

        hardware_interface::JointHandle pos_handle_LHip(jnt_state_interface.getHandle("LHip"), &cmd[5]);
        jnt_pos_interface.registerHandle(pos_handle_LHip);

        registerInterface(&jnt_pos_interface);

        ///IMU
        hardware_interface::ImuSensorHandle IMU_handle("name", "frame_id", &imuregister[0], &imuregister[1], &imuregister[2], &imuregister[3], &imuregister[4], &imuregister[5]);
        imu_state_interface.registerHandle(IMU_handle);

        registerInterface(&imu_state_interface);
    }

    double imuregister[6];
    double cmd[7];

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::ImuSensorInterface imu_state_interface;
    //double cmd[7];
    double pos[7];
    double vel[7];
    double eff[7];
    mraa_spi_context spi;
};

int Yoyoman01Class::ReadWrite()
{
    /* SPI VERIF */
    SPI_check_connection();

    ptr_wbuffer->w_flag = currentFlag; //Update flag before transmit

    /* SPI TRANSFER */
    mraa_spi_transfer_buf(spi, (uint8_t *)ptr_wbuffer, (uint8_t *)ptr_rbuffer, SIZE_BUFFER); //TX,RX,size
}

int Yoyoman01Class::UpdateImu()
{
    /* Mise Ã  l'echelle DATA IMU */
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

/* get position from STM32 */
int Yoyoman01Class::UpdateSensor()
{
    //----temporary initialisation for tests !!!-----------
    ptr_rbuffer->rAx1_pos = 512;
    ptr_rbuffer->rAx2_pos = 512;
    ptr_rbuffer->rXm1_pos = 2048;
    ptr_rbuffer->rXm2_pos = 2048;
    ptr_rbuffer->rCodHip0 = 0;
    ptr_rbuffer->rCodHip1 = 0;
    //--------------------------------------------------
    //Converting points to radians
    pos[0] = (TAU / 1023) * (ptr_rbuffer->rAx1_pos - 512);
    pos[1] = (TAU / 1023) * (ptr_rbuffer->rAx2_pos - 512);
    pos[2] = (TAU / 4095) * (ptr_rbuffer->rXm1_pos - 2048);
    pos[3] = (TAU / 4095) * (ptr_rbuffer->rXm2_pos - 2048);
    pos[4] = -(TAU / 2048) * ptr_rbuffer->rCodHip0;
    pos[5] = (TAU / 2048) * ptr_rbuffer->rCodHip1;
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
    ptr_wbuffer->wspi_test = SESAME; //number sent

    if (rbuffer.rspi_test != SESAME) //Corrupted data
    {
        ROS_INFO("Connecting to STM32..."); // Au premier démarrage un reset du STm32 est parfois necessaire
        ptr_wbuffer->w_flag = (NO_FLAG);    // All desactivated

        while (rbuffer.rspi_test != SESAME)
        {
            mraa_spi_transfer_buf(spi, (uint8_t *)ptr_wbuffer, (uint8_t *)ptr_rbuffer, SIZE_BUFFER); //S/R until goods values
            ROS_ERROR("SPI connection FAULT *Maybe Reset stm32* %d ", rbuffer.rspi_test);            // Au premier démarrage un reset du STm32 est parfois necessaire
            usleep(100000);                                                                          //wait 0.1s
        }
        ROS_INFO("SPI Ready");
        ROS_INFO("-----------------------------------------");
    }
    else //OK Sending == receiving
    {
        //ROS_INFO("SPI connection OK");
        ptr_rbuffer->rspi_test = 0; //reset reception
    }
}
float ph1 = 0.0;
//float ph2 = 0;

float cm3, cm4, cm5, cm6;

/* get cmd from ROS */
int Yoyoman01Class::UpdateCmd()
{

    //cmd[1] = 802;
    //cmd[2] = 2058;
    //cmd[3] = 0; //(0.2 * sin(ph2));
    //cmd[4] = 0;
    //cmd[4] = (0.3 * sin(ph1));
    //cmd[5] = (0.3 * sin(ph1));

    cm3 = cmd[2];
    cm4 = cmd[3];
    cm5 = cmd[4];
    cm6 = cmd[5];

    /* Convert radians to HW position */
    cAx1_pos = (cmd[0] * (1023 / TAU) + 512);
    cAx2_pos = (cmd[1] * (1023 / TAU) + 512);
    cXm1_pos = (cmd[2] * (4095 / TAU) + 2048);
    cXm2_pos = (cmd[3] * (4095 / TAU) + 2048);
    cOd0_pos = -(cmd[4] * (8192 / TAU) * (48 / 10)); //wh*4.8=wm
    cOd1_pos = (cmd[5] * (8192 / TAU) * (48 / 10));

    /* Check the limits for AX */
    if (cAx1_pos < LIM_INF_AX || cAx1_pos > LIM_SUP_AX || cAx2_pos < LIM_INF_AX || cAx2_pos > LIM_SUP_AX) // for 15°
    {
        ROS_WARN("AX cmd out of range");
        // return false;
    }
    /* Check the limits for XM */
    if (cXm1_pos < LIM_INF_XM || cXm1_pos > LIM_SUP_XM || cXm2_pos < LIM_INF_XM || cXm2_pos > LIM_SUP_XM)
    {
        ROS_WARN("Xm cmd out of range");
        return false;
    }
    /* Check the limits for ODrive */
    if (cOd0_pos < LIM_INF_OD || cOd0_pos > LIM_SUP_OD || cOd1_pos < LIM_INF_OD || cOd1_pos > LIM_SUP_OD)
    {
        ROS_WARN("ODrive cmd out of range");
        return false;
    }
    /* Write data */
    else
    {
        ph1 = ph1 + 0.05;
        //ph2 = ph2 + 0.007;
        ptr_wbuffer->wAx1_pos = cAx1_pos; //head
        ptr_wbuffer->wAx2_pos = cAx2_pos; //neck
        ptr_wbuffer->wXm1_pos = cXm1_pos;
        ptr_wbuffer->wXm2_pos = cXm2_pos;
        ptr_wbuffer->wOd0_pos = cOd0_pos;
        ptr_wbuffer->wOd1_pos = cOd1_pos;
        return true;
    }
}

int Yoyoman01Class::InitMotorsXM()
{
    /*Sequence inititialisation moteurs 
    Les dynamixels ne font pas de tours complets, il faut les initialiser en position médiane.
    Il y a risque de casse si le moteur cherche à atteindre une position en faisant un tour complet.
    - Vérifier que la postion du dynamixel est comprise entre les bornes limites 
    - Si la condition est vérifiée alors la Position initiale est envoyée (2048 pour XM et 512 pour AX) */

    currentFlag = (FLAG_RXM); // Read only -> /!\ Include a delay /!\ 
    ReadWrite();
    usleep(50000);

    /* Check the limits for XM */
    ROS_INFO("Check XM limits");
    while (rbuffer.rXm1_pos < LIM_INF_XM || rbuffer.rXm1_pos > LIM_SUP_XM || rbuffer.rXm2_pos < LIM_INF_XM || rbuffer.rXm2_pos > LIM_SUP_XM) // Wait until correct limits
    {
        ReadWrite();
        ROS_WARN("Impossible to initialize Xm : position out of range");
        ROS_WARN("Manually go to position between %d and %d", LIM_INF_XM, LIM_SUP_XM);
        sleep(1);
        ROS_INFO(" XM1 %d | XM2 %d ", rbuffer.rXm1_pos, rbuffer.rXm2_pos); //Current position
    }
    ROS_INFO("Limits : OK");
    sleep(1);
    ROS_INFO("Sending commands for arms...");
    sleep(1);
    currentFlag = (FLAG_RXM | FLAG_TORQUEXM | FLAG_WXM); //Lecture,ecriture,autorisation mouvement
    ptr_wbuffer->wXm1_pos = 2048;                        //180°
    ptr_wbuffer->wXm2_pos = 2048;                        //180°
    ReadWrite();                                         // Send new position
    sleep(1);
    currentFlag = (NO_FLAG); //Reset FLAG
    ROS_INFO("XM ready");
    ROS_INFO("-----------------------------------------");
    sleep(1);
}

int Yoyoman01Class::InitMotorsAX()
{
    /*Sequence inititialisation moteurs 
    Les dynamixels ne font pas de tours complets, il faut les initialiser en position médiane.
    Il y a risque de casse si le moteur cherche à atteindre une position en faisant un tour complet.
    - Vérifier que la postion du dynamixel est comprise entre les bornes limites 
    - Si la condition est vérifiée alors la Position initiale est envoyée (2048 pour XM et 512 pour AX) */

    currentFlag = (FLAG_RAX); //Lecture seule
    ReadWrite();
    usleep(50000);

    /* Check the limits for XM */
    ROS_INFO("Check AX limits");
    while (rbuffer.rAx1_pos < LIM_INF_AX || rbuffer.rAx1_pos > LIM_SUP_AX || rbuffer.rAx2_pos < LIM_INF_AX || rbuffer.rAx2_pos > LIM_SUP_AX) // for 15° // Wait until correct limits
    {
        ReadWrite();
        ROS_WARN("Impossible to initialize AX : position out of range");
        ROS_WARN("Manually go to position between %d and %d", LIM_INF_AX, LIM_SUP_AX);
        sleep(1);
        ROS_INFO(" AX1 %d | AX2 %d ", rbuffer.rAx1_pos, rbuffer.rAx2_pos); //Current position
    }
    ROS_INFO("Limits : OK");
    usleep(50000);
    ROS_INFO("Sending commands for head and neck...");
    usleep(50000);
    currentFlag = (FLAG_WAX);    //ecriture
    ptr_wbuffer->wAx1_pos = 512; //150°
    ptr_wbuffer->wAx2_pos = 512; //150°
    ReadWrite();                 // Send new position
    sleep(1);
    currentFlag = (NO_FLAG); //Reset FLAG
    ROS_INFO("AX ready");
    ROS_INFO("-----------------------------------------");
    sleep(1);
}

typedef struct arg_struct
{
    controller_manager::ControllerManager *cm;
    Yoyoman01Class *yoyoman01;
} RTloopArgs;

void *RTloop(void *argument)
{
    ROS_INFO("IN thread1 OK");
    RTloopArgs *aRTloopArgs;
    aRTloopArgs = (RTloopArgs *)argument;

    ros::Time last_time = ros::Time::now();

    aRTloopArgs->yoyoman01->SpiInit(); //Initialisation matérielle
    while (ros::ok)                    /// Boucle tant que le master existe (ros::ok())
    {
        pthread_mutex_lock(&mutx); // On verrouille les variables pour ce thread

        // Read phase
        aRTloopArgs->yoyoman01->ReadWrite();    //Receiving data only one time
        aRTloopArgs->yoyoman01->UpdateImu();    //Processing incoming data
        aRTloopArgs->yoyoman01->UpdateSensor(); //Processing incoming data

        // Call Controllers
        ros::Duration duration = ros::Time::now() - last_time;
        aRTloopArgs->cm->update(ros::Time::now(), duration);
        last_time = ros::Time::now();

        /// Write Phase
        aRTloopArgs->yoyoman01->UpdateCmd();

        //currentFlag = (FLAG_RAX | FLAG_WAX | FLAG_RXM | FLAG_WXM | FLAG_WOD | FLAG_IMU | FLAG_CODEURS);//(loop 7ms) 250Hz max
        //currentFlag = (FLAG_WAX | FLAG_WXM | FLAG_WOD | FLAG_IMU | FLAG_CODEURS);//(loop 5.6ms) 180Hz max
        currentFlag = ( FLAG_WAX | FLAG_WOD | FLAG_WXM | FLAG_IMU | FLAG_CODEURS);
        //currentFlag = (FLAG_RAX | FLAG_RXM | FLAG_IMU | FLAG_CODEURS);//Read only
        //currentFlag = (FLAG_WOD); //(loop 0.75ms) but 100Hz max
        //currentFlag = (NO_FLAG);     // write Flag Explicitly before next sending
        pthread_mutex_unlock(&mutx); // On deverrouille les variables
        /// Pause
        usleep(10000); //100HZ
        //usleep(100000); //10HZ
        //usleep(20000); //50HZ ->ok
        //usleep(5000); //200HZ
        //usleep(2000); //500HZ
        //usleep(3000);
        //usleep(1000);

        //ros::spinOnce(); // will block while a callback is performed
    }
    pthread_exit(EXIT_SUCCESS);
}

void *ROSloop(void *argument)
{
    ROS_INFO("IN thread2 OK");
    RTloopArgs *aRTloopArgs;
    aRTloopArgs = (RTloopArgs *)argument;
    ros::Rate loop_rate(10); /// Frequence boucle en Hz
    while (ros::ok)          // NOT real time loop
    {
        //Affichage Reception
        ROS_INFO("----Reception----");
        ROS_INFO(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d", rbuffer.rOd0_pos, rbuffer.rOd1_pos, rbuffer.rAx1_pos, rbuffer.rAx2_pos, rbuffer.rXm1_pos, rbuffer.rXm2_pos);
        ROS_INFO("----Codeurs----");
        ROS_INFO(" C0 %d | C1 %d ", rbuffer.rCodHip0, rbuffer.rCodHip1);
        ROS_INFO("----Envoie----");
        ROS_INFO(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d", wbuffer.wOd0_pos, wbuffer.wOd1_pos, wbuffer.wAx1_pos, wbuffer.wAx2_pos, wbuffer.wXm1_pos, wbuffer.wXm2_pos);
        //ROS_INFO("----cmd----");
        //ROS_INFO(" cmd3 %f | cmd4 %f | cmd5 %f| cmd6 %f",cm3, cm4,cm5,cm6);
        /*         ROS_INFO("----IMU Scaled----");
        ROS_INFO("Received rate X %f | Y %f | Z %f\n", ptr_rate->rXrate_scaled, ptr_rate->rYrate_scaled, ptr_rate->rZrate_scaled);
        ROS_INFO("Received accel X %f | Y %f | Z %f\n", ptr_acc->rXacc_scaled, ptr_acc->rYacc_scaled, ptr_acc->rZacc_scaled);
        ROS_INFO("Received mag  X %f | Y %f | Z %f\n", ptr_mag->rXmag_scaled, ptr_mag->rYmag_scaled, ptr_mag->rZmag_scaled); */

        /* ROS_WARN("Entrer cmd ODrive [0;8000] : ");
        int od;
        scanf("%d", &od);
        wbuffer.wOd0_pos = od;
        wbuffer.wOd1_pos = od;*/

        /*         ROS_WARN("Entrer cmd XM [0;4095]: ");
        int test;and addi
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
    /* Initialisation du node : le troisieme argument est son nom */
    ros::init(argc, argv, "hw_node");

    /* Connexion au master et initialisation du NodeHandle qui permet d avoir acces aux topics et services */
    ros::NodeHandle nh; //yoyoman01_nh;
    controller_manager::ControllerManager cm(&yoyoman01);
    RTloopArgs aRTloopArgs;
    aRTloopArgs.cm = &cm;
    aRTloopArgs.yoyoman01 = &yoyoman01;

    /*---------------- Validation fonctionnement SPI ------------------ */
    currentFlag = NO_FLAG;            // All disabled
    yoyoman01.SpiInit();              //Initialisation SPI
    yoyoman01.SPI_check_connection(); //Verifivation envoie/reception

    /*---------------- Initialisation moteurs ------------------ */
    yoyoman01.InitMotorsXM();
    yoyoman01.InitMotorsAX();
    ROS_INFO("Starting in 5s");
    ROS_INFO("-------------------------------- 5");
    sleep(1);
    ROS_INFO("------------------------- 4");
    sleep(1);
    ROS_INFO("----------------- 3");
    sleep(1);
    ROS_INFO("--------- 2");
    sleep(1);
    ROS_INFO("--- 1");
    sleep(1);

    /*---------------- Gestion des threads ------------------ */
    /* 
    Thread "number 1" : Main  
    Thread "number 2" : Rosloop
    Thread "number 3" : RTloop
    Thread "number 4" : spinner
    */
    pthread_t thread1;
    pthread_t thread2;
    int error_return;

    struct sched_param params1;
    params1.sched_priority = 90;                                                 // 1(low) to 99(high)
    error_return = pthread_setschedparam(thread1, SCHED_RR, &params1);           // function sets the scheduling policy and parameters of the thread
    error_return = pthread_create(&thread1, NULL, RTloop, (void *)&aRTloopArgs); // create a new thread
    ROS_INFO("thread1 created OK");
    error_return = pthread_create(&thread2, NULL, ROSloop, (void *)&aRTloopArgs); // create a new thread
    ROS_INFO("thread2 created OK");

    if (error_return)
    {
        ROS_ERROR("return code from pthread_create() is %d\n", error_return);
        exit(-1);
    }

    /* Spinners */
    ros::MultiThreadedSpinner spinner(4); //unspecified (or set to 0), it will use a thread for each CPU core
    spinner.spin();                       // spin() will not return until the node has been shutdown

    /* depreciated because disturbs the ctrl-c exit */
    //pthread_join(thread1, NULL); // Wait the end of the thread before exit
    //pthread_join(thread2, NULL); // Wait the end of the thread before exit

    return 0;
}