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

//Motors constants
#define TAU 6.2831
#define OFFSET0 0
#define OFFSET1 0
#define ZERO_AX1 502 //Ref is 512
#define ZERO_AX2 502
#define ZERO_XM1 2048 //Ref is 2048
#define ZERO_XM2 2048

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

#include <time.h>//temps dans le fichier texte
#include <stdio.h> 
double posk0 = 0;
int flag_mvtStop = 0;

FILE* fichier = NULL; //fichier texte ecriture capteurs
//******************************************************

///Attention à bien inclure chaque type de message !
#include <std_msgs/Float64.h>

//void quitRequested(int sig) {
//g_quit = true;
//}

//~ namespace barrett_hw
//~ {
//******************************************************
class Yoyoman01Class : public hardware_interface::RobotHW
{
  public:
    int ReadWrite();

    int SpiInit();
    void SPI_check_connection();
    int UpdateImu();
    void UpdateCmd();
    int UpdateSensor();
    int InitMotorsXM();
    int InitMotorsAX();
    int InitMotorsOD();
    void resetcmd();

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
    double cmd[7] ;

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
    //Converting points to radians
    pos[0] = (TAU / 1023) * (ptr_rbuffer->rAx1_pos - 512);
    pos[1] = (TAU / 1023) * (ptr_rbuffer->rAx2_pos - 512);
    pos[2] = (TAU / 4095) * (ptr_rbuffer->rXm1_pos - 2048);
    pos[3] = (TAU / 4095) * (ptr_rbuffer->rXm2_pos - 2048);
    pos[4] = (TAU / 4096) * ptr_rbuffer->rCodHip0;
    pos[5] = (TAU / 4096) * ptr_rbuffer->rCodHip1;


    fichier = fopen("/home/up-board/Documents/catkin_ws/src/yoyoman01_robot/yoyoman01_hw/codeurs.txt", "a");//"a": mode d'ajout.

      if (fichier != NULL)//verification d'erreur
    {
        fprintf(fichier,"%d %d %d %d\n", ptr_rbuffer->rCodHip0, rbuffer.wOd0_pos_fb, ptr_rbuffer->rCodHip1, rbuffer.wOd1_pos_fb);// Tout en points
        fclose(fichier);
    }
        else
    {
        ROS_WARN("Erreur ecriture fichier test.txt");
    }

}
//debug
float ph1,ph2;
float cmd0,cmd1,cmd2,cmd3,cmd4,cmd5;//debug variables
/* get cmd from ROS */
void Yoyoman01Class::UpdateCmd()
{
    //cmd[1] = 80002;
    //cmd[2] = 20508;
    //cmd[3] = (0.2 * sin(ph2));
    //cmd[4] = 0;
    //cmd[4] = (0.1 * sin(ph1));
    //cmd[5] = (0.1 * sin(ph1));
   
// 1er filtrage 
// Au démarrage playmotion renvoie des données erronées qu'il faut éliminer
// Apres qqs secondes les angles deviennent corrects
    if (cmd[0] <= -3.1415 || cmd[0] >= 3.1415) cmd[0] = 0;
    if (cmd[1] <= -3.1415 || cmd[1] >= 3.1415) cmd[1] = 0;
    if (cmd[2] <= -3.1415 || cmd[2] >= 3.1415) cmd[2] = 0;
    if (cmd[3] <= -3.1415 || cmd[3] >= 3.1415) cmd[3] = 0;
    if (cmd[4] <= -3.1415 || cmd[4] >= 3.1415) cmd[4] = 0;
    if (cmd[5] <= -3.1415 || cmd[5] >= 3.1415) cmd[5] = 0;
    if (cmd[6] <= -3.1415 || cmd[6] >= 3.1415) cmd[6] = 0; 


    if (posk0 == cmd[0]) // la cmd n'evolue pas, est differente de 0, mvt pas encore stoppé
    {
        flag_mvtStop = 255;// stopper le mvt
    }

    //    flag_mvtStop = 127;
/*     else  //mvt en action
    {
        flag_encours = 255;
    } 

    if (flag_mvtStop == 255 && flag_encours == 255 )// && flag_mvtStop != 255) // la cmd n'evolue pas, est differente de 0, mvt pas encore stoppé
    {
        resetcmd();
    } */

    posk0 = cmd[0];// sauvegarde la position "-1"


//debug
cmd0=cmd[0];
cmd1=cmd[1];
cmd2=cmd[2];
cmd3=cmd[3];
cmd4=cmd[4];
cmd5=cmd[5];

    /* Convert radians to HW position */
    cAx1_pos = (cmd[0] * (1023 / TAU) + ZERO_AX1);
    cAx2_pos = (cmd[1] * (1023 / TAU) + ZERO_AX2);
    cXm1_pos = (cmd[2] * (4095 / TAU) + ZERO_XM1);
    cXm2_pos = (cmd[3] * (4095 / TAU) + ZERO_XM2);
    cOd0_pos = -( OFFSET0 + cmd[4] * (8192 / TAU) * (48 / 10))*1; //wh*4.8=wm //Positif = sens horaire   
    cOd1_pos = ( OFFSET1 + cmd[5] * (8192 / TAU) * (48 / 10))*1;//Positif = sens anti horaire  

//2nd filtrage
// Les données reçues peuvent être correctes mais en dehors des limites acceptables par les actionneurs 

    /* Check the limits for AX */
        if (cAx1_pos < LIM_INF_AX || cAx1_pos > LIM_SUP_AX || cAx2_pos < LIM_INF_AX || cAx2_pos > LIM_SUP_AX) // for 15°
    {
        ROS_WARN("AX cmd out of range"); 
    }  
    /* Check the limits for XM */
        if (cXm1_pos < LIM_INF_XM || cXm1_pos > LIM_SUP_XM || cXm2_pos < LIM_INF_XM || cXm2_pos > LIM_SUP_XM)
    {
        ROS_WARN("Xm cmd out of range");
    }
    /* Check the limits for ODrive */
        if (cOd0_pos < LIM_INF_OD || cOd0_pos > LIM_SUP_OD || cOd1_pos < LIM_INF_OD || cOd1_pos > LIM_SUP_OD)
    {
        ROS_WARN("ODrive cmd out of range");
    }
    /* Write data */
        else
    {
        ph1 = ph1 + 0.01;
        ph2 = ph2 + 0.007;
        ptr_wbuffer->wAx1_pos = cAx1_pos; //head
        ptr_wbuffer->wAx2_pos = cAx2_pos; //neck
        ptr_wbuffer->wXm1_pos = cXm1_pos;
        ptr_wbuffer->wXm2_pos = cXm2_pos;
        ptr_wbuffer->wOd0_pos = cOd0_pos;
        ptr_wbuffer->wOd1_pos = cOd1_pos;
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
    ROS_INFO("Checking XM limits");
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
    ptr_wbuffer->wXm1_pos = ZERO_XM1;                        //180°
    ptr_wbuffer->wXm2_pos = ZERO_XM2;                        //180°
    ReadWrite();                                         // Send new position
    sleep(1);
    currentFlag = (NO_FLAG); //Reset FLAG
    ROS_INFO("XM ready");
    ROS_INFO("-----------------------------------------");
    sleep(1);
    return true;
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
    ROS_INFO("Checking AX limits");
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
    ptr_wbuffer->wAx1_pos = ZERO_AX1; //~150°
    ptr_wbuffer->wAx2_pos = ZERO_AX2; //~150°
    ReadWrite();                 // Send new position
    sleep(1);
    currentFlag = (NO_FLAG); //Reset FLAG
    ROS_INFO("AX ready");
    ROS_INFO("-----------------------------------------");
    sleep(1);
    return true;
}

int Yoyoman01Class::InitMotorsOD()
{
    /*Alignement des jambes avec le buste 
    Permet d'ajouter un offset entre la position initiale d'apres calibration et la position initiale désirée (alignée avec le buste)
    Attention après un RESET de l'OD la position initiale d'apres calibration sera la dernière position de l'OD 
    l'offset n'est donc pas sytématiquement nécessaire*/
    ROS_WARN("ODrive OFFSETS active");
    ROS_INFO("OFFSETS: OD0 %d | OD1 %d",OFFSET0,OFFSET1);
    ROS_INFO("Setting Zero of ODrive");
    sleep(1);
    currentFlag = (FLAG_WOD);
    ptr_wbuffer->wOd0_pos = OFFSET0/4;
    ptr_wbuffer->wOd1_pos = OFFSET1/4;
    ReadWrite();                 // Send new position
    sleep(2);
    currentFlag = (FLAG_WOD);
    ptr_wbuffer->wOd0_pos = OFFSET0/2;
    ptr_wbuffer->wOd1_pos = OFFSET1/2;
    ReadWrite();                 // Send new position
    sleep(2);
    ptr_wbuffer->wOd0_pos = OFFSET0;
    ptr_wbuffer->wOd1_pos = OFFSET1;
    ReadWrite();                 // Send new position
    sleep(2);
    ROS_INFO("OD ready");
    ROS_INFO("-----------------------------------------");
    sleep(1);
}

void Yoyoman01Class::resetcmd()
{
    cmd[0] = 0;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;
    cmd[6] = 0;
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
        //currentFlag = ( FLAG_WOD | FLAG_WAX | FLAG_WXM | FLAG_CODEURS | FLAG_IMU );
        //currentFlag = (FLAG_WAX  | FLAG_IMU | FLAG_CODEURS);//(loop 5.6ms) 180Hz max
        //currentFlag = (FLAG_WAX);//(loop 5.6ms) 180Hz max
        //currentFlag = (FLAG_ROD);
        currentFlag = (FLAG_WOD | FLAG_CODEURS);
        //currentFlag = (FLAG_RAX | FLAG_RXM | FLAG_IMU | FLAG_CODEURS);//Read only
        //currentFlag = (FLAG_WOD); //(loop 0.75ms) but 100Hz max
        //currentFlag = (NO_FLAG);     // write Flag Explicitly before next sending
        pthread_mutex_unlock(&mutx); // On deverrouille les variables
        /// Pause
        usleep(20000);//50HZ
        //usleep(10000);//100HZ
        //usleep(5000);//200HZ
        //usleep(2000);//500HZ->ok mais trop rapide pour lecture OD
        //usleep(1000);//1000HZ

        //ros::spinOnce(); // will block while a callback is performed
    }
    pthread_exit(EXIT_SUCCESS);
}



void *ROSloop(void *argument)
{
    ROS_INFO("IN thread2 OK");
    //RTloopArgs *aRTloopArgs;
    //aRTloopArgs = (RTloopArgs *)argument;
    ros::Rate loop_rate(30); /// Frequence boucle en Hz
    while (ros::ok)          // NOT real time loop
    {
        //Affichage Reception
        ROS_INFO("----Reception----");
        ROS_INFO(" AX1 %d | AX2 %d | XM1 %d | XM2 %d | OD0 %d | OD1 %d ",rbuffer.rAx1_pos, rbuffer.rAx2_pos, rbuffer.rXm1_pos, rbuffer.rXm2_pos, rbuffer.rOd0_pos, rbuffer.rOd1_pos);
        ROS_INFO("----Codeurs----");
        ROS_INFO(" C0 %d | C1 %d ", rbuffer.rCodHip0, rbuffer.rCodHip1);
        ROS_INFO("----Envoie----");
        ROS_INFO(" AX1 %d | AX2 %d | XM1 %d | XM2 %d | OD0 %d | OD1 %d ", wbuffer.wAx1_pos, wbuffer.wAx2_pos, wbuffer.wXm1_pos, wbuffer.wXm2_pos, wbuffer.wOd0_pos, wbuffer.wOd1_pos); 
        ROS_INFO("----cmd brutes----");
        ROS_INFO(" cmd0 %f | cmd1 %f | cmd2 %f | cmd3 %f | cmd4 %f | cmd5 %f",cmd0,cmd1,cmd2,cmd3,cmd4,cmd5);
        ROS_INFO("----IMU Scaled----");
        ROS_INFO("Received rate X %f | Y %f | Z %f\n", ptr_rate->rXrate_scaled, ptr_rate->rYrate_scaled, ptr_rate->rZrate_scaled);
        ROS_INFO("Received accel X %f | Y %f | Z %f\n", ptr_acc->rXacc_scaled, ptr_acc->rYacc_scaled, ptr_acc->rZacc_scaled);
        ROS_INFO("Received mag  X %f | Y %f | Z %f\n", ptr_mag->rXmag_scaled, ptr_mag->rYmag_scaled, ptr_mag->rZmag_scaled); 
        ROS_INFO("----time loop : %d ", rbuffer.looptime);

 
        if (flag_mvtStop == 255)
        {
            ROS_INFO("mouvement finit");
        }



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
    yoyoman01.InitMotorsXM();// if disabled think to remove FLAG_WXM
    //yoyoman01.InitMotorsAX();// if disabled think to remove FLAG_WAX
    //yoyoman01.InitMotorsOD();

    posk0 =0;
    yoyoman01.resetcmd(); //clean the cmd register

    /*---------------- enregistrement capteurs ------------------ */
    time_t timestamp; 
    struct tm instant; 
    time(&timestamp); 
    instant=*localtime(&timestamp);


    fichier = fopen("/home/up-board/Documents/catkin_ws/src/yoyoman01_robot/yoyoman01_hw/codeurs.txt", "w+");//"w": lecture/ecriture/RAZ

    fprintf(fichier, "Donnees supprimées à chaque démarrage!\n");
    fprintf(fichier,"créé le %d/%d, ", instant.tm_mday+1, instant.tm_mon+1);//date de création
    fprintf(fichier,"à %d:%d:%0d\n", instant.tm_hour, instant.tm_min, instant.tm_sec); 

    fprintf(fichier,"Codeur0 | cmdOD0 | Codeur1 | cmdOD1\n");
    fclose(fichier);
    ROS_INFO("RAZ codeurs.txt");

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
        ROS_ERROR("return code from pthread_create() is %d", error_return);
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
