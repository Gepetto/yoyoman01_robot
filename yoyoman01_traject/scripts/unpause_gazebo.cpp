#include <std_srvs/Empty.h>
#include<roscpp>

int main(int argc, char **argv){
  ros::ServiceClient pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty emptySrv;
  pauseGazebo.call(emptySrv);
}
 
