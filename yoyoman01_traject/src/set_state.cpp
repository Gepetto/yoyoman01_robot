/*#include <std_srvs/Empty.h>
//#include <roscpp>
#include <physics/physics.hh>
//#include "ros/ros.h"
int main(){
	//gazebo::physics::PhysicsEngine::SetGravity ([-0,0085608389 0 9,8099962646]);
}*/

#include <ros/ros.h>
//引入的是ROS Service的type
//因此在開新專案的時候，就需要引入Service端的Package相依性
//格式為<Server的Package名稱/Service名稱.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;

int main(int argc,char **argv)
{
    //初使化ROS的Client端
    ros::init(argc,argv,"move_yoyoman01_by_magic_node"); //此字串不能有空白

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    //設定yoyoman01 Position
    geometry_msgs::Point yoyoman01_position;
    yoyoman01_position.x = 0.116245021398;
    yoyoman01_position.y = -0.0763049851059;
    yoyoman01_position.z = 0.815279089915;
    //設定yoyoman01 orientation
    geometry_msgs::Quaternion yoyoman01_orientation;
    yoyoman01_orientation.x = 0.0497451000883;
    yoyoman01_orientation.y = -0.0331579906013;
    yoyoman01_orientation.z = -0.0327788139959;
    yoyoman01_orientation.w = 0.998209016642;

    //設定yoyoman01 pose (Pose + Orientation)
    geometry_msgs::Pose yoyoman01_pose;
    yoyoman01_pose.position = yoyoman01_position;
    yoyoman01_pose.orientation = yoyoman01_orientation;
    
    //twist linear
    geometry_msgs::Vector3 yoyoman01_twist_linear;
    yoyoman01_twist_linear.x = -0.186742570216;
    yoyoman01_twist_linear.y = 0.0402938445798;
    yoyoman01_twist_linear.z = -0.00844364927522;
    
    //twist angulare
    geometry_msgs::Vector3 yoyoman01_twist_angular;
    yoyoman01_twist_angular.x = 0.76056627175555;
    yoyoman01_twist_angular.y = 0.5703256717500523;
    yoyoman01_twist_angular.z = 0.5760695316148732;
    
    //twist angular+linear
    geometry_msgs::Twist yoyoman01_twist;
    yoyoman01_twist.linear = yoyoman01_twist_linear;
    yoyoman01_twist.angular = yoyoman01_twist_angular;
    
    //設定ModelState
    gazebo_msgs::ModelState yoyoman01_modelstate;
    yoyoman01_modelstate.model_name = (std::string) "yoyoman01";
    yoyoman01_modelstate.pose = yoyoman01_pose;
    yoyoman01_modelstate.twist = yoyoman01_twist;

    //準備設定gazebo中yoyoman01瞬移後的位置pose
    ros::Duration(0.4).sleep();
	ROS_INFO("set_state");
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = yoyoman01_modelstate;

    //跟Server端連線，送出yoyoman01要瞬移的位置
    if(client.call(srv))
    {
        ROS_INFO("yoyoman01's magic moving success!!");
    }
    else
    {
        ROS_ERROR("Failed to magic move yoyoman01! Error msg:%s",srv.response.status_message.c_str());
    }
    return 0;
}
