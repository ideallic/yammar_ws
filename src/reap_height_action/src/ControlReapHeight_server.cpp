#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "reap_height_action/ControlReapHeightAction.h"

typedef actionlib::SimpleActionServer<reap_height_action::ControlReapHeightAction> Server;

// 收到action的goal后调用的回调函数
void execute(const reap_height_action::ControlReapHeightGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    reap_height_action::ControlReapHeightFeedback feedback;

    ROS_INFO("Dishwasher %d is working.", goal->dishwasher_id);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    for(int i=1; i<=10; i++)
    {
        feedback.percent_complete = i * 10;
        as->publishFeedback(feedback);
        r.sleep();
    }

	// 当action完成后，向客户端返回结果
    ROS_INFO("Dishwasher %d finish working.", goal->dishwasher_id);
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_reap_height_server");
    ros::NodeHandle n;

	// 定义一个服务器，do_dishes就是topic
    Server server(n, "control_reap_height", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
