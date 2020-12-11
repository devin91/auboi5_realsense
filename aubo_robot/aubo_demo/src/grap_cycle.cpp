#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grap_demo");
    ros::NodeHandle nh;
    ros::ServiceClient recognition_client = nh.serviceClient<std_srvs::SetBool>("start_recognition");
    ros::ServiceClient grap_client = nh.serviceClient<std_srvs::SetBool>("start_grap");
    int i = 0;
    while (ros::ok() && i > 20)
    {
        bool has_obj = false;
        std_srvs::SetBool rec;
        rec.request.data = true;
        if (recognition_client.call(rec))
        {
            if (rec.response.success)
            {
                has_obj = true;
            }
            else
            {
                has_obj = false;
                ROS_WARN("Could not found object.");
                ros::spinOnce();
                sleep(5);
                continue;
            }
        }
        else
        {
            ROS_ERROR("Call recognition object service failed.");
            ros::spinOnce();
            sleep(5);
            continue;
        }
        ros::spinOnce();
        sleep(1);
        if (has_obj)
        {
            std_srvs::SetBool grap;
            grap.request.data = true;
            if (grap_client.call(grap))
            {
                if (grap.response.success)
                {
                    ROS_INFO("Action has done : %d.",i);
                }else
                {
                    ROS_ERROR("Grap action failed:%s",grap.response.message.c_str());
                    ros::spinOnce();
                    sleep(3);
                    continue;
                }
                
            }
        }
        i ++;
        ros::spinOnce();
        sleep(5);
    }
    return 0;
}