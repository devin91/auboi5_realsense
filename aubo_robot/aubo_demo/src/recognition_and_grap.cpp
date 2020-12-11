#include <ros/ros.h>
#include <std_srvs/SetBool.h>
//object recognition
#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#include <dh_hand_driver/ActuateHandAction.h>

typedef actionlib::SimpleActionClient<dh_hand_driver::ActuateHandAction> Client;
typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> client;
namespace rvt = rviz_visual_tools;

struct landPose
{
    geometry_msgs::Pose recognition_pose; //识别点
    geometry_msgs::Pose ready_pose;  //准备点
    geometry_msgs::Pose grap_pose;   //抓取点
    geometry_msgs::Pose up_pose;     //抬起点
    geometry_msgs::Pose target_pose; //目标点
    geometry_msgs::Pose action_pose; //动作姿态
    geometry_msgs::Pose exit_pose;   //退出点
};

struct tfStruct
{
    std::string link;
    std::string base_link;
    tf::Transform transform;
};





class DH_HandActionClient
{
private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState &state,
                const dh_hand_driver::ActuateHandResultConstPtr &result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("result  : %i", result->opration_done);
    }

    // when target active, call this once
    void ActiveCb()
    {
        ROS_INFO("Goal just went active");
    }

    // received feedback
    void FeedbackCb(
        const dh_hand_driver::ActuateHandFeedbackConstPtr &feedback)
    {
        ROS_INFO("Got Feedback: %i", feedback->position_reached);
    }

public:
    DH_HandActionClient(const std::string client_name, bool flag = true) : client(client_name, flag)
    {
    }

    //client start
    void Start(int32_t motorID, int32_t setpos, int32_t setforce)
    {
        ROS_INFO("wait server");
        client.waitForServer();
        //set goal
        dh_hand_driver::ActuateHandGoal goal;
        // AG2E just has one motor (ID:1)
        // AG3E has two motor (ID:1 and 2)
        goal.MotorID = motorID;
        goal.force = setforce;
        goal.position = setpos;

        ROS_INFO("Send goal %d %d %d", goal.MotorID, goal.force, goal.position);
        //sen goal
        client.sendGoal(goal,
                        boost::bind(&DH_HandActionClient::DoneCb, this, _1, _2),
                        boost::bind(&DH_HandActionClient::ActiveCb, this),
                        boost::bind(&DH_HandActionClient::FeedbackCb, this, _1));
        ROS_INFO("wait result");

        client.waitForResult(ros::Duration(15.0));

        //process the result
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Send commond succeeded");
        else
        {
            ROS_WARN("Cancel Goal!");
            client.cancelAllGoals();
        }

        printf("Current State: %s\n", client.getState().toString().c_str());
    }

private:
    Client client;
};

class RecognitionAndGrap
{
public:
    RecognitionAndGrap(ros::NodeHandle &nh, tf::TransformListener &tf, tf::TransformBroadcaster &tb) : nh_(nh), tf_(tf), tb_(tb), ac_("recognize_objects", true), obj_num_(0),
                                                                                                       move_group("manipulator_i5"), visual_tools("base_link"),dh_client_("actuate_hand",true)
    {
        ac_.waitForServer();
        start_server_ = nh_.advertiseService("start_recognition", &RecognitionAndGrap::callback, this);
        do_server_ = nh_.advertiseService("start_grap", &RecognitionAndGrap::doCallback, this);

        move_group.setPoseReferenceFrame("base_link");
        joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator_i5");
        visual_tools.deleteAllMarkers();
        //Load remote control tool
        visual_tools.loadRemoteControl();
        move_group.setMaxVelocityScalingFactor(0.1);

        text_pose = Eigen::Affine3d::Identity();
        text_pose.translation().z() = 1.2;
        visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
        // Text visualization takes effect
        visual_tools.trigger();

        //aubo i5 goto capture positon
        tf::Quaternion q;
        q.setRPY(2.01, 0, -1.57); //radian

        geometry_msgs::Pose target_pose;
        target_pose.position.x = -0.307;
        target_pose.position.y = -0.128;
        target_pose.position.z = 0.548;
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        gotoPosition(target_pose);
        tf_thread_ = std::thread(&RecognitionAndGrap::tfThread, this);
    }

    ~RecognitionAndGrap()
    {
        if (tf_thread_.joinable())
        {
            tf_thread_.join();
        }
    }

private:
    bool gotoPosition(geometry_msgs::Pose target_pose)
    {
        move_group.setPoseTarget(target_pose);

        // Call the planner for planning calculations Note: This is just planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");
        if(!success)
        {
            return false;
        }
        // visual planning path in Rviz
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose, "pose1");
        visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
        // Parameter 1 (trajectory_): path information
        // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

        // Perform planning actions
        move_group.execute(my_plan);
        return true;
        //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go on");
    }

    bool computePose(landPose &positions, std::string source_link, std::string base_link = "base_link", std::string target_link = "")
    {
        geometry_msgs::PoseStamped source_pose;
        geometry_msgs::PoseStamped land_pose;
        source_pose.header.frame_id = source_link;
        source_pose.pose.position.x = 0;
        source_pose.pose.position.y = 0;
        source_pose.pose.position.z = 0;
        source_pose.pose.orientation.x = 0;
        source_pose.pose.orientation.y = 0;
        source_pose.pose.orientation.z = 0;
        source_pose.pose.orientation.w = 1.0;

        tf::Quaternion q;
        q.setRPY(-1.57, 3.14, 0);
        //compute grap pose
        source_pose.pose.position.x = 0.05;
        source_pose.pose.position.y = -0.085;
        source_pose.pose.position.z = 0.06;
        source_pose.pose.orientation.x = q.x();
        source_pose.pose.orientation.y = q.y();
        source_pose.pose.orientation.z = q.z();
        source_pose.pose.orientation.w = q.w();
        try
        {
            tf_.transformPose(base_link, source_pose, land_pose);
            positions.grap_pose = land_pose.pose;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Received an exception: %s", ex.what());
            return false;
        }
        //compute ready pose
        source_pose.pose.position.y = -0.2;
        try
        {
            tf_.transformPose(base_link, source_pose, land_pose);
            positions.ready_pose = land_pose.pose;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Received an exception: %s", ex.what());
            return false;
        }
        //compute up pose
        source_pose.pose.position.y = -0.085;
        source_pose.pose.position.z = 0.20;
        try
        {
            tf_.transformPose(base_link, source_pose, land_pose);
            positions.up_pose = land_pose.pose;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Received an exception: %s", ex.what());
            return false;
        }
        //compute target pose/ action pose / exit pose
        if (target_link == "")
        {
            if (positions.up_pose.position.y >= 0)
            {
                source_pose.pose.position.x = -0.15;
            }
            else
            {
                source_pose.pose.position.x = 0.15;
            }
            try
            {
                tf_.transformPose(base_link, source_pose, land_pose);
                positions.target_pose = land_pose.pose;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Received an exception: %s", ex.what());
                return false;
            }
            source_pose.pose.position.z = 0.060;
            try
            {
                tf_.transformPose(base_link, source_pose, land_pose);
                positions.action_pose = land_pose.pose;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Received an exception: %s", ex.what());
                return false;
            }
            source_pose.pose.position.y = -0.2;
            try
            {
                tf_.transformPose(base_link, source_pose, land_pose);
                positions.exit_pose = land_pose.pose;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Received an exception: %s", ex.what());
                return false;
            }
        }
        else
        {
        }
    }

    bool doCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            bool success = false;
            landPose land_pose;
            computePose(land_pose, "target_0");
            //goto ready_pose
            success = gotoPosition(land_pose.ready_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to ready pose failed.";
                return true;
            }
            //goto grap_pose
            success = gotoPosition(land_pose.grap_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to grap pose failed.";
                return true;
            }
            //wait for gripper
            dh_client_.Start(1,40,30);
            ros::spinOnce();
            //goto up_pose
            success = gotoPosition(land_pose.up_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to up pose failed.";
                return true;
            }
            //goto target pose
            success = gotoPosition(land_pose.target_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to target pose failed.";
                return true;
            }
            //goto action pose
            success = gotoPosition(land_pose.action_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to action pose failed.";
                return true;
            }
            //wait for gripper
            dh_client_.Start(1,100,100);
            ros::spinOnce();
            //goto exit pose
            success = gotoPosition(land_pose.exit_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to exit pose failed.";
                return true;
            }
            //goto capture pose
            tf::Quaternion q;
            q.setRPY(2.01, 0, -1.57); //radian

            geometry_msgs::Pose target_pose;
            target_pose.position.x = -0.307;
            target_pose.position.y = -0.128;
            target_pose.position.z = 0.548;
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
            success = gotoPosition(target_pose);
            if(!success)
            {
                res.success = false;
                res.message = "Move to capture pose failed.";
                return true;
            }
            ROS_INFO("All action has done.");
            res.success = true;
        }
        return true;
    }

    void doneCb(const actionlib::SimpleClientGoalState &state,
                const object_recognition_msgs::ObjectRecognitionResultConstPtr &result)
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        tf_v_.clear();
        if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (!result->recognized_objects.objects.empty())
            {
                obj_num_ = result->recognized_objects.objects.size();
                for (int i = 0; i < obj_num_; i++)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.pose = result->recognized_objects.objects.at(0).pose.pose.pose;
                    pose.header = result->recognized_objects.objects.at(0).pose.header;

                    //tf　transform
                    geometry_msgs::PoseStamped base_pose;
                    try
                    {

                        tf_.transformPose("base_link", pose, base_pose);
                        ROS_INFO("camera_link:(%f,%f,%f,%f,%f,%f,%f)------>base_link:(%f,%f,%f,%f,%f,%f,%f).",
                                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
                                 base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z,
                                 base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w);

                        //保存工件坐标系
                        tf::Transform transform;
                        std::string link = "target_";
                        tfStruct tf_st;
                        link.append(std::to_string(i));
                        ROS_INFO("Creat a link named %s.", link.c_str());
                        transform.setOrigin(tf::Vector3(base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z));
                        transform.setRotation(tf::Quaternion(base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w));
                        tf_st.link = link;
                        tf_st.base_link = "base_link";
                        tf_st.transform = transform;
                        tf_v_.push_back(tf_st);
                        //tb_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", link));
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("Received an exception: %s", ex.what());
                    }
                }
            }
            else
            {
                obj_num_ = 0;
                ROS_WARN("Could not find target.");
            }
        }
        else
        {
            obj_num_ = 0;
            ROS_WARN("Could not find target.");
        }
    }
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }
    void feedbackCb(const object_recognition_msgs::ObjectRecognitionFeedbackConstPtr &feedback)
    {
        ROS_INFO("Got Feedback");
    }

    bool callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            //开始
            object_recognition_msgs::ObjectRecognitionGoal goal;
            ac_.sendGoal(goal,
                         boost::bind(&RecognitionAndGrap::doneCb, this, _1, _2),
                         boost::bind(&RecognitionAndGrap::activeCb, this),
                         boost::bind(&RecognitionAndGrap::feedbackCb, this, _1));
            ac_.waitForResult();
            if(obj_num_ >0)
            {
                res.success = true;
            }
            else
            {
                res.success = false;
                res.message = "Could not recognition object.";
            }
        }
        else
        {
            //取消
            ac_.cancelAllGoals();
            res.success = true;
        }

        return true;
    }

    void tfThread()
    {
        ros::Rate r(30);
        while (ros::ok())
        {
            {
                boost::lock_guard<boost::mutex> lock(mutex_);
                if (!tf_v_.empty())
                {
                    for (int i = 0; i < tf_v_.size(); i++)
                    {
                        //ROS_INFO("TF broadcaster a transform.");
                        tb_.sendTransform(tf::StampedTransform(tf_v_[i].transform, ros::Time::now(), tf_v_[i].base_link, tf_v_[i].link));
                    }
                }
            }
            r.sleep();
        }
    }

    ros::NodeHandle nh_;
    client ac_;
    tf::TransformListener &tf_;
    tf::TransformBroadcaster &tb_;
    ros::ServiceServer start_server_;
    ros::ServiceServer do_server_;

    //tf broadcaster thread
    std::thread tf_thread_;
    std::vector<tfStruct> tf_v_;
    boost::mutex mutex_;

    DH_HandActionClient dh_client_;

    int obj_num_;

    // Define the planning group name
    //std::string PLANNING_GROUP = "manipulator_i5";

    //Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group;

    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a robot model information object
    const robot_state::JointModelGroup *joint_model_group;

    // Create an object of the visualization class
    moveit_visual_tools::MoveItVisualTools visual_tools;

    Eigen::Affine3d text_pose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recognition_and_grap_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    tf::TransformListener tf;
    tf::TransformBroadcaster tb;
    RecognitionAndGrap rag(nh, tf, tb);

    ROS_INFO("recognition and grap node initialized.");
    ros::spin();
    return 0;
}