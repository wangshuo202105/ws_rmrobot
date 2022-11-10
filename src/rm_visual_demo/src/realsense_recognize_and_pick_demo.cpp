#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <rm_msgs/Gripper_Pick.h>
#include <rm_msgs/Gripper_Set.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveC.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/Plan_State.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


enum PLAN_STATE
{
    NOPLAN,
    PLAN_ING,
    PLAN_OK,
    PLAN_FAIL,
}plan_state;

//marker标记物转换到base_link下的位姿
geometry_msgs::PoseStamped transed_pose;
geometry_msgs::PoseStamped marker_pose;

bool updateMarkerPos_flag = false; //是否需要更新Marker标记物的位姿
bool findMarkerObj = false; //判断是否发现标记物
bool isBeginPose = false;   //判断是否处于起始姿态
//bool planState = false; //规划返回状态
bool isGripperOpen = false; //判断抓手是否处于打开状态

ros::Publisher gripper_pick_pub, gripper_set_pub, moveJ_pub, moveL_pub;

int wait_count = 0; //等待机械臂返回状态尝试次数


// 接收到订阅的消息后，会进入消息回调函数
void markerObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    if (updateMarkerPos_flag)
    {
        marker_pose.pose = msg->pose;
        marker_pose.header = msg->header;
        findMarkerObj = true;
    }

}

// 机械臂执行规划状态回调函数
void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    if (plan_state == PLAN_ING)
    {
        plan_state = msg->state? PLAN_OK : PLAN_FAIL;
        ROS_INFO("*******Plan State %s*******", msg->state? "OK" : "Fail");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rm65_realsense");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(3);
    spin.start();

    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface group("arm");
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);


    //定义一个MoveJ指令的起始位姿
    rm_msgs::MoveJ moveJ_BeginPose;
    moveJ_BeginPose.joint[0] = -0.360829;
    moveJ_BeginPose.joint[1] = 0.528468;
    moveJ_BeginPose.joint[2] = 1.326293;
    moveJ_BeginPose.joint[3] = -0.000454;
    moveJ_BeginPose.joint[4] = 1.221748;
    moveJ_BeginPose.joint[5] = 0.000052;
    moveJ_BeginPose.speed = 0.3;

    //定义一个MoveJ指令的放置的位姿
    rm_msgs::MoveJ moveJ_PlacePose;
    moveJ_PlacePose.joint[0] = 0.587408;
    moveJ_PlacePose.joint[1] = 0.794997;
    moveJ_PlacePose.joint[2] = 1.205796;
    moveJ_PlacePose.joint[3] = 0.000628;
    moveJ_PlacePose.joint[4] = 1.062644;
    moveJ_PlacePose.joint[5] = 0.000052;
    moveJ_PlacePose.speed = 0.4;

    /*
     * 1.相关初始化
     */

    plan_state = NOPLAN;

    //创建控制夹爪的Publisher
    gripper_pick_pub = nh.advertise<rm_msgs::Gripper_Pick>("/rm_driver/Gripper_Pick", 10);
    gripper_set_pub = nh.advertise<rm_msgs::Gripper_Set>("/rm_driver/Gripper_Set", 10);

    //空间规划指令Publisher
    moveJ_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    //直线规划指令Publisher
    moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);

    /*
     * 订阅realsense使用aruco识别到标记物pose的topic
     */
    //创建一个Subscriber，订阅名为/aruco_single/pose的topic，注册回调函数markerObjectPoseCallback
    ros::Subscriber pose_sub = nh.subscribe("/aruco_single/pose", 10, markerObjectPoseCallback);
    //创建机械臂规划运行状态的Subscriber
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    //坐标变换Listener
    tf::TransformListener tfListener;


    //设置抓手的抓取力度和速度
    ros::Duration(1.0).sleep();
    rm_msgs::Gripper_Pick gripperPick;
    gripperPick.force = 100;
    gripperPick.speed = 500;
    gripper_pick_pub.publish(gripperPick);

    ros::Duration(3.0).sleep();

    rm_msgs::Gripper_Set gripperSet;


    while (ros::ok())
    {
        updateMarkerPos_flag = false;

        //确保抓手打开才能进行下一步操作
        if (!isGripperOpen)
        {
            ROS_INFO("**********Set Gripper Open**********");

            gripperSet.position = 1000;
            gripper_set_pub.publish(gripperSet); //打开抓手

            ros::Duration(3.0).sleep();
            isGripperOpen = true;

        }

        /**
         * 2.控制机械臂运动到识别抓取的初始位姿
         */
        //判断是否在起始位姿，如果不在则先控制机械臂移动到初始位姿
        while (!isBeginPose)
        {
            ROS_INFO("**********MoveJ to BeginPose Start**********");
            plan_state = PLAN_ING;
            //发布空间规划指令运动到起始位姿
            moveJ_pub.publish(moveJ_BeginPose);

            //等待规划运动完成返回
            while (plan_state == PLAN_ING && wait_count < 10)
            {
                ros::Duration(1.0).sleep();
                wait_count++;
            }
            wait_count = 0;

            if (plan_state == PLAN_OK)
            {
                isBeginPose = true;
            } else {
                plan_state = PLAN_FAIL;
            }
        }


        ros::Duration(1.0).sleep();
        /**
         * 3.等待2秒,然后判断是否识别到抓取物,如果没有识别到marker标记物则一直等待直到识别到
         */
        findMarkerObj = false;
        updateMarkerPos_flag = true;
        ros::WallDuration(1.0).sleep(); //等待1秒
        //如果没有识别到marker标记物则一直等待直到识别到
        while (!findMarkerObj) {
            ros::WallDuration(2.0).sleep(); //等待2秒
        }
        updateMarkerPos_flag = false;

        /**
         * 4.开始转换坐标系并控制机械臂抓取放置操作
         */
        try {

            //获取base_link和camera_color_frame之间的关系，也就是手眼标定的结果
            ROS_INFO("waitForTransform ok [base_link, camera_color_frame]");
            tfListener.waitForTransform("/base_link", "/camera_color_frame", ros::Time(0), ros::Duration(50.0));

            tfListener.transformPose("base_link", marker_pose, transed_pose);
            ROS_INFO("Transed pose->position[%f, %f, %f]", transed_pose.pose.position.x, transed_pose.pose.position.y,
                     transed_pose.pose.position.z);
            ROS_INFO("Transed pose->orientation[%f, %f, %f, %f]", transed_pose.pose.orientation.x,
                     transed_pose.pose.orientation.y, transed_pose.pose.orientation.z, transed_pose.pose.orientation.w);

            //确保抓手不会碰到台面
            if (transed_pose.pose.position.z + 0.12 > 0.192) {
                //获取末端当前位姿
                geometry_msgs::PoseStamped currPose_ToolLink = group.getCurrentPose("Link6");

                tf::Quaternion transed_pose_quat;
                tf::quaternionMsgToTF(transed_pose.pose.orientation, transed_pose_quat);

                double roll, pitch, yaw;
                tf::Matrix3x3(transed_pose_quat).getRPY(roll, pitch, yaw);   //将抓取物姿态的四元数转换为欧拉角


                geometry_msgs::Pose pickPose;

                pickPose.orientation = currPose_ToolLink.pose.orientation;

                pickPose.position.x = transed_pose.pose.position.x;
                pickPose.position.y = transed_pose.pose.position.y;
                pickPose.position.z = transed_pose.pose.position.z + 0.13;  //z轴方向需加上抓手的长度进行偏移


                //q_orig  是原姿态转换的tf的四元数
                tf2::Quaternion q_orig;
                tf2::convert(pickPose.orientation, q_orig);

                q_orig.normalize(); //归一化
                //将旋转后的tf四元数转换为msg四元数
                tf2::convert(q_orig, pickPose.orientation);
                

                //发布直线规划指令运动到抓取位姿
                ROS_INFO("**********MoveL to PickPose Start**********");
                plan_state = PLAN_ING;
                rm_msgs::MoveL moveL_pickPose;
                moveL_pickPose.Pose = pickPose;
                moveL_pickPose.speed = 0.5;
                moveL_pub.publish(moveL_pickPose);

                while (plan_state == PLAN_ING && wait_count < 20)
                {
                    ros::Duration(1.0).sleep();
                    wait_count++;
                }
                wait_count = 0;

                if (plan_state == PLAN_OK)
                {
                    isBeginPose = false;
                } else {
                    plan_state = PLAN_FAIL;
                }


                if (plan_state == PLAN_OK)
                {
                    if (isGripperOpen)
                    {
                        //控制手爪关闭
                        ROS_INFO("**********Set Gripper Close**********");
                        gripperSet.position = 100;
                        gripper_set_pub.publish(gripperSet); //关闭抓手

                        ros::Duration(3.0).sleep();
                        isGripperOpen = false;

                    }


                    //控制机械臂抬起
                    geometry_msgs::Pose upPose;
                    upPose.position.x = pickPose.position.x;
                    upPose.position.y = pickPose.position.y;
                    upPose.position.z = pickPose.position.z + 0.03;  //z轴方向需加上抓手的长度进行偏移
                    upPose.orientation = pickPose.orientation;

                    //发布直线规划指令运动到抬起位姿
                    ROS_INFO("**********MoveL to UpPose Start**********");
                    plan_state = PLAN_ING;
                    rm_msgs::MoveL moveL_upPose;
                    moveL_upPose.Pose = upPose;
                    moveL_upPose.speed = 0.5;
                    moveL_pub.publish(moveL_upPose);

                    while (plan_state == PLAN_ING && wait_count < 10) {
                        ros::Duration(1.0).sleep();
                        wait_count++;
                    }
                    wait_count = 0;

                    plan_state = (plan_state == PLAN_OK) ? PLAN_OK : PLAN_FAIL;


                    if (plan_state == PLAN_OK) {
                        //发布空间规划指令运动到放置位姿
                        ROS_INFO("**********MoveL to PlacePose Start**********");
                        plan_state = PLAN_ING;
                        moveJ_pub.publish(moveJ_PlacePose);

                        while (plan_state == PLAN_ING && wait_count < 10) {
                            ros::Duration(1.0).sleep();
                            wait_count++;
                        }
                        wait_count = 0;

                        plan_state = (plan_state == PLAN_OK) ? PLAN_OK : PLAN_FAIL;


                        if (plan_state == PLAN_OK) {

                            //控制手爪放开
                            ROS_INFO("**********Set Gripper Open**********");
                            gripperSet.position = 1000;
                            gripper_set_pub.publish(gripperSet); //打开抓手

                            ros::Duration(2.0).sleep();
                            isGripperOpen = true;


                            //发布直线规划指令运动到抬起离开的位姿
                            ROS_INFO("**********MoveL to LeavePlacePose Start**********");
                            plan_state = PLAN_ING;
                            currPose_ToolLink = group.getCurrentPose("Link6");
                            geometry_msgs::Pose leavePlacePose;
                            leavePlacePose.orientation = currPose_ToolLink.pose.orientation;
                            leavePlacePose.position.x = currPose_ToolLink.pose.position.x;
                            leavePlacePose.position.y = currPose_ToolLink.pose.position.y;
                            leavePlacePose.position.z = currPose_ToolLink.pose.position.z + 0.03;  //z轴方向需加上抓手的长度进行偏移
                            rm_msgs::MoveL moveL_leavePose;
                            moveL_leavePose.Pose = leavePlacePose;
                            moveL_leavePose.speed = 0.5;
                            moveL_pub.publish(moveL_leavePose);

                            while (plan_state == PLAN_ING && wait_count < 10) {
                                ros::Duration(1.0).sleep();
                                wait_count++;
                            }
                            wait_count = 0;

                            plan_state = (plan_state == PLAN_OK) ? PLAN_OK : PLAN_FAIL;

                        }

                    }

                }

            }

        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
            ros::Duration(1.0).sleep();
        }

    }


    ros::waitForShutdown();

    return 0;
}

