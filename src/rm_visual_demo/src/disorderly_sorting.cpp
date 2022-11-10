//
//增加帧数判断稳定图像
// Created by ubuntu on 22-6-30.
//
#include <ros/ros.h>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <rm_msgs/MoveL.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Socket_Command.h>
#include <rm_msgs/ChangeTool_Name.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Arm_Digital_Output.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <sophus/se3.hpp>



using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;


typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


static const std::string OPENCV_WINDOW = "Disorderly Sorting";
//查看是否获取相机坐标系与世界坐标系之间的转换
int get_trans = 0;

int task = 0; 		 // 1代表处理图像，2代表机械臂执行任务；3代表TCP控制开始分拣，4代表TCP控制结束分拣
int grasp_once = 1;  //1代表抓取完成并回到初始位姿，0代表未完成抓取
int task_step = 6;   //6代表初始化时的回调
int color = 1;		 //1代表绿色，2代表红色，3代表黄色，4代表紫色
int num_frame = 1;
int pause_sort = 0;		//暂停与恢复指令的控制变量，1代表暂停




//木块放置点位姿与机械臂初始位姿
rm_msgs::MoveJ_P Place_Pose, Initial_Pose, Grasp_Pose, Before_Grasp_Pose, ForwordPose;


//用于记录初始位姿下相机坐标系与世界坐标系的转换
Sophus::SE3d trans_camera2base;


//话题发布
ros::Publisher moveJ_P_pub, moveL_pub, changetool_pub, ARM_Digital_Output_pub;
//话题订阅
ros::Subscriber planState_sub;


//相机内参
//K: [605.8885498046875, 0.0, 322.9019470214844, 0.0, 605.9415283203125, 237.97732543945312, 0.0, 0.0, 1.0]
double cx = 322.90;
double cy = 237.98;
double fx = 605.89;
double fy = 605.94;
double depthScale = 1000.0;





Sophus::SE3d get_transpose()
{
    //监听坐标变换
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/base_link", "/camera_color_frame", ros::Time(0), ros::Duration(30));
        listener.lookupTransform("/base_link", "/camera_color_frame",ros::Time(0), transform);

        get_trans = 1;

        ROS_INFO("Transed pose->position[%f,%f, %f, %f，%f，%f，%f]",
                 transform.getRotation().getW(),
                 transform.getRotation().getX(),
                 transform.getRotation().getY(),
                 transform.getRotation().getZ(),
                  transform.getOrigin().x(),
		transform.getOrigin().y(),
                transform.getOrigin().z());

    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    if(get_trans)
    {
        Sophus::SE3d trans(Eigen::Quaterniond(transform.getRotation().getW(),
					      transform.getRotation().getX(),
					      transform.getRotation().getY(), 
					      transform.getRotation().getZ()),
                           Eigen::Vector3d(transform.getOrigin().x(), 
			  	           transform.getOrigin().y(), 
				           transform.getOrigin().z()));
        return trans;
    }
    else
    {
        ROS_INFO("Listening frame transformation between carema and base_link failed!");
        Sophus::SE3d trans(Eigen::Quaterniond(50, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
        return trans;
    }
}


void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来，显示机械臂是否完成运动
    if(msg->state)
    {
    	while(pause_sort){}
        //机械臂到达预抓取位置，发布抓取位姿
        if(task_step == 1)
        {
            if(task != 4)
            {
                ros::Duration(1.0).sleep();
                while(pause_sort){};
                task_step = 2;
                moveL_pub.publish(Grasp_Pose);
                //moveJ_P_pub.publish(Grasp_Pose);
                ROS_INFO("step1");
            }

        }
        //机械臂到达抓取位置并抓取，发布预抓取位置
        else if(task_step == 2)
        {
            if(task != 4)
            {
                //ros::Duration(2.0).sleep();
                //吸取，1秒延时，方便暂停吸取物体
                ros::Duration(1.0).sleep();
                while(pause_sort){};
                rm_msgs::Arm_Digital_Output state;
                state.num = 2;
                state.state = 1;
                ARM_Digital_Output_pub.publish(state);
                ROS_INFO("get the target");
           }
			
            //吸取后1秒钟延时，方便暂停机械臂移动
            ros::Duration(1.0).sleep();
            while(pause_sort){};
            ROS_INFO("ready to push");
			
            if(task != 4)
            {
                task_step = 3;
                moveL_pub.publish(Before_Grasp_Pose);
                //moveJ_P_pub.publish(Before_Grasp_Pose);
                //moveJ_P_pub.publish(Place_Pose);
                ROS_INFO("step2");
            }
        }
        //机械臂抓取完成到达预抓取位置，发布放置位置
        else if (task_step == 3)
        {
            if(task != 4)
            {
                ros::Duration(1.0).sleep();
                while(pause_sort){};
                task_step = 4;
                moveJ_P_pub.publish(Place_Pose);
                ROS_INFO("step3");
            }
        }
        //机械臂到达放置位置并放下物体，发布初始位置
        else if(task_step == 4)
        {
            //放置，1秒延时，方便暂停放置物体
            ros::Duration(1.0).sleep();
            while(pause_sort){};
            if(task != 4)
            {
                rm_msgs::Arm_Digital_Output state;
                state.num = 2;
                state.state = 0;
                ARM_Digital_Output_pub.publish(state);
                ROS_INFO("push the target");
            }

            //放置后1秒钟延时，方便暂停机械臂移动
            ros::Duration(1.0).sleep();
            while(pause_sort){};
            ROS_INFO("ready to move");

            if(task != 4)
            {
                task_step = 5;
                //ros::Duration(2.0).sleep();
                moveJ_P_pub.publish(Initial_Pose);
                ROS_INFO("step4");
            }
        }
        //机械臂完成一次抓取，到达初始位姿，并初始化下一次抓取参数
        else if(task_step == 5)
        {
            //完成一次抓取后，1秒钟延时，方便暂停下一次抓取
            ros::Duration(1.0).sleep();
            while(pause_sort){};
            ROS_INFO("ready to grasp next target");

            if(task != 4)
            {
                ros::Duration(2.0).sleep();
                task_step = 1;
                task = 1;
                grasp_once = 1;
                ROS_INFO("step5");
            }
        }
        //机械臂初始化完成
        else if(task_step == 6)
        {
            task_step = 1;
            ROS_INFO("initial step");
        }
        //机械臂接收到结束指令
        else if(task_step == 7)
            ROS_INFO("Stop sort!");
        //中途规划异常，初始化
        else if(task_step == 8)
        {
            ros::Duration(2.0).sleep();
            grasp_once = 1;
            task = 1;
            task_step = 1;
        }

    }
    else 
    {
        if(task != 4)
        {
            ROS_INFO("step %d plan failed!", task_step);
            ROS_INFO("Retrying...");

            rm_msgs::Arm_Digital_Output state;
            state.num = 2;
            state.state = 0;
            ARM_Digital_Output_pub.publish(state);


            Initial_Pose.Pose.position.x = -0.3;//-0.382839;
            Initial_Pose.Pose.position.y = 0;//0.001034;
            Initial_Pose.Pose.position.z = 0.32;//0.210839;
            Initial_Pose.Pose.orientation.x = -0.999668;
            Initial_Pose.Pose.orientation.y = -0.000793;
            Initial_Pose.Pose.orientation.z = 0.025765;
            Initial_Pose.Pose.orientation.w = 0.000019;
            Initial_Pose.speed = 0.3;

            task_step = 8;
            moveJ_P_pub.publish(Initial_Pose);
        }
    }

}


void TcpComCallback(const rm_msgs::Socket_Command::ConstPtr& msg)
{
    //开始
    if(msg->command == 1)
    {
        task = 3;
        ROS_INFO("start");
    }
    //暂停
    else if(msg->command == 2)
    {
        pause_sort = 1;
        ROS_INFO("pause");
    }
    //继续
    else if(msg->command == 3)
    {
        pause_sort = 0;
        ROS_INFO("continue");
    }
    //结束
    else if(msg->command == 4)
    {
        task = 4;
        ROS_INFO("end");
    }
}




void pick_place_once(double* pointWorld)
{
    //机械臂的最终抓取位姿，z方向有补偿值
    tf::Quaternion q;
    q.setRPY(-3.141514, 0, 0);
    Grasp_Pose.Pose.position.x = pointWorld[0];
    Grasp_Pose.Pose.position.y = pointWorld[1]-0.005;
    Grasp_Pose.Pose.position.z = 0.185;
    //Grasp_Pose.Pose.position.z = pointWorld[2]+0.16;
    ROS_INFO("目标位置x,y,z:[%f, %f, %f]", pointWorld[0], pointWorld[1],pointWorld[2]);
    Grasp_Pose.Pose.orientation.x = q.x();
    Grasp_Pose.Pose.orientation.y = q.y();
    Grasp_Pose.Pose.orientation.z = q.z();
    Grasp_Pose.Pose.orientation.w = q.w();
    Grasp_Pose.speed = 0.4;


    ForwordPose.Pose.position.x = -0.38;
    ForwordPose.Pose.position.y = 0;
    ForwordPose.Pose.position.z = 0.26;
    ForwordPose.Pose.orientation.x = q.x();
    ForwordPose.Pose.orientation.y = q.y();
    ForwordPose.Pose.orientation.z = q.z();
    ForwordPose.Pose.orientation.w = q.w();
    ForwordPose.speed = 0.5;



    //机械臂先到达抓取物体上方3厘米处
    Before_Grasp_Pose.Pose.position.x = pointWorld[0];
    Before_Grasp_Pose.Pose.position.y = pointWorld[1]-0.005;
    Before_Grasp_Pose.Pose.position.z = 0.225;
    //Before_Grasp_Pose.Pose.position.z = pointWorld[2]+0.20;
    Before_Grasp_Pose.Pose.orientation.x = q.x();
    Before_Grasp_Pose.Pose.orientation.y = q.y();
    Before_Grasp_Pose.Pose.orientation.z = q.z();
    Before_Grasp_Pose.Pose.orientation.w = q.w();
    Before_Grasp_Pose.speed = 0.5;

    moveL_pub.publish(Before_Grasp_Pose);
    //moveJ_P_pub.publish(Before_Grasp_Pose);
    //moveJ_P_pub.publish(Grasp_Pose);
    //moveJ_P_pub.publish(ForwordPose);

}



double* get_center(Mat img_BGR, Mat img_depth)
{
    Mat imgHSV, imgThresholded;

    double *point_center = new double[5];
    //记录无用轮廓，实际为噪声
    int counter_Inter = 0;
	
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

	
    switch(color)
    {
        case 1:
        {
            ROS_INFO("green");
            //绿色HSV值
            iLowH = 35;//35
            iHighH = 99;//77
            iLowS = 43;//43
            iHighS = 255;
            iLowV = 46;//46
            iHighV = 255;
            //木块放置点机械臂位姿
            Place_Pose.Pose.position.x = 0;//-0.317239;
            Place_Pose.Pose.position.y = -0.3;//0.120903;
            Place_Pose.Pose.position.z = 0.3;//0.255765;
            Place_Pose.Pose.orientation.x = -0.999668;//-0.983404;
            Place_Pose.Pose.orientation.y = -0.000793;//-0.178432;
            Place_Pose.Pose.orientation.z = 0.025765;//0.032271;
            Place_Pose.Pose.orientation.w = 0.000019;//0.006129;
            Place_Pose.speed = 0.3;
            break;
        }
        case 2:
        {
            ROS_INFO("red");
            //红色HSV值
            iLowH = 0;
            iHighH = 5;
            iLowS = 76;
            iHighS = 255;
            iLowV = 76;
            iHighV = 255;
            //木块放置点机械臂位姿
            Place_Pose.Pose.position.x = -0.1;//-0.317239;
            Place_Pose.Pose.position.y = -0.3;//0.120903;
            Place_Pose.Pose.position.z = 0.3;//0.255765;
            Place_Pose.Pose.orientation.x = -0.999668;//-0.983404;
            Place_Pose.Pose.orientation.y = -0.000793;//-0.178432;
            Place_Pose.Pose.orientation.z = 0.025765;//0.032271;
            Place_Pose.Pose.orientation.w = 0.000019;//0.006129;
            Place_Pose.speed = 0.3;
            break;
        }
        case 3:
        {
            ROS_INFO("yellow");
            //黄色HSV值
            iLowH = 5;
            iHighH = 15;
            iLowS = 100;
            iHighS = 255;
            iLowV = 25;
            iHighV = 255;
            //木块放置点机械臂位姿
            Place_Pose.Pose.position.x = -0.2;//-0.317239;
            Place_Pose.Pose.position.y = -0.3;//0.120903;
            Place_Pose.Pose.position.z = 0.3;//0.255765;
            Place_Pose.Pose.orientation.x = -0.999668;//-0.983404;
            Place_Pose.Pose.orientation.y = -0.000793;//-0.178432;
            Place_Pose.Pose.orientation.z = 0.025765;//0.032271;
            Place_Pose.Pose.orientation.w = 0.000019;//0.006129;
            Place_Pose.speed = 0.3;
            break;
        }
        case 4:
        {
            ROS_INFO("purple");
            //紫色HSV值
            iLowH = 95;
            iHighH = 165;
            iLowS = 35;
            iHighS = 255;
            iLowV = 46;
            iHighV = 255;
            //木块放置点机械臂位姿
            Place_Pose.Pose.position.x = -0.3;//-0.317239;
            Place_Pose.Pose.position.y = -0.3;//0.120903;
            Place_Pose.Pose.position.z = 0.3;//0.255765;
            Place_Pose.Pose.orientation.x = -0.999668;//-0.983404;
            Place_Pose.Pose.orientation.y = -0.000793;//-0.178432;
            Place_Pose.Pose.orientation.z = 0.025765;//0.032271;
            Place_Pose.Pose.orientation.w = 0.000019;//0.006129;
            Place_Pose.speed = 0.3;
            break;
        }
        default:
        {
            color = 1;
            break;
        }
    }
	
    num_frame = num_frame+1;
	
    if(num_frame == 50)
    {
        //test  在回调函数里控制变量也可以
        num_frame = 35;
        if(color <5)
        {
            //BGR格式图片转换为HSV格式
            cvtColor(img_BGR, imgHSV, COLOR_BGR2HSV);

            //转换为阈值图像
            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
			
            //中值滤波
            medianBlur(imgThresholded, imgThresholded, 9);
		  
            //形态学开口（从前景中移除小物体）
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            //形态闭合（填充前景中的小孔）
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            //寻找轮廓
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            vector<double>angles;
            findContours(imgThresholded, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);


            vector<Moments> mu(contours.size());
            vector<Point2f> mc(contours.size());
			
            //test
            imshow("imgThresholded", imgThresholded);
            waitKey(1);
			
            if(contours.size() > 0)
            {
                for (size_t i = 0; i < contours.size(); ++i)
                {
                    double area = contourArea(contours[i]);
                    if (area < 1.5e3 || 1e5 < area)
                    {
                        counter_Inter = counter_Inter+1;
                        continue;
                    }
                    task = 2;
                    //绘制轮廓
                    drawContours(img_BGR, contours, i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);
		
                    // 计算每个轮廓所有矩
                    // 创建一个vector,元素个数为contours.size()，获得轮廓的所有最高达三阶所有矩
                    mu[i] = moments(contours[i], false );

                    // 计算轮廓的质心，质心的 X,Y 坐标：(m10/m00, m01/m00)
                    mc[i] = Point2f(static_cast<float>(mu[i].m10/mu[i].m00), static_cast<float>(mu[i].m01/mu[i].m00));
                    //ROS_INFO("area is : %f", area);

                    //计算世界坐标
                    int u = (int)mc[i].x;
                    int v = (int)mc[i].y;
                    // 深度值
                    unsigned int depth_point = img_depth.at<uint16_t>(u, v);

                    //相机坐标系下坐标值
                    Eigen::Vector3d point;
                    point[2] = double(depth_point) / depthScale;
                    point[0] = (u - cx) * point[2] / fx;
                    point[1] = (v - cy) * point[2] / fy;

                    //世界坐标系下坐标值
                    Eigen::Vector3d pointWorld = trans_camera2base * point;

                    //防止畸变产生错误坐标
                    if(pointWorld[2]<0 || pointWorld[2]>5)
                    {
                        point_center[0] = 0;
                        point_center[1] = 0;
                        point_center[2] = 0;
                        point_center[3] = 700.0;
                        point_center[4] = 700.0;
                        continue;
                    }
					
                    point_center[0] = pointWorld[0];
                    point_center[1] = pointWorld[1];
                    point_center[2] = pointWorld[2];
                    point_center[3] = mc[i].x;
                    point_center[4] = mc[i].y;
                    ROS_INFO("point(%d,%d) in realword is (%f, %f, %f)",u, v, pointWorld[0], pointWorld[1], pointWorld[2]);
                }
                if(counter_Inter == contours.size())
                {
                    point_center[0] = 0;
                    point_center[1] = 0;
                    point_center[2] = 0;
                    point_center[3] = 700.0;
                    point_center[4] = 700.0;
                    color = color+1;
                }

        }
            else
            {
                color = color+1;
                point_center[0] = 0;
                point_center[1] = 0;
                point_center[2] = 0;
                point_center[3] = 700.0;
                point_center[4] = 700.0;
            }
        }
        else
        {
            //task = 3;
            color = 1;
            point_center[0] = 0;
            point_center[1] = 0;
            point_center[2] = 0;
            point_center[3] = 700.0;
            point_center[4] = 700.0;
        }
    }
    else
    {
        point_center[0] = 0;
        point_center[1] = 0;
        point_center[2] = 0;
        point_center[3] = 700.0;
        point_center[4] = 700.0;
        num_frame = 49;
    }
    return point_center;
    delete []point_center;
}


void initialize()
{
    ROS_INFO("Initializing...");
    ros::Duration(1.0).sleep();
	
    //rm_msgs::MoveJ_P Initial_Pose;
    Initial_Pose.Pose.position.x = -0.3;//-0.382839;
    Initial_Pose.Pose.position.y = 0;//0.001034;
    Initial_Pose.Pose.position.z = 0.32;//0.210839;
    Initial_Pose.Pose.orientation.x = -0.999668;
    Initial_Pose.Pose.orientation.y = -0.000793;
    Initial_Pose.Pose.orientation.z = 0.025765;
    Initial_Pose.Pose.orientation.w = 0.000019;
    Initial_Pose.speed = 0.3;

    task_step = 6;
    grasp_once = 1;
    moveJ_P_pub.publish(Initial_Pose);
    //ROS_INFO("MoveJ_P success!");
    ros::Duration(3.0).sleep();


    ROS_INFO("Listening transform between camera and base_link...");
    trans_camera2base = get_transpose();



    if(get_trans)
        ROS_INFO("Initialization is complete, and unordered sorting begins...");
    else
    {
        ROS_INFO("There is no transform between camera and base_link!");
    }
}




void callback(const ImageConstPtr& img, const ImageConstPtr& depth)
{
    cv_bridge::CvImagePtr cv_img, cv_depth;
    
    double *point = new double[5];
    
    try
    {
        cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        cv_depth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if(task == 1)
    {
        point = get_center(cv_img->image, cv_depth->image);
        if(point[4] < 690)
        {
            ROS_INFO("grasp_once:%d", grasp_once);
            if(grasp_once == 1)
            {
                grasp_once = 0;
                //ROS_INFO("point_grasp[%f,%f,%f,%f,%f]:",point[0], point[1],point[2], point[3], point[4]);
                pick_place_once(point);
            }
        }
		
    }
    //TCP控制开始分拣
    else if(task == 3)
    {
        //暂停后执行重新开始
        task_step = 6;
        pause_sort = 0;
        grasp_once = 1;
		
        //初始化IO
        rm_msgs::Arm_Digital_Output state;
        state.num = 2;
        state.state = 0;
        ARM_Digital_Output_pub.publish(state);
		
        //初始化位姿、获取坐标系转换
        initialize();
        task = 1;
    }
    else if(task == 4)
    {
        //停止当前任务
        pause_sort = 0;
        task_step = 7;
        pause_sort = 1;
        task = 4;
    }
	

    //显示视频
    imshow(OPENCV_WINDOW, cv_img->image);
    //imwrite("test.jpg", cv_img->image);
    waitKey(1);
    //ROS_INFO("task = %d", task);
    delete []point;

}



int main(int argc, char **argv) 
{

    // 初始化ROS节点
    ros::init(argc, argv, "disorderly_sorting");
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(3);
    spin.start();

    //创建发布节点
    moveL_pub = nh.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);
    moveJ_P_pub = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    changetool_pub = nh.advertise<rm_msgs::ChangeTool_Name>("/rm_driver/ChangeToolName_Cmd", 10);
    ARM_Digital_Output_pub = nh.advertise<rm_msgs::Arm_Digital_Output>("/rm_driver/Arm_Digital_Output", 10);
    
    //控制机械臂连续运动
    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);
    //订阅tcp的控制命令
    ros::Subscriber Com_sub = nh.subscribe("TcpCommand", 10, TcpComCallback);


    //同步处理RGB图与深度图
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(img_sub, depth_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::waitForShutdown();

    return 0;
}
