#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

tf::Transform transform;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


bool computeMatrix(PointCloud::Ptr target,
                   PointCloud::Ptr world,
                   std::string target_name,
                   std::string world_name,
                   const bool broadcast)
{
    if ((!world_name.empty()) && (!target_name.empty()) &&
            (target->points.size() > 2) && (world->points.size() == target->points.size()))
    {
        Eigen::Matrix4f trMatrix;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;

        svd.estimateRigidTransformation(*target, *world, trMatrix);

        ROS_INFO("Registration completed and Registration Matrix is being broadcasted");

         transform=tf::Transform(tf::Matrix3x3(trMatrix(0, 0), trMatrix(0, 1), trMatrix(0, 2),
                                              trMatrix(1, 0), trMatrix(1, 1), trMatrix(1, 2),
                                              trMatrix(2, 0), trMatrix(2, 1), trMatrix(2, 2)),
                                tf::Vector3(trMatrix(0, 3), trMatrix(1, 3), trMatrix(2, 3)));

        Eigen::Vector3d origin(transform.getOrigin());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	std::cout << std::endl << "#################################################" << std::endl; 
	std::cout << std::endl << "########### TRANSFORMATION PARAMETERS ###########" << std::endl; 
	std::cout << std::endl << "#################################################" << std::endl; 
        std::cout << "origin: "<<origin.transpose() << std::endl;
        std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;


    }

    return true;
}




int main(int argc, char *argv[])
{
    ros::init (argc, argv, "marker_detect");
    ros::NodeHandle n;

    ros::NodeHandle n_priv("~");
    double min_x,min_y,min_z;
    double max_x,max_y,max_z;
    double roll_angle_range, pitch_angle_range, yaw_angle_range;
    double roll_angle_offset, pitch_angle_offset, yaw_angle_offset;
    int number_of_points;

    std::string marker_link;
    std::string end_effector_link;
    std::string camera_link;
    std::string arm_base_link;

    n_priv.param<int>("number_of_points",number_of_points, 6);

    n_priv.param<double>("roll_angle_range",    roll_angle_range,  1.2);
    n_priv.param<double>("roll_angle_offset",   roll_angle_offset,  0.6);
    n_priv.param<double>("pitch_angle_range",   pitch_angle_range, 1.0);
    n_priv.param<double>("pitch_angle_offset",  pitch_angle_offset,  0.65);
    n_priv.param<double>("yaw_angle_range",     yaw_angle_range,   -1.2);
    n_priv.param<double>("yaw_angle_offset",    yaw_angle_offset,  0.1);

    n_priv.param<double>("min_x",min_x, 0.4);
    n_priv.param<double>("max_x",max_x, 1.8);

    n_priv.param<double>("min_y",min_y, -1.3);
    n_priv.param<double>("max_y",max_y, 1.3);

    n_priv.param<double>("min_z",min_z, 0.3);
    n_priv.param<double>("max_z",max_z, 1.4);

    n_priv.param<std::string>("arm_base_link",arm_base_link, "arm_base_link");
    n_priv.param<std::string>("camera_link",camera_link, "camera_link");
    n_priv.param<std::string>("end_effector",end_effector_link, "end_effector");
    n_priv.param<std::string>("marker_link",marker_link, "ar_marker_4");


    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("arm");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/planning_scene", 1, true);

    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.

    group.setPoseReferenceFrame(arm_base_link);
    //std::cout << group.getPlanningFrame() << std::endl;
    group.setEndEffectorLink(end_effector_link);


    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group.setWorkspace(min_x,min_y,min_z,max_x,max_y,max_z);
    

    group.setGoalTolerance(0.1);

     //**************************************new part ************************************ 
   // specify that our target will be a fixed one
    geometry_msgs::Pose fixed_pose;
    //fixed_pose.header.frame_id=arm_base_link;
    
    // to get the model frame name
    //random_pose_.header.frame_id=group.getCurrentState()->getRobotModel()->getModelFrame();
    //std::cout << "MODEL Frame: "<<random_pose_.header.frame_id<<std::endl;
      
    std::cout << "get end effector link:"<< group.getEndEffectorLink()<<std::endl;
    PointCloud::Ptr arm_cloud(new PointCloud);
    PointCloud::Ptr camera_cloud(new PointCloud);
    float positionx[8]={0.84207,0.82051,0.77428,0.82044,0.78233,0.80127,0.84587,0.90135};
    float positiony[8]={0.016322,0.14474, 0.24908,-0.14473,-0.23492,-0.2452,0.1348,-0.14999};
    float positionz[8]={0.2885,0.28846,0.28843,0.28843,0.28845,0.24544,0.2455,0.28731};
    float quatx[8]={0.032703, -0.061438, -0.14309,-0.14939,-0.21778,0.17326,0.095717,0.090634};
    float quaty[8]={0.70635,0.70443,0.69248,-0.69115,-0.67273, 0.68555,-0.7006,0.70127};
    float quatz[8]={-0.70487,-0.69114,-0.66903,0.70443,0.69476, -0.68468,0.70106,-0.70036};
    float quatw[8]={0.056222,0.14944,0.22892,0.061501,0.13154, -0.17666,-0.092313,-0.097434};
    int j=0;
    while(ros::ok() &&
          camera_cloud->points.size()!=number_of_points &&
          arm_cloud->points.size()!=number_of_points && j<8)
    {
        ROS_INFO_STREAM("Samples acquired so far: "<<camera_cloud->points.size()<< " out of "<<number_of_points);

	// Keep trying to generate possible end effector pose
	 
	//fixed positions

	     fixed_pose.position.x =  positionx[j];
	     fixed_pose.position.y =  positiony[j];
	     fixed_pose.position.z =  positionz[j];
	     geometry_msgs::Quaternion quat_msg;
	     quat_msg.x =  quatx[j];
	     quat_msg.y =  quaty[j];
	     quat_msg.z =  quatz[j];
	     quat_msg.w =  quatw[j];
	     fixed_pose.orientation = quat_msg;
	     group.setJointValueTarget(fixed_pose);
             std::cout << fixed_pose.position << std::endl;
	     sleep(5.0);
	     group.move();
	      
        tf::TransformListener listener;
        // Get some point correspondences
        try
        {
            /////////////////////////////////////
            // Point in the end effector frame //
            /////////////////////////////////////

            tf::StampedTransform marker_end_effector_tf;
            listener.waitForTransform(arm_base_link, end_effector_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(arm_base_link, end_effector_link, ros::Time(0), marker_end_effector_tf);

            ///////////////////////////
            // Point in camera frame //
            ///////////////////////////

            tf::StampedTransform marker_camera_tf;
            listener.waitForTransform(camera_link, marker_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(camera_link, marker_link, ros::Time(0), marker_camera_tf);

            PointT arm_point;
            arm_point.x=marker_end_effector_tf.getOrigin().x();
            arm_point.y=marker_end_effector_tf.getOrigin().y();
            arm_point.z=marker_end_effector_tf.getOrigin().z();
            arm_cloud->points.push_back(arm_point);

            //            std::cout << "arm_point:" << arm_point  << std::endl;

            PointT camera_point;
            camera_point.x=marker_camera_tf.getOrigin().x();
            camera_point.y=marker_camera_tf.getOrigin().y();
            camera_point.z=marker_camera_tf.getOrigin().z();
            camera_cloud->points.push_back(camera_point);

            //            std::cout << "camera_point:" << camera_point  << std::endl;

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        j++;
    }



    PointCloud::Ptr source, target;
    Eigen::Matrix4f pairTransform;

    source = arm_cloud;
    target = camera_cloud;

    // Add visualization data
    //showCloudsLeft(source,  target);

    computeMatrix(target,
                      source,
                      camera_link,
                      arm_base_link,
                      true);
    ros::Rate r(100.0);
    while(ros::ok)
    {


            static tf::TransformBroadcaster br;
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                  arm_base_link, camera_link));


	r.sleep();
    }

    return 1;
}


