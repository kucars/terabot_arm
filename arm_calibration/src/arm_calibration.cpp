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
    n_priv.param<std::string>("marker_link",marker_link, "ar_marker_0");


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
    
    float positionx[13]={0.82633,0.81688,0.79482,0.82561,0.85233,0.85823, 0.84924,0.84145,0.80744, 0.83643,0.91167,0.76203,0.92631};
    float positiony[13]={-5.4058e-06,-0.09412, -0.16984,-0.18168,-0.084054,0.033804,0.10065,0.1336,0.12433,0.16757,-0.0374,-0.12664,0.1364};
    float positionz[13]={0.24769,0.24767,0.24769,0.29213,0.29221,0.2922,0.29219, 0.29219,0.26394,0.28019,0.33795,0.29291, 0.35577};
    
    float quatx[13]={-0.071466, -0.00091693, 0.057936,0.10371,0.03338, 0.049068,0.095922,0.1192,0.22109,-0.095911, -0.14986,0.02758,-0.047177};  
    float quaty[13]={0.70349,0.70711, 0.70473,0.69946,0.70632,-0.7054,-0.70057,-0.69699,-0.67165,0.70057,-0.69104,0.70657,0.70553};
    float quatz[13]={-0.70349,-0.69283,-0.67866,-0.69015,-0.70207,0.7071,0.70567, 0.70376,0.70618,-0.69339,0.6995,-0.68318,-0.69732};
    float quatw[13]={-0.071474,-0.14136,-0.19853,-0.15393,-0.084216,0.001914,-0.045118,-0.068728,0.036208,0.13859,-0.10341,-0.1824,0.11726};
    
  /*  
    float positionx[8]={0.84207,0.82051,0.77428,0.82044,0.78233,0.80127,0.84587,0.90135};
    float positiony[8]={0.016322,0.14474, 0.24908,-0.14473,-0.23492,-0.2452,0.1348,-0.14999};
    float positionz[8]={0.2885,0.28846,0.28843,0.28843,0.28845,0.24544,0.2455,0.28731};
    float quatx[8]={0.032703, -0.061438, -0.14309,-0.14939,-0.21778,0.17326,0.095717,0.090634};
    float quaty[8]={0.70635,0.70443,0.69248,-0.69115,-0.67273, 0.68555,-0.7006,0.70127};
    float quatz[8]={-0.70487,-0.69114,-0.66903,0.70443,0.69476, -0.68468,0.70106,-0.70036};
    float quatw[8]={0.056222,0.14944,0.22892,0.061501,0.13154, -0.17666,-0.092313,-0.097434};*/
    int j=0;
    while(ros::ok() &&
          camera_cloud->points.size()!=number_of_points &&
          arm_cloud->points.size()!=number_of_points && j<13)
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
	     sleep(1);
	     group.move();
	     sleep(5);
	     //ros::Duration(5).sleep();	     	      
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

    return 0;
}


