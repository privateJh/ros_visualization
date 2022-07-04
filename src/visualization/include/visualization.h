#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"

#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "jsk_recognition_msgs/BoundingBox.h"
// #include "vision_msgs/BoundingBox3D.h"
// #include "vision_msgs/create_aabb.h"


#define PI 3.14159265358979323846 /* pi */

class Visualizatoin{
    private:
        ros::NodeHandle nh_;

        tf::TransformBroadcaster broadcast_TF;

        ros::Publisher pub_vehicleModelMarker;
        ros::Publisher pub_targetVehicleModelMarker;

        ros::Publisher pub_targetBoundingBox;

        ros::Publisher pub_vehicleStringMarker;
        ros::Publisher pub_targetVehicleStringMarker;

        ros::Publisher pub_egoOdom;
        ros::Publisher pub_targetOdom;
                
        ros::Subscriber sub_egoVehicle;
        ros::Subscriber sub_targetVehicle;
        
        // Input
        geometry_msgs::Pose ego_vehicle_;
        geometry_msgs::Pose target_vehicle_;

        // Output
        visualization_msgs::Marker vehicle_model_marker_;
        visualization_msgs::Marker target_vehicle_model_marker_;
        visualization_msgs::Marker vehicle_string_marker_;
        visualization_msgs::Marker target_vehicle_string_marker_;
        jsk_recognition_msgs::BoundingBox target_bounding_box_ ;
        nav_msgs::Odometry ego_vehicle_odom_;
        nav_msgs::Odometry target_vehicle_odom_;

        // Variables
        bool isEgoVehicleUpdate = false;
        bool isTargetVehicleUpdate = false;
        // tf
        // tf::Quaternion world2ego_q_;
        tf::Transform world2ego_transform_;
        tf::Transform world2target_transform_;
        tf::Transform ego2lidar_transform_;

    public:
        Visualizatoin();
        ~Visualizatoin();
    public:
        // Callback
        void get_egoVehicle(const geometry_msgs::Pose::ConstPtr& msg);
        void get_targetVehicle(const geometry_msgs::Pose::ConstPtr& msg);
        // Make Dae File
        void makeEgoVehicleModelMarker();
        void makeTargetVehicleModelMarker();
        // Publish
        void publishAllMarker();
        // tf
        void broadcast_all_tf();

        //tf
        void broadcasting_tf();
        void Update_Ego_TF();
        void Update_Target_TF();


};



#endif