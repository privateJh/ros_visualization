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
#include "jsk_recognition_msgs/BoundingBoxArray.h"

#include "ref_msgs/GlobalGT.h"
#include "ref_msgs/TrackObject.h"
// #include "vision_msgs/BoundingBox3D.h"
// #include "vision_msgs/create_aabb.h"


#define PI 3.14159265358979323846 /* pi */

class Visualizatoin{
    private:
        ros::NodeHandle nh_;

        tf::TransformBroadcaster broadcast_TF;

        ros::Publisher pub_gv80_marker;
        ros::Publisher pub_kusv_marker;
        ros::Publisher pub_starex_marker;
        ros::Publisher pub_rent_marker;

        ros::Publisher pub_boundingboxes;

        ros::Publisher pub_gv80_string_marker;
        ros::Publisher pub_kusv_string_marker;
        ros::Publisher pub_starex_string_marker;
        ros::Publisher pub_rent_string_marker;

        ros::Publisher pub_egoOdom;
        ros::Publisher pub_targetOdom;
                
        ros::Subscriber sub_gv80;
        ros::Subscriber sub_kusv;
        ros::Subscriber sub_starex;
        ros::Subscriber sub_rent;
        ros::Subscriber sub_track_info;
        
        // Input
        geometry_msgs::Pose gv80;
        geometry_msgs::Pose kusv;
        geometry_msgs::Pose starex;
        geometry_msgs::Pose rent;
        ref_msgs::GlobalGT track_info;

        // Output
        visualization_msgs::Marker gv80_marker_;
        visualization_msgs::Marker gv80_string_marker_;

        visualization_msgs::Marker kusv_marker_;
        visualization_msgs::Marker kusv_string_marker_;
        
        visualization_msgs::Marker starex_marker_;
        visualization_msgs::Marker starex_string_marker_;
        
        visualization_msgs::Marker rent_marker_;
        visualization_msgs::Marker rent_string_marker_;
        
        jsk_recognition_msgs::BoundingBoxArray bounding_boxes_ ;
        
        nav_msgs::Odometry ego_vehicle_odom_;
        nav_msgs::Odometry target_vehicle_odom_;

        // Variables
        bool isGv80Update = false;
        bool isKusvUpdate = false;
        bool isStarexUpdate = false;
        bool isRentUpdate = false;
        bool isTrackInfoUpdate = false;
        // tf
        // tf::Quaternion world2ego_q_;
        tf::Transform world2gv80_transform_;
        tf::Transform world2kusv_transform_;
        tf::Transform world2starex_transform_;
        tf::Transform world2rent_transform_;

    public:
        Visualizatoin();
        ~Visualizatoin();
    public:
        // Callback
        void get_gv80(const geometry_msgs::Pose::ConstPtr& msg);
        void get_kusv(const geometry_msgs::Pose::ConstPtr& msg);
        void get_starex(const geometry_msgs::Pose::ConstPtr& msg);
        void get_rent(const geometry_msgs::Pose::ConstPtr& msg);
        void get_track_info(const ref_msgs::GlobalGT::ConstPtr& msg);
        
        // Make Dae File
        void makeGv80Marker();
        void makeKusvMarker();
        void makeStarexMarker();
        void makeRentMarker();
        void makeBoundingBoxesMarker();
        // Publish
        void publishAllMarker();
        // tf
        void broadcast_all_tf();

        //tf
        void broadcasting_tf();
        void Update_Gv80_TF();
        void Update_Kusv_TF();
        void Update_Starex_TF();
        void Update_Rent_TF();


};



#endif