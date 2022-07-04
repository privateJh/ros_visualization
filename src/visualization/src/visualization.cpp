#include "visualization.h"

Visualizatoin::Visualizatoin(){
  
  pub_vehicleModelMarker = nh_.advertise<visualization_msgs::Marker>("/hmi/ego_model",100);
  pub_targetVehicleModelMarker = nh_.advertise<visualization_msgs::Marker>("/hmi/target_model",100);
  pub_vehicleStringMarker = nh_.advertise<visualization_msgs::Marker>("/hmi/ego_model_name",100);
  pub_targetVehicleStringMarker = nh_.advertise<visualization_msgs::Marker>("/hmi/target_model_name",100);  
  pub_targetBoundingBox = nh_.advertise<jsk_recognition_msgs::BoundingBox>("/hmi/target_bounding_box",100);

  pub_egoOdom = nh_.advertise<nav_msgs::Odometry>("/hmi/ego_odom",100);  
  pub_targetOdom = nh_.advertise<nav_msgs::Odometry>("/hmi/target_odom",100);  

  sub_egoVehicle = nh_.subscribe("/ego_vehicle",10,&Visualizatoin::get_egoVehicle,this);
  sub_targetVehicle = nh_.subscribe("/target_vehicle",10,&Visualizatoin::get_targetVehicle,this);


  nh_.param("visualization/ego2lidar_x", ego2lidar_x, -0.5);
  nh_.param("visualization/ego2lidar_y", ego2lidar_y, 0.0);
  nh_.param("visualization/ego2lidar_z", ego2lidar_z, 2.0);

  ego_vehicle_odom_.header.frame_id = "/world";
  target_vehicle_odom_.header.frame_id = "/world";
  // Init State
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  q.normalize();
  
  world2ego_transform_.setOrigin(tf::Vector3(0.0,0.0, 0.0));
  world2ego_transform_.setRotation(q);

  world2target_transform_.setOrigin(tf::Vector3(5.0,2.0, 0.0));
  world2target_transform_.setRotation(q);
  
  ego2lidar_transform_.setOrigin(tf::Vector3(ego2lidar_x,ego2lidar_y, ego2lidar_z));
  ego2lidar_transform_.setRotation(q);

}
Visualizatoin::~Visualizatoin() {}

void Visualizatoin::get_egoVehicle(const geometry_msgs::Pose::ConstPtr& msg){
  ego_vehicle_ = *msg;
  isEgoVehicleUpdate = true;
}

void Visualizatoin::get_targetVehicle(const geometry_msgs::Pose::ConstPtr& msg){
  target_vehicle_ = *msg;
  isTargetVehicleUpdate = true;
}

void Visualizatoin::Update_Ego_TF(){
  if(isEgoVehicleUpdate){
    tf::Quaternion q;
    q[0] = ego_vehicle_.orientation.x;
    q[1] = ego_vehicle_.orientation.y;
    q[2] = ego_vehicle_.orientation.z;
    q[3] = ego_vehicle_.orientation.w;
    world2ego_transform_.setOrigin(tf::Vector3(ego_vehicle_.position.x,ego_vehicle_.position.y, 0.0));
    world2ego_transform_.setRotation(q);
    ego_vehicle_odom_.header.stamp = ros::Time::now();
    
  }else{
    ROS_WARN("Ego Vehicle State Not Updated!");
  }
}

void Visualizatoin::Update_Target_TF(){
  if(isTargetVehicleUpdate){
    tf::Quaternion q;
    q[0] = target_vehicle_.orientation.x;
    q[1] = target_vehicle_.orientation.y;
    q[2] = target_vehicle_.orientation.z;
    q[3] = target_vehicle_.orientation.w;
    world2target_transform_.setOrigin(tf::Vector3(target_vehicle_.position.x,target_vehicle_.position.y, 0.0));
    world2target_transform_.setRotation(q);
  }else{
    ROS_ERROR("Target Vehicle State Not Updated!");
  }
}

void Visualizatoin::broadcasting_tf() {
    
    Update_Ego_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2ego_transform_, ros::Time::now(), "/world", "/ego"));

    Update_Target_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2target_transform_, ros::Time::now(), "/world", "/target"));

    broadcast_TF.sendTransform(tf::StampedTransform(
        ego2lidar_transform_, ros::Time::now(), "/ego", "/lidar"));
}

void Visualizatoin::makeEgoVehicleModelMarker(){
    // Update_Ego_TF();
    vehicle_model_marker_.header.frame_id = "/ego";
    vehicle_model_marker_.header.stamp = ros::Time::now();
    // vehicle_model_marker_.ns = "--KKANBU--";
    vehicle_model_marker_.id = 0;
    // Set the marker type
    vehicle_model_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    vehicle_model_marker_.mesh_resource =
        "package://visualization/resources/SimpleCar.dae";
    vehicle_model_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,M_PI/2);
    q.normalize();
    // vehicle_model_marker_.pose.position.x = ego_vehicle_.position.x;
    // vehicle_model_marker_.pose.position.y = ego_vehicle_.position.y;
    // vehicle_model_marker_.pose.position.z = ego_vehicle_.position.z;
    // vehicle_model_marker_.pose.orientation.x = ego_vehicle_.orientation.x;
    // vehicle_model_marker_.pose.orientation.y = ego_vehicle_.orientation.y;
    // vehicle_model_marker_.pose.orientation.z = ego_vehicle_.orientation.z;
    // vehicle_model_marker_.pose.orientation.w = ego_vehicle_.orientation.w;

    vehicle_model_marker_.pose.position.x = -0.5;
    vehicle_model_marker_.pose.position.y = 0.0;
    vehicle_model_marker_.pose.position.z = 0.0;
    vehicle_model_marker_.pose.orientation.x = q[0];
    vehicle_model_marker_.pose.orientation.y = q[1];
    vehicle_model_marker_.pose.orientation.z = q[2];
    vehicle_model_marker_.pose.orientation.w = q[3];
    
    // Set the scale of the marker
    vehicle_model_marker_.scale.x = 1.0;
    vehicle_model_marker_.scale.y = 1.0;
    vehicle_model_marker_.scale.z = 1.0;
    // Color Pink
    vehicle_model_marker_.color.r = 1.0;
    vehicle_model_marker_.color.g = 0.3;
    vehicle_model_marker_.color.b = 0.5;
    vehicle_model_marker_.color.a = 1.0;

    vehicle_model_marker_.lifetime = ros::Duration(0.1);

    vehicle_string_marker_.header.frame_id = "/ego";
    vehicle_string_marker_.header.stamp = ros::Time::now();
    vehicle_string_marker_.id = 2;
    vehicle_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // vehicle_string_marker_.pose.position.x = ego_vehicle_.position.x;
    // vehicle_string_marker_.pose.position.y = ego_vehicle_.position.y;
    // vehicle_string_marker_.pose.position.z = ego_vehicle_.position.z + 1.0;

    vehicle_string_marker_.pose.position.x = 0.0;
    vehicle_string_marker_.pose.position.y = 0.0;
    vehicle_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    vehicle_string_marker_.scale.x = 0.2;
    vehicle_string_marker_.scale.y = 0.2;
    vehicle_string_marker_.scale.z = 0.2;
    // Color Pink
    vehicle_string_marker_.color.r = 1.0;
    vehicle_string_marker_.color.g = 1.0;
    vehicle_string_marker_.color.b = 1.0;
    vehicle_string_marker_.color.a = 1.0;
    vehicle_string_marker_.lifetime = ros::Duration(0.1);

    vehicle_string_marker_.text = "Ego";

}

void Visualizatoin::makeTargetVehicleModelMarker(){
    
    target_vehicle_model_marker_.header.frame_id = "/target";
    target_vehicle_model_marker_.header.stamp = ros::Time::now();
    // target_vehicle_model_marker_.ns = "--KKANBU--";
    target_vehicle_model_marker_.id = 1;
    // Set the marker type
    // target_vehicle_model_marker_.type = visualization_msgs::Marker::CUBE;
    
    target_vehicle_model_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    target_vehicle_model_marker_.mesh_resource =
        "package://visualization/resources/evoque_new.dae";
    target_vehicle_model_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,M_PI/2);
    q.normalize();
    // target_vehicle_model_marker_.pose.position.x = target_vehicle_.position.x;
    // target_vehicle_model_marker_.pose.position.y = target_vehicle_.position.y;
    // target_vehicle_model_marker_.pose.position.z = target_vehicle_.position.z;
    // target_vehicle_model_marker_.pose.orientation.x = target_vehicle_.orientation.x;
    // target_vehicle_model_marker_.pose.orientation.y = target_vehicle_.orientation.y;
    // target_vehicle_model_marker_.pose.orientation.z = target_vehicle_.orientation.z;
    // target_vehicle_model_marker_.pose.orientation.w = target_vehicle_.orientation.w;
    
    target_vehicle_model_marker_.pose.position.x = 3.0;
    // target_vehicle_model_marker_.pose.position.y = 0.0;
    target_vehicle_model_marker_.pose.position.y = 0.0;
    target_vehicle_model_marker_.pose.position.z = 0.0;
    target_vehicle_model_marker_.pose.orientation.x = q[0];
    target_vehicle_model_marker_.pose.orientation.y = q[1];
    target_vehicle_model_marker_.pose.orientation.z = q[2];
    target_vehicle_model_marker_.pose.orientation.w = q[3];

    // Set the scale of the marker
    target_vehicle_model_marker_.scale.x = 1.0;
    target_vehicle_model_marker_.scale.y = 1.0;
    target_vehicle_model_marker_.scale.z = 1.0;
    // Color Pink
    target_vehicle_model_marker_.color.r = 0.0;
    target_vehicle_model_marker_.color.g = 0.3;
    target_vehicle_model_marker_.color.b = 0.5;
    target_vehicle_model_marker_.color.a = 1.0;

    target_vehicle_model_marker_.lifetime = ros::Duration(0.1);

    target_vehicle_string_marker_.header.frame_id = "/target";
    target_vehicle_string_marker_.header.stamp = ros::Time::now();
    target_vehicle_string_marker_.id = 2;
    target_vehicle_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // target_vehicle_string_marker_.pose.position.x = target_vehicle_.position.x;
    // target_vehicle_string_marker_.pose.posit0ion.y = target_vehicle_.position.y;
    // target_vehicle_string_marker_.pose.position.z = target_vehicle_.position.z + 1.0;

    target_vehicle_string_marker_.pose.position.x = 0.0;
    target_vehicle_string_marker_.pose.position.y = 0.0;
    target_vehicle_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    target_vehicle_string_marker_.scale.x = 0.2;
    target_vehicle_string_marker_.scale.y = 0.2;
    target_vehicle_string_marker_.scale.z = 0.2;
    // Color Pink
    target_vehicle_string_marker_.color.r = 1.0;
    target_vehicle_string_marker_.color.g = 1.0;
    target_vehicle_string_marker_.color.b = 1.0;
    target_vehicle_string_marker_.color.a = 1.0;
    target_vehicle_string_marker_.lifetime = ros::Duration(0.1);

    target_vehicle_string_marker_.text = "Target";

    // target_bounding_box_.pose = target_vehicle_;
    tf::Quaternion q_;
    q_.setRPY(0, 0, 0);
    q_.normalize();
    target_bounding_box_.header.frame_id = "/target";
    target_bounding_box_.pose.position.x = 1.7;
    target_bounding_box_.pose.position.y = 0.0;
    target_bounding_box_.pose.position.z = 1.0;
    target_bounding_box_.pose.orientation.x = q_[0];
    target_bounding_box_.pose.orientation.y = q_[1];
    target_bounding_box_.pose.orientation.z = q_[2];
    target_bounding_box_.pose.orientation.w = q_[3];
    target_bounding_box_.dimensions.x = 5.0;
    target_bounding_box_.dimensions.y = 2.0;
    target_bounding_box_.dimensions.z = 2.0;
    

}

void Visualizatoin::publishAllMarker(){
    pub_vehicleModelMarker.publish(vehicle_model_marker_);
    pub_targetVehicleModelMarker.publish(target_vehicle_model_marker_);
    pub_vehicleStringMarker.publish(vehicle_string_marker_);
    pub_targetVehicleStringMarker.publish(target_vehicle_string_marker_);
    pub_targetBoundingBox.publish(target_bounding_box_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualization");
  // Vehicle vehicle;
  Visualizatoin visualization;

  // 60 Hz visualization
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    visualization.broadcasting_tf();
    visualization.makeEgoVehicleModelMarker();
    visualization.makeTargetVehicleModelMarker();
    visualization.publishAllMarker();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}