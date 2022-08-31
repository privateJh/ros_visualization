#include "visualization.h"

Visualizatoin::Visualizatoin(){
  
  pub_gv80_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/gv80",100);
  pub_kusv_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/kusv",100);
  pub_starex_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/starex",100);
  pub_rent_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/rent",100);
  
  pub_gv80_string_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/gv80_name",100);
  pub_kusv_string_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/kusv_name",100);
  pub_starex_string_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/starex_name",100);
  pub_rent_string_marker = nh_.advertise<visualization_msgs::Marker>("/hmi/rent_name",100);
  
  pub_boundingboxes = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/hmi/bounding_boxes",100);

  pub_egoOdom = nh_.advertise<nav_msgs::Odometry>("/hmi/ego_odom",100);  
  pub_targetOdom = nh_.advertise<nav_msgs::Odometry>("/hmi/target_odom",100);  

  sub_gv80 = nh_.subscribe("/gv80",10,&Visualizatoin::get_gv80,this);
  sub_kusv = nh_.subscribe("/kusv",10,&Visualizatoin::get_kusv,this);
  sub_starex = nh_.subscribe("/starex",10,&Visualizatoin::get_starex,this);
  sub_rent = nh_.subscribe("/rent",10,&Visualizatoin::get_rent,this);
  sub_track_info = nh_.subscribe("/track_info", 100, &Visualizatoin::get_track_info, this);

  ego_vehicle_odom_.header.frame_id = "/world";
  target_vehicle_odom_.header.frame_id = "/world";
  // Init State
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  q.normalize();
  
  world2gv80_transform_.setOrigin(tf::Vector3(0.0,0.0, 0.0));
  world2gv80_transform_.setRotation(q);

  world2kusv_transform_.setOrigin(tf::Vector3(3.0,0.0, 0.0));
  world2kusv_transform_.setRotation(q);

  world2starex_transform_.setOrigin(tf::Vector3(6.0,0.0, 0.0));
  world2starex_transform_.setRotation(q);

  world2rent_transform_.setOrigin(tf::Vector3(9.0,0.0, 0.0));
  world2rent_transform_.setRotation(q);

}
Visualizatoin::~Visualizatoin() {}

void Visualizatoin::get_gv80(const geometry_msgs::Pose::ConstPtr& msg){
  gv80 = *msg;
  isGv80Update = true;
}

void Visualizatoin::get_kusv(const geometry_msgs::Pose::ConstPtr& msg){
  kusv = *msg;
  isKusvUpdate = true;
}

void Visualizatoin::get_starex(const geometry_msgs::Pose::ConstPtr& msg){
  starex = *msg;
  isStarexUpdate = true;
}

void Visualizatoin::get_rent(const geometry_msgs::Pose::ConstPtr& msg){
  rent = *msg;
  isRentUpdate = true;
}

void Visualizatoin::get_track_info(const ref_msgs::GlobalGT::ConstPtr& msg){
  track_info = *msg;
  isTrackInfoUpdate = true;
}

void Visualizatoin::Update_Gv80_TF(){
  if(isGv80Update){
    tf::Quaternion q;
    q[0] = gv80.orientation.x;
    q[1] = gv80.orientation.y;
    q[2] = gv80.orientation.z;
    q[3] = gv80.orientation.w;
    world2gv80_transform_.setOrigin(tf::Vector3(gv80.position.x,gv80.position.y, 0.0));
    world2gv80_transform_.setRotation(q);
    ego_vehicle_odom_.header.stamp = ros::Time::now();
    
  }else{
    ROS_WARN("GV80 State Not Updated!");
  }
}

void Visualizatoin::Update_Kusv_TF(){
  if(isKusvUpdate){
    tf::Quaternion q;
    q[0] = kusv.orientation.x;
    q[1] = kusv.orientation.y;
    q[2] = kusv.orientation.z;
    q[3] = kusv.orientation.w;
    world2kusv_transform_.setOrigin(tf::Vector3(kusv.position.x,kusv.position.y, 0.0));
    world2kusv_transform_.setRotation(q);
  }else{
    ROS_WARN("KUSV State Not Updated!");
  }
}

void Visualizatoin::Update_Starex_TF(){
  if(isStarexUpdate){
    tf::Quaternion q;
    q[0] = starex.orientation.x;
    q[1] = starex.orientation.y;
    q[2] = starex.orientation.z;
    q[3] = starex.orientation.w;
    world2starex_transform_.setOrigin(tf::Vector3(starex.position.x,starex.position.y, 0.0));
    world2starex_transform_.setRotation(q);
  }else{
    ROS_WARN("STAREX State Not Updated!");
  }
}

void Visualizatoin::Update_Rent_TF(){
  if(isRentUpdate){
    tf::Quaternion q;
    q[0] = rent.orientation.x;
    q[1] = rent.orientation.y;
    q[2] = rent.orientation.z;
    q[3] = rent.orientation.w;
    world2rent_transform_.setOrigin(tf::Vector3(rent.position.x,rent.position.y, 0.0));
    world2rent_transform_.setRotation(q);
  }else{
    ROS_WARN("RENT State Not Updated!");
  }
}

void Visualizatoin::broadcasting_tf() {
    
    Update_Gv80_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2gv80_transform_, ros::Time::now(), "/world", "/gv80"));

    Update_Kusv_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2kusv_transform_, ros::Time::now(), "/world", "/kusv"));

    Update_Starex_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2starex_transform_, ros::Time::now(), "/world", "/starex"));
    
    Update_Rent_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        world2rent_transform_, ros::Time::now(), "/world", "/rent"));
}

void Visualizatoin::makeGv80Marker(){
    // Update_Ego_TF();
    gv80_marker_.header.frame_id = "/gv80";
    gv80_marker_.header.stamp = ros::Time::now();
    gv80_marker_.id = 0;
    // Set the marker type
    gv80_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    gv80_marker_.mesh_resource =
        "package://visualization/resources/SimpleCar.dae";
    gv80_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // gv80_marker_.pose.position.x = ego_vehicle_.position.x;
    // gv80_marker_.pose.position.y = ego_vehicle_.position.y;
    // gv80_marker_.pose.position.z = ego_vehicle_.position.z;
    // gv80_marker_.pose.orientation.x = ego_vehicle_.orientation.x;
    // gv80_marker_.pose.orientation.y = ego_vehicle_.orientation.y;
    // gv80_marker_.pose.orientation.z = ego_vehicle_.orientation.z;
    // gv80_marker_.pose.orientation.w = ego_vehicle_.orientation.w;

    gv80_marker_.pose.position.x = 0.0;
    gv80_marker_.pose.position.y = 0.0;
    gv80_marker_.pose.position.z = 0.0;
    gv80_marker_.pose.orientation.x = q[0];
    gv80_marker_.pose.orientation.y = q[1];
    gv80_marker_.pose.orientation.z = q[2];
    gv80_marker_.pose.orientation.w = q[3];
    
    // Set the scale of the marker
    gv80_marker_.scale.x = 1.0;
    gv80_marker_.scale.y = 1.0;
    gv80_marker_.scale.z = 1.0;
    // Color Pink
    gv80_marker_.color.r = 0.5882;
    gv80_marker_.color.g = 0.2941;
    gv80_marker_.color.b = 0.0;
    gv80_marker_.color.a = 1.0;

    gv80_marker_.lifetime = ros::Duration(0.1);

    gv80_string_marker_.header.frame_id = "/gv80";
    gv80_string_marker_.header.stamp = ros::Time::now();
    gv80_string_marker_.id = 1;
    gv80_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // gv80_string_marker_.pose.position.x = gv80.position.x;
    // gv80_string_marker_.pose.position.y = gv80.position.y;
    // gv80_string_marker_.pose.position.z = gv80.position.z + 1.0;

    gv80_string_marker_.pose.position.x = 0.0;
    gv80_string_marker_.pose.position.y = 0.0;
    gv80_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    gv80_string_marker_.scale.x = 0.2;
    gv80_string_marker_.scale.y = 0.2;
    gv80_string_marker_.scale.z = 0.2;
    // Color Pink
    gv80_string_marker_.color.r = 1.0;
    gv80_string_marker_.color.g = 1.0;
    gv80_string_marker_.color.b = 1.0;
    gv80_string_marker_.color.a = 1.0;
    gv80_string_marker_.lifetime = ros::Duration(0.1);

    gv80_string_marker_.text = "GV80";

}

void Visualizatoin::makeKusvMarker(){
    // Update_Ego_TF();
    kusv_marker_.header.frame_id = "/kusv";
    kusv_marker_.header.stamp = ros::Time::now();
    kusv_marker_.id = 2;
    // Set the marker type
    kusv_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    kusv_marker_.mesh_resource =
        "package://visualization/resources/SimpleCar.dae";
    kusv_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // kusv_marker_.pose.position.x = kusv.position.x;
    // kusv_marker_.pose.position.y = kusv.position.y;
    // kusv_marker_.pose.position.z = kusv.position.z;
    // kusv_marker_.pose.orientation.x = kusv.orientation.x;
    // kusv_marker_.pose.orientation.y = kusv.orientation.y;
    // kusv_marker_.pose.orientation.z = kusv.orientation.z;
    // kusv_marker_.pose.orientation.w = kusv.orientation.w;

    kusv_marker_.pose.position.x = 0.0;
    kusv_marker_.pose.position.y = 0.0;
    kusv_marker_.pose.position.z = 0.0;
    kusv_marker_.pose.orientation.x = q[0];
    kusv_marker_.pose.orientation.y = q[1];
    kusv_marker_.pose.orientation.z = q[2];
    kusv_marker_.pose.orientation.w = q[3];
    
    // Set the scale of the marker
    kusv_marker_.scale.x = 1.0;
    kusv_marker_.scale.y = 1.0;
    kusv_marker_.scale.z = 1.0;
    // Color Pink
    kusv_marker_.color.r = 0.52;
    kusv_marker_.color.g = 0.52;
    kusv_marker_.color.b = 0.52;
    kusv_marker_.color.a = 1.0;

    kusv_marker_.lifetime = ros::Duration(0.1);

    kusv_string_marker_.header.frame_id = "/kusv";
    kusv_string_marker_.header.stamp = ros::Time::now();
    kusv_string_marker_.id = 3;
    kusv_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // kusv_string_marker_.pose.position.x = kusv.position.x;
    // kusv_string_marker_.pose.position.y = kusv.position.y;
    // kusv_string_marker_.pose.position.z = kusv.position.z + 1.0;

    kusv_string_marker_.pose.position.x = 0.0;
    kusv_string_marker_.pose.position.y = 0.0;
    kusv_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    kusv_string_marker_.scale.x = 0.2;
    kusv_string_marker_.scale.y = 0.2;
    kusv_string_marker_.scale.z = 0.2;
    // Color Pink
    kusv_string_marker_.color.r = 1.0;
    kusv_string_marker_.color.g = 1.0;
    kusv_string_marker_.color.b = 1.0;
    kusv_string_marker_.color.a = 1.0;
    kusv_string_marker_.lifetime = ros::Duration(0.1);

    kusv_string_marker_.text = "KUSV";

}

void Visualizatoin::makeStarexMarker(){
    // Update_Ego_TF();
    starex_marker_.header.frame_id = "/starex";
    starex_marker_.header.stamp = ros::Time::now();
    // starex_marker_.ns = "--KKANBU--";
    starex_marker_.id = 4;
    // Set the marker type
    starex_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    starex_marker_.mesh_resource =
        "package://visualization/resources/SimpleCar.dae";
    starex_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // starex_marker_.pose.position.x = starex.position.x;
    // starex_marker_.pose.position.y = starex.position.y;
    // starex_marker_.pose.position.z = starex.position.z;
    // starex_marker_.pose.orientation.x = starex.orientation.x;
    // starex_marker_.pose.orientation.y = starex.orientation.y;
    // starex_marker_.pose.orientation.z = starex.orientation.z;
    // starex_marker_.pose.orientation.w = starex.orientation.w;

    starex_marker_.pose.position.x = 0.0;
    starex_marker_.pose.position.y = 0.0;
    starex_marker_.pose.position.z = 0.0;
    starex_marker_.pose.orientation.x = q[0];
    starex_marker_.pose.orientation.y = q[1];
    starex_marker_.pose.orientation.z = q[2];
    starex_marker_.pose.orientation.w = q[3];
    
    // Set the scale of the marker
    starex_marker_.scale.x = 1.0;
    starex_marker_.scale.y = 1.0;
    starex_marker_.scale.z = 1.0;
    // Color Pink
    starex_marker_.color.r = 1.0;
    starex_marker_.color.g = 1.0;
    starex_marker_.color.b = 1.5;
    starex_marker_.color.a = 1.0;

    starex_marker_.lifetime = ros::Duration(0.1);

    starex_string_marker_.header.frame_id = "/starex";
    starex_string_marker_.header.stamp = ros::Time::now();
    starex_string_marker_.id = 5;
    starex_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // starex_string_marker_.pose.position.x = starex.position.x;
    // starex_string_marker_.pose.position.y = starex.position.y;
    // starex_string_marker_.pose.position.z = starex.position.z + 1.0;

    starex_string_marker_.pose.position.x = 0.0;
    starex_string_marker_.pose.position.y = 0.0;
    starex_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    starex_string_marker_.scale.x = 0.2;
    starex_string_marker_.scale.y = 0.2;
    starex_string_marker_.scale.z = 0.2;
    // Color Pink
    starex_string_marker_.color.r = 1.0;
    starex_string_marker_.color.g = 1.0;
    starex_string_marker_.color.b = 1.0;
    starex_string_marker_.color.a = 1.0;
    starex_string_marker_.lifetime = ros::Duration(0.1);

    starex_string_marker_.text = "STAREX";

}

void Visualizatoin::makeRentMarker(){
    // Update_Ego_TF();
    rent_marker_.header.frame_id = "/rent";
    rent_marker_.header.stamp = ros::Time::now();
    // rent_marker_.ns = "--KKANBU--";
    rent_marker_.id = 6;
    // Set the marker type
    rent_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    rent_marker_.mesh_resource =
        "package://visualization/resources/SimpleCar.dae";
    rent_marker_.mesh_use_embedded_materials = true;
    tf::Quaternion q;
    q.setRPY(0,0,0);

    // rent_marker_.pose.position.x = rent.position.x;
    // rent_marker_.pose.position.y = rent.position.y;
    // rent_marker_.pose.position.z = rent.position.z;
    // rent_marker_.pose.orientation.x = rent.orientation.x;
    // rent_marker_.pose.orientation.y = rent.orientation.y;
    // rent_marker_.pose.orientation.z = rent.orientation.z;
    // rent_marker_.pose.orientation.w = rent.orientation.w;

    rent_marker_.pose.position.x = 0.0;
    rent_marker_.pose.position.y = 0.0;
    rent_marker_.pose.position.z = 0.0;
    rent_marker_.pose.orientation.x = q[0];
    rent_marker_.pose.orientation.y = q[1];
    rent_marker_.pose.orientation.z = q[2];
    rent_marker_.pose.orientation.w = q[3];
    
    // Set the scale of the marker
    rent_marker_.scale.x = 1.0;
    rent_marker_.scale.y = 1.0;
    rent_marker_.scale.z = 1.0;
    // Color Pink
    rent_marker_.color.r = 0.0;
    rent_marker_.color.g = 0.3373;
    rent_marker_.color.b = 0.4;
    rent_marker_.color.a = 1.0;

    rent_marker_.lifetime = ros::Duration(0.1);

    rent_string_marker_.header.frame_id = "/rent";
    rent_string_marker_.header.stamp = ros::Time::now();
    rent_string_marker_.id = 7;
    rent_string_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // rent_string_marker_.pose.position.x = rent.position.x;
    // rent_string_marker_.pose.position.y = rent.position.y;
    // rent_string_marker_.pose.position.z = rent.position.z + 1.0;

    rent_string_marker_.pose.position.x = 0.0;
    rent_string_marker_.pose.position.y = 0.0;
    rent_string_marker_.pose.position.z = 2.0;

    // Set the scale of the marker
    rent_string_marker_.scale.x = 0.2;
    rent_string_marker_.scale.y = 0.2;
    rent_string_marker_.scale.z = 0.2;
    // Color Pink
    rent_string_marker_.color.r = 1.0;
    rent_string_marker_.color.g = 1.0;
    rent_string_marker_.color.b = 1.0;
    rent_string_marker_.color.a = 1.0;
    rent_string_marker_.lifetime = ros::Duration(0.1);

    rent_string_marker_.text = "RENT";

}

void Visualizatoin::makeBoundingBoxesMarker(){
  bounding_boxes_.header.frame_id = "/gv80";
  for(int i=0;i<track_info.track_objects.size();i++){
    jsk_recognition_msgs::BoundingBox boundingbox;
    tf::Quaternion q_;
    q_.setRPY(track_info.track_objects[i].roll, track_info.track_objects[i].pitch, track_info.track_objects[i].pitch);
    q_.normalize();
    boundingbox.header.frame_id = "/gv80";
    boundingbox.pose.position.x = track_info.track_objects[i].center.x;
    boundingbox.pose.position.y = track_info.track_objects[i].center.y;
    boundingbox.pose.position.z = track_info.track_objects[i].center.z;
    boundingbox.pose.orientation.x = q_[0];
    boundingbox.pose.orientation.y = q_[1];
    boundingbox.pose.orientation.z = q_[2];
    boundingbox.pose.orientation.w = q_[3];
    boundingbox.dimensions.x = track_info.track_objects[i].width;
    boundingbox.dimensions.y = track_info.track_objects[i].length;
    boundingbox.dimensions.z = track_info.track_objects[i].height;
    boundingbox.label = track_info.track_objects[i].header.id;
    bounding_boxes_.boxes.push_back(boundingbox);
  }
}

void Visualizatoin::publishAllMarker(){
    pub_gv80_marker.publish(gv80_marker_);
    pub_gv80_string_marker.publish(gv80_string_marker_);
    
    pub_kusv_marker.publish(kusv_marker_);
    pub_kusv_string_marker.publish(kusv_string_marker_);
    
    pub_starex_marker.publish(starex_marker_);
    pub_starex_string_marker.publish(starex_string_marker_);
    
    pub_rent_marker.publish(rent_marker_);
    pub_rent_string_marker.publish(rent_string_marker_);
    
    pub_boundingboxes.publish(bounding_boxes_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualization");
  // Vehicle vehicle;
  Visualizatoin visualization;

  // 60 Hz visualization
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    visualization.broadcasting_tf();
    visualization.makeGv80Marker();
    visualization.makeKusvMarker();
    visualization.makeStarexMarker();
    visualization.makeRentMarker();
    visualization.makeBoundingBoxesMarker();
    visualization.publishAllMarker();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}