//  AprilTag YAML encoding.
//  Added 21/07/2014 by gareth.

#include "tag_yaml.hpp"


void operator >> (const YAML::Node &node, apriltag_ros::Tag &rhs) {
  
  node["id"] >> rhs.id;
  
  const YAML::Node& pointSeq = node["corners"];
  
  //  this will trigger an exception if any points are invalid...
  for (int i=0; i < 4; i++) {
    pointSeq[i]["x"] >> rhs.p[i].x;
    pointSeq[i]["y"] >> rhs.p[i].y;
    pointSeq[i]["z"] >> rhs.p[i].z;
  }
  
  return;
}

YAML::Emitter& operator << (YAML::Emitter& out, const apriltag_ros::Tag& tag) {  
  out << YAML::Block;
  out << YAML::BeginMap;
  out << YAML::Key << "id" << YAML::Value << tag.id;
  out << YAML::Key << "corners" << YAML::Value;
  out << YAML::BeginSeq;
  for (int i=0; i < 4; i++) {
    out << YAML::Flow;
    out << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << tag.p[i].x;
    out << YAML::Key << "y" << YAML::Value << tag.p[i].y;
    out << YAML::Key << "z" << YAML::Value << tag.p[i].z;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}
