#include <iostream>
#include <fstream>
#include <string>
#include <urdf/model.h>
#include "ros/ros.h"

typedef boost::shared_ptr<const urdf::Joint> MJoint;
typedef boost::shared_ptr<const urdf::Link> MLink;

void indent(std::ostream& s, int level) {
  while(level-- > 0) s << " ";
}

void indent(int level) {
  indent(std::cout, level);
}

void printVector(std::ostream& s, const urdf::Vector3& v) {
  s << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

void printVector(const urdf::Vector3& v) {
  printVector(std::cout, v);
}

void printRPY(std::ostream& s, const urdf::Rotation& r) {
  double roll, pitch, yaw;
  r.getRPY(roll, pitch, yaw);
  s << "(" << roll << ", " << pitch << ", " << yaw << ")";
}

void printRPY(const urdf::Rotation& r) {
  printRPY(std::cout, r);
}

void walkLink(int level, MJoint joint, MLink link) {
  indent(level);
  std::cout << link->name << std::endl;
  if(joint) {
    urdf::Pose p = joint->parent_to_joint_origin_transform;
    indent(level);
    std::cout << "|__ ";
    printVector(p.position);
    std::cout << " ";
    printRPY(p.rotation);
    std::cout << " ";
    printVector(joint->axis);
    std::cout << std::endl;
  }
  for(size_t i = 0; i < link->child_links.size(); i++) {
    walkLink(level + 2, link->child_joints[i], link->child_links[i]);
  }
}

int dumpModel(std::ostream& s, int parent, int index, MJoint joint, MLink link,
              std::vector<std::pair<int,int> >& arcs) {
  s << link->name << ": ";
  if(joint) {
    printVector(s, joint->parent_to_joint_origin_transform.position);
    s << " ";
    printRPY(s, joint->parent_to_joint_origin_transform.rotation);
    s << " ";
    printVector(s, joint->axis);
  }
  else {
    s << "(0, 0, 0) (0, 0, 0) (0, 0, 0)";
  }
  s << std::endl;
  if(parent > -1) arcs.push_back(std::pair<int,int>(parent,index));
  int n = index + 1;
  for(size_t i = 0; i < link->child_links.size(); i++) {
    n += dumpModel(s, index, n, link->child_joints[i], link->child_links[i], arcs);
  }
  return (n - index);
}

// A simple 3D vector structure to demonstrate dump file parsing
// independence from the ROS URDF libraries.
struct Vec3 { 
  double x,y,z;
  Vec3(double x0, double y0, double z0) : 
    x(x0), y(y0), z(z0) {};
};

Vec3 readVector(std::istream& s) {
  double x, y, z;
  s.ignore(256, '(');
  s >> x;
  s.ignore(256, ',');
  s >> y;
  s.ignore(256, ',');
  s >> z;
  s.ignore(256, ')');
  return Vec3(x,y,z);
}

// Parse our flat text model format. See README for definition.
void readDump(const char* fileName) {
  std::ifstream f(fileName, std::ios_base::in);
  int n;
  f >> std::dec >> n;
  f.ignore(256, '\n');

  // Read in named links
  std::cout << "Reading in " << n << " links" << std::endl;
  for(int i = 0; i < n; i++) {
    std::string s;
    getline(f, s, ':');
    std::cout << "Link " << s << " ";
    Vec3 translation = readVector(f);
    printVector(urdf::Vector3(translation.x, translation.y, translation.z));
    Vec3 rpy = readVector(f);
    urdf::Rotation rotation;
    rotation.setFromRPY(rpy.x, rpy.y, rpy.z);
    std::cout << " ";
    printRPY(rotation);
    Vec3 axis = readVector(f);
    std::cout << " ";
    printVector(urdf::Vector3(axis.x, axis.y, axis.z));
    std::cout << std::endl;
    f.ignore(256, '\n');
  }

  // Read in robot model graph arcs
  while(1) {
    int src, dst;
    f >> src;
    f.ignore(256, '>');
    f >> dst;
    f.ignore(256, '\n');
    std::cout << "Arc from " << src << " to " << dst << std::endl;
    if(f.peek() == EOF) break;
  }
  f.close();
}

/* When given just a URDF file as an argument, prints a tree-like
 * display of a URDF hierarchy. When given two arguments, the second
 * is treated as an output file to which a simplified model format is
 * written. */
int main(int argc, char** argv) {
  ros::init(argc, argv, "my_parser");
  if(argc < 2) {
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];
  urdf::Model model;
  if(!model.initFile(urdf_file)) {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  if(argc == 2) walkLink(0, MJoint(), model.getRoot());
  else if(argc == 3) {
    std::ofstream f(argv[2], std::ios_base::out | std::ios_base::trunc);
    f << model.links_.size() << std::endl;
    std::vector<std::pair<int,int> > arcs = std::vector<std::pair<int,int> >();
    dumpModel(f, -1, 0, MJoint(), model.getRoot(), arcs);
    for(size_t i = 0; i < arcs.size(); i++) {
      f << arcs[i].first << " -> " << arcs[i].second << std::endl;
    }
    f.close();

    // Test dump file parsing
    readDump(argv[2]);
  }
  return 0;
}
