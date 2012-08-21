#include <iostream>
#include <fstream>
#include <string>
#include <urdf/model.h>
#include "ros/ros.h"
#include <vector>
#include <tinyxml.h>

typedef boost::shared_ptr<const urdf::Joint> MJoint;
typedef boost::shared_ptr<const urdf::Link> MLink;

struct Vec3 {
  double x,y,z;
  Vec3(){};
  Vec3(double x0, double y0, double z0) :
    x(x0), y(y0), z(z0) {};
};

struct Triangle {
  Vec3 pts[3];
  Triangle(Vec3 a, Vec3 b, Vec3 c){
    pts[0]=a; pts[1]=b; pts[2]=c;
  }
};

void checkNotNull(void * ptr, char * name){
  if(!ptr){
    std::cout<<"The pointer "<<name<<" is null."<<std::endl;
    exit(0);
  }
}

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

void daeToStl(char * filename){
  TiXmlDocument doc(filename);
  filename[strlen(filename)-3] = 's';
  filename[strlen(filename)-2] = 't';
  filename[strlen(filename)-1] = 'l';
  std::ofstream stlfile((const char *)filename, std::ios::binary|std::ios::trunc);
  doc.LoadFile();
  
  TiXmlElement * lgeo=doc.RootElement()->FirstChildElement("library_geometries");
  
  
  checkNotNull((void *)lgeo, "lgeo");
  TiXmlElement * geo=lgeo->FirstChildElement("geometry");
  checkNotNull((void *)geo, "geo");
  TiXmlElement * mesh=geo->FirstChildElement("mesh");
  checkNotNull((void *)mesh, "mesh");
  TiXmlElement * xml_vertices=mesh->FirstChildElement("vertices");
  checkNotNull((void *)xml_vertices, "xml_vertices");
  TiXmlElement * input = xml_vertices->FirstChildElement("input");
  checkNotNull((void *)input, "input");

  const char * position_name=0;
  const char * position_char_data=0;
  const char * p_char_data;

  std::vector<int> p_vector;
  std::vector<double> position_char_vector;
  std::vector<Vec3> vec3_vector;
  std::vector<Triangle> triangle_vector;
  std::vector<int> indexes;


  
  while(!position_name){
    if(strcmp(input->Attribute("semantic"), "POSITION")==0){
      position_name = input->Attribute("source");
    }
    input=input->NextSiblingElement("input");
    checkNotNull(input, "'input in loop'");
  }
 
  position_name+=1;
  std::cout<<"The position name is "<<position_name<<std::endl;
  TiXmlElement * source=mesh->FirstChildElement("source");
  
  checkNotNull((void *)source, "source pointer");
 
  while(!position_char_data){
    if(strcmp(source->Attribute("id"), position_name)==0){
      position_char_data=source->FirstChildElement("float_array")->GetText();
      break;
    }
    source=source->NextSiblingElement("source");
    checkNotNull((void *)source, "source ptr in loop");
  }
  
  std::cout<<"Got the position_char_data"<<std::endl;

  TiXmlElement * triangles = mesh->FirstChildElement("triangles");
  checkNotNull((void *)triangles, "triangles");
  TiXmlElement * p = triangles->FirstChildElement("p");
  checkNotNull((void *)p, "p");
  TiXmlElement * t_input = triangles->FirstChildElement("input");
  checkNotNull((void *)t_input, "t_input");

  int num_inputs=0;
  int offset=0;
  int num_triangles=0;
  source->FirstChildElement("float_array")->QueryIntAttribute("count", &num_triangles);
  num_triangles/=3;
    
  while(t_input){
    num_inputs++;
    if(strcmp(t_input->Attribute("semantic"), "VERTEX")==0){
      offset=num_inputs-1;
    }
    t_input=t_input->NextSiblingElement("input");
  }
  
  std::cout<<"num_triangles: "<<num_triangles<<std::endl;
  std::cout<<"offset: "<<offset<<std::endl;
  std::cout<<"num_inputs: "<<num_inputs<<std::endl;


  p_char_data=p->GetText();
  
  std::istringstream pos(position_char_data);
  std::istringstream pd(p_char_data);

  std::copy(std::istream_iterator<double>(pos),
	    std::istream_iterator<double>(),
	    std::back_inserter<std::vector<double> >(position_char_vector));

  std::copy(std::istream_iterator<int>(pd),
	    std::istream_iterator<int>(),
	    std::back_inserter<std::vector<int> >(p_vector));
  
  for(int i = 0; i < position_char_vector.size()/3; i++){
    vec3_vector.push_back(Vec3(position_char_vector[3*i],position_char_vector[3*i+1],position_char_vector[3*i+2]));
  }
  
  for(int i = 0; i < p_vector.size()/num_inputs; i++){
    indexes.push_back(p_vector[num_inputs*i + offset]);
  }

  int count = 0;
  for(int i = 0; i < num_triangles;i++){
    triangle_vector.push_back(Triangle(vec3_vector[indexes[count]], vec3_vector[indexes[count+1]], vec3_vector[indexes[count+2]]));
    count+=3;
  }
  /*
  for(int i = 0; i < triangle_vector.size(); i++){
    std::cout<<"----Triangle "<<i<<"----"<<std::endl;
    std::cout<<triangle_vector[i].pts[0].x<<" "<<triangle_vector[i].pts[0].y<<" "<<triangle_vector[i].pts[0].z<<std::endl;
    std::cout<<triangle_vector[i].pts[1].x<<" "<<triangle_vector[i].pts[1].y<<" "<<triangle_vector[i].pts[1].z<<std::endl;
    std::cout<<triangle_vector[i].pts[2].x<<" "<<triangle_vector[i].pts[2].y<<" "<<triangle_vector[i].pts[2].z<<std::endl;
    std::cout<<"-------------------"<<std::endl;
  }
  exit(0);*/
  char c = (char)0x00;

  /*Write the header*/
  for(int i = 0; i < 80; i++){
    stlfile<<c;
  }
  stlfile.write((const char *)(&num_triangles), 4);
  float entry[4][3];
  for(int i = 0; i < triangle_vector.size(); i++){
    
    /*make a float array of the normal vector and vertices*/
    
    for(int j = 0; j < 3;j++)
      entry[0][j]=0.0;
    for(int j = 1; j < 4;j++)
      entry[j][0]=(float)triangle_vector[i].pts[j-1].x;
    for(int j = 1; j < 4;j++)
      entry[j][1]=(float)triangle_vector[i].pts[j-1].y;
    for(int j = 1; j < 4;j++)
      entry[j][2]=(float)triangle_vector[i].pts[j-1].z;
    
  
  /* write the vertices to the file*/
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 3;j++){
      stlfile.write((const char *)&(entry[i][j]), 4);
    }
  }

  /* 16 bit terminator*/
  stlfile.write((const char *)&c, 1);
  stlfile.write((const char *)&c, 1);
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
  
  if(link->visual!=0&&link->visual->geometry!=0){
    boost::shared_ptr<urdf::Mesh> pt =boost::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    if(pt!=0){
      //s << pt->filename;

      const char * name = pt->filename.c_str();
      const char * extension = (name+strlen(name)-4);
      const char * pathless_name;
      for(pathless_name=extension;*pathless_name!='/'&&pathless_name!=name;pathless_name--){}
      
      
      if(*pathless_name=='/')
	pathless_name++;
      
      char * pathed_name= (char *)malloc(strlen(pathless_name)+strlen("../../meshes/")+1);
      strcpy(pathed_name, "../../meshes/");
      strcat(pathed_name, pathless_name);

      if(strcmp(extension, ".stl")==0||strcmp(extension, ".STL")==0){
	pathed_name=pathed_name+6;
	s<<" "<<pathed_name;
	
      }
      else if(strcmp(extension, ".dae")==0||strcmp(extension, ".DAE")==0){
	daeToStl(pathed_name);
	pathed_name=pathed_name+6;
	s<<" "<<pathed_name;
	
      }
      else {
	std::cout<<"file extension not recognized: -"<<extension<<"-"<<std::endl;
	exit(0);
      }
    }
    else s<<"";
  }
  else
    s<<"";
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
    std::cout << " ";
    getline(f, s, '\n');
    std::cout<<s<<std::endl;
    
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
