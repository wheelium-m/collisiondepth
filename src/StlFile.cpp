#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include "StlFile.h"
#include <string>
#ifdef DAE
#include <tinyxml.h>
#endif
#include <sstream>
#include <algorithm>
#include <iterator>

//using namespace std;

StlFile::StlFile() : header(NULL) {
  ReadFile((const char *)NULL);
}

StlFile::StlFile(const char * filename) : header(NULL) {
  ReadFile(filename);
}

StlFile::StlFile(const StlFile* stl) : header(NULL), numTriangles(0) {
  if(!stl) return;
  if(stl->header) {
    header = malloc(80);
    memcpy(header, stl->header, (size_t)80);
  }
  numTriangles = stl->numTriangles;
  for(int i = 0; i < numTriangles; i++){
    normals.push_back(stl->normals[i]);
    triangles.push_back(stl->triangles[i]);
  }
}

StlFile::~StlFile() {
  if(header) free(header);
}

/* Any file you pass, local or absolute, .dae or .stl will be turned
 * into an stl file that is kept in the Joint class.
 * TODO: reconcile STL and DAE normal formats.
 */
void StlFile::ReadFile(const char * filename){
  if(header) free(header);
  header = malloc(80);
  numTriangles =0;
  if(filename==NULL||strcmp(filename, "")==0)
    return;
  const char * extension = (filename+strlen(filename)-4);
  const char * pathless_name;
  for(pathless_name=extension;*pathless_name!='/'&&pathless_name!=filename;pathless_name--){}
  if(*pathless_name=='/')
    pathless_name++;
  char pathed_name[strlen(pathless_name)+strlen("meshes/")+1];
  strcpy(pathed_name, "meshes/");
  strcat(pathed_name, pathless_name);
  cout<<"Loading mesh: "<<pathless_name<<endl;
  if(strcmp(extension, ".stl")==0||strcmp(extension, ".STL")==0)
    StlReadFile(pathed_name);
  else if(strcmp(extension, ".dae")==0||strcmp(extension, ".DAE")==0)
    DaeReadFile(pathed_name);
  else{
    cout<<"file extension not recognized: -"<<extension<<"-"<<endl;
    exit(0);
  }
}

void StlFile::StlReadFile(const char * filename){
  
  ifstream file(filename, ios::binary);
  if(!file.is_open()){
    cout<<"The file could not be opened: "<<filename<<endl;
    return;
  }
  char format[6];
  format[5] = '\0';
  const char *solid = "solid";

  for(int i = 0; i < 5;i++){
    file>>format[i];
  }
  
  file.close();
  if(strcmp((char *)format, solid)==0)
    StlReadAsciiFile(filename);
  else
    StlReadBinaryFile(filename);

}
#ifdef DAE
void StlFile::DaeReadFile(const char * filename){
  
  TiXmlDocument doc(filename);
  doc.LoadFile();
  
  TiXmlElement * lgeo=doc.RootElement()->FirstChildElement("library_geometries");
  
  TiXmlElement * geo=lgeo->FirstChildElement("geometry");
  TiXmlElement * mesh=geo->FirstChildElement("mesh");
  
  TiXmlElement *input=mesh->FirstChildElement("vertices")->FirstChildElement("input");
  
  const char * position_name=0;
  const char * normals_name=0;
  const char * position_char_data=0;
  const char * normals_char_data=0;
  const char * pdata;
  
  while(input&&(!position_name||!normals_name)){
    if(strcmp(input->Attribute("semantic"), "POSITION")==0){
      position_name = input->Attribute("source");
    }
    else if(strcmp(input->Attribute("semantic"), "NORMAL")==0){
      normals_name = input->Attribute("source");
    }
    input=input->NextSiblingElement("input");
  }
  
  if(!position_name||!normals_name){
    cout<<"Incomplete information in the DAE file"<<endl;
    return;
  }
  position_name+=1;
  normals_name+=1;
  
  TiXmlElement * source=mesh->FirstChildElement("source");
  
  while(source&&(!position_char_data||!normals_char_data)){
    if(strcmp(source->Attribute("id"), position_name)==0){
      position_char_data=source->FirstChildElement("float_array")->GetText(); 
    }
    else if(strcmp(source->Attribute("id"), normals_name)==0){
      normals_char_data=source->FirstChildElement("float_array")->GetText();
      
    }
    source=source->NextSiblingElement("source");
  }
  
  TiXmlElement * triangles = mesh->FirstChildElement("triangles");
  
  TiXmlElement * p = triangles->FirstChildElement("p");
  
  TiXmlElement * t_input = triangles->FirstChildElement("input");
  
  int offset=0;
  int num_attr=0;
  while(t_input){
    if(strcmp(t_input->Attribute("semantic"), "VERTEX")==0)
      offset=num_attr;   
    num_attr++;
    t_input=t_input->NextSiblingElement("input");
  }
  
  pdata=p->GetText();

  vector<float> vertex_points, normal_points;
  vector<int> vector_pdata, order;
  vector<Point> vertices;
  istringstream pos(position_char_data);
  istringstream nor(normals_char_data);
  istringstream pd(pdata);

  copy(istream_iterator<float>(pos),
       istream_iterator<float>(),
       back_inserter<vector<float> >(vertex_points));
  copy(istream_iterator<float>(nor),
       istream_iterator<float>(),
       back_inserter<vector<float> >(normal_points));
  copy(istream_iterator<int>(pd),
       istream_iterator<int>(),
       back_inserter<vector<int> >(vector_pdata));
  for(int i = offset; i < vector_pdata.size()/num_attr; i+=num_attr){
    order.push_back(vector_pdata[i]);
  }
  int count=0;
  while(count<=normal_points.size()-3){
    normals.push_back(Point(normal_points[count++], normal_points[count++], normal_points[count++]));
  }
  count=0;
  while(count<=vertex_points.size()-3){
    vertices.push_back(Point(vertex_points[count++],vertex_points[count++],vertex_points[count++]));
  }
  count=0;
  while(count<=order.size()-3){
    this->triangles.push_back(Triangle(vertices[order[count++]],vertices[order[count++]],vertices[vector_pdata[count++]]));
  }
  numTriangles=this->triangles.size();
}
#else
void StlFile::DaeReadFile(const char * filename) {
  cout << filename << " not loaded. Project not built with DAE support." << endl;
}
#endif

void StlFile::StlReadBinaryFile(const char * filename){
  ifstream file(filename, ios::binary);
  file.read((char *)header, 80);
  file.read((char *)&numTriangles, 4);
  float normal[3];
  float point1[3],point2[3],point3[3];
  for(int i = 0; i < numTriangles; i++){
    file.read((char *)normal, 12);
    file.read((char *)point1, 12);
    file.read((char *)point2, 12);
    file.read((char *)point3, 12);
    normals.push_back(Point((float *)normal));
    triangles.push_back(Triangle(Point((float *)point1), Point((float *)point2), Point((float *)point3)));
    file.seekg(2, ios::cur);
  }
}

void StlFile::StlReadAsciiFile(const char * filename){
  ifstream file(filename);
  float normal[3];
  float point1[3], point2[3], point3[3];
  string s;
  file.ignore(256,'\n');
  file>>s;
  cout<<"You got here "<<s<<endl;
  int numtri=0;
  while(s=="facet"){
    cout<<"You got here, too"<<endl;
    file>>s>>normal[0]>>normal[1]>>normal[2];
    file>>s>>s>>s;
    file>>point1[0]>>point1[1]>>point1[2];
    file>>s;
    file>>point2[0]>>point2[1]>>point2[2];
    file>>s;
    file>>point3[0]>>point3[1]>>point3[2];
    file>>s>>s>>s;
    normals.push_back(Point(normal));
    triangles.push_back(Triangle(Point((float *)point1),Point((float *)point2),Point((float *)point3)));
    numtri++;
  }  
  numTriangles = numtri;
}
