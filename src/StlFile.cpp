#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include "StlFile.h"

//using namespace std;

StlFile::StlFile(){
  StlReadFile((const char *)NULL);
}

StlFile::StlFile(const char * filename){
  StlReadFile(filename);
}

StlFile::StlFile(const StlFile & stl){
  header = malloc(80);
  if(!stl.header==NULL)
    memcpy(header, stl.header, (size_t)80);
  else 
    header=NULL;
  numTriangles = stl.numTriangles;
  normals = *(new vector<Point>());
  triangles = *(new vector<Triangle>());
  for(int i = 0; i < numTriangles; i++){
    normals.push_back(stl.normals[i]);
    triangles.push_back(stl.triangles[i]);
  }
}

void StlFile::StlReadFile(const char * filename){
  
  header = malloc(80);
  numTriangles =0;
  normals = *(new std::vector<Point>());
  triangles = *(new std::vector<Triangle>());
  if(filename==NULL)
    return;
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
