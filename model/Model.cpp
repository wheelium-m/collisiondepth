#include "Model.h"
#include "Joint.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <btBulletDynamicsCommon.h>

using namespace std;

Model::Model(){
  joints = NULL;
  numJoints=0;
}

Model::Model(Joint* jts, int num){
  joints=jts;
  numJoints = num;
}

Model::Model(const Model &m){
  numJoints = m.numJoints;
  joints = (Joint *)(new Joint[m.numJoints]);
  memcpy(joints, m.joints, m.numJoints*sizeof(Joint));
}

void Model::internalTransform(btQuaternion *quats, int num){
  
  if(!(num==numJoints)){
    cout<<"The number of joints does not equal the number of transformations"<<endl;
    exit(0);
  }
  btTransform trans[numJoints];
  for(int i = 0; i <numJoints;i++){
    trans[i] = *(new btTransform(quats[i], btVector3(0.0,0.0,0.0)));
  } 
  for(int i = 0; i < numJoints; i++){
    joints[i].trans=joints[i].trans*trans[i];
    
  }
}

Model Model::externalTransform(btQuaternion *quats, int num){
  Model m(*this);
  m.internalTransform(quats, num);
  return m;
}


/*
Model::~Model(){
  delete joints;
}
*/
