#include <XnCppWrapper.h>
#include "MapMaker.h"
#include <fstream>

using namespace std;

MapMaker::MapMaker() {
  nRetVal = XN_STATUS_OK;
  nRetVal = context.Init();
  checkError("Error creating context: ", nRetVal);
  nRetVal = depth.Create(context);
  checkError("Error creating depth generator: ", nRetVal);
  init();
}

void MapMaker::init() {
  nRetVal = context.StartGeneratingAll();
  checkError("Error starting the depth generator: ", nRetVal);
}

void MapMaker::stop() {
  xnContextRelease((XnContext*)&context);
}

void MapMaker::grabFrame() {
  nRetVal = context.WaitOneUpdateAll(depth);
  checkError("Failed updating data: ", nRetVal);
  // Take current depth map  
  const XnDepthPixel* pDepthMap = depth.GetDepthMap();
  ofstream file("depth_texture.bin", std::ios::binary);
  float data[640*480];
  for(int i = 0; i < 640*480; i++){
    data[i]=(float)(*(pDepthMap+i))/1000.0;
  } 
  file.write((char *)data, 640*480*4);
  file.close();
  //std::cout<<"(float)pDepthMap[0] = "<<(float)pDepthMap[0]<<endl;
  //std::cout<<"pDepthMap[0] = "<<pDepthMap[0]<<endl;
}

void MapMaker::checkError(std::string where, XnStatus what) {
  if(!(nRetVal==XN_STATUS_OK)){
    cout<<where<<xnGetStatusString(what)<<endl;
    exit(0);
  }
  return;
}

