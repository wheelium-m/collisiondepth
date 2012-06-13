#include <iostream>
#include <fstream>
 
using namespace std;

int main(){
  float x[100][100][3];
  ofstream file("floats.bin");
  for(int i = 0; i < 30000; i++){
    ((float *)x)[i] = 0.5;
  }
  file.write((char *)x, 120000);
  file.close();
  return 0;
}
