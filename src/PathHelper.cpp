#include "PathHelper.h"
#include <cstdlib>
#include <iostream>
using namespace std;

static string pkgPath;

void makePath(string& localName) {
  static const char* pkgPath = getenv("PACKAGE_ROOT");
  static const char* defaultPath = ".";
  if(!pkgPath) pkgPath = defaultPath;
  cout << "Package root is " << pkgPath << endl;
  localName.insert(0,"/");
  localName.insert(0,string(pkgPath));
}
