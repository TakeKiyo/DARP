#include <iostream>
#include "InputData.h"
using namespace std;

int main(int argc, char *argv[]){
    InputData inputdata;
    string inputfile = argv[1];
    inputdata.setInputData(inputfile = argv[1]);
    cout << "Hello,world" << endl;
  
}
