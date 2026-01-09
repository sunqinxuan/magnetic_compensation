#include <iostream>
#include <string>
#include <fstream>
#include "api/interface.h"

using namespace mic;
using namespace std;

int main(int argc, char *argv[])
{
    initialize();

    loadCalibData("Flight.txt");
    cout<<"load calib data"<<endl;
    calibModel("a.mdl");
    cout<<"calibrate model"<<endl;

    loadTaskData("task.txt");
    cout<<"load task data"<<endl;
    loadModel("a.mdl");
    cout<<"load model"<<endl;
    compensate("out.txt");
    cout<<"compensate"<<endl;

    return 0;
}
