#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include "api/mic_compensation_api.h"

using namespace mic;
using namespace std;

int main(int argc, char *argv[])
{
    mic_init_worker("ellipsoid", "mic_model_ellipsoid_1002_02.mdl");
    return 0;
}