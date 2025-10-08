//
// Created by Junny on  2020-06-29.
// Last Update on       2020-06-30
//

#ifndef SRC_Z1_PARAMETERS_HPP
#define SRC_Z1_PARAMETERS_HPP
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;

const double Deg2Radf = 0.0174533f;
const int WINDOW_SIZE = 15;
const int NUM_OF_DATA = 8000;
const int NUM_OF_F = 1000;
const double FORCE_THRESHOLD = 40.0;
const double LPF_FREQ = 40.0;
const double ALPHA = 1;



//#define UNIT_SPHERE_ERROR

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 12,
    SIZE_FORCE = 12,
    NUM_LEGS = 4
};

#endif //SRC_Z1_PARAMETERS_HPP
