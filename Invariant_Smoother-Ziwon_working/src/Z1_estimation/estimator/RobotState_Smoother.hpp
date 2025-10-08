//
// Created by Junny on  2020-06-29.
// Last Update on       2020-06-30
//

#ifndef ROBOTSTATE_SMOOTHER
#define ROBOTSTATE_SMOOTHER

// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <unordered_map>

#include <set>

// Parameters
#include "Z1_Parameters.hpp"


using namespace Eigen;
using namespace std;



typedef struct _ROBOT_STATES_
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::Matrix<double, 12, 1> d;
    Eigen::Matrix<double, 12, 1> d_v;

    Eigen::Vector3d Bias_Gyro;
    Eigen::Vector3d Bias_Acc;

    Eigen::Matrix<bool, 4, 1> Contact;
    Eigen::Matrix<bool, 4, 1> Hard_Contact;
    Eigen::Matrix<bool, 4, 1> Slip;

    int contact_leg_num;

    int state_size;
    int state_idx;
    int para_size;
    int para_idx;

}ROBOT_STATES;




class kinematics_info {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d fk_kin;
    Eigen::Matrix3d meas_primitive_sqrt_info;

    int leg_no;
    int leg_num_in_state;

    kinematics_info(int _leg_no) : leg_no(_leg_no) {}

    bool operator<(const kinematics_info& t) const {
      return leg_no < t.leg_no;
    }

};


class factor_info{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        const static int num_z = 30;

        Eigen::Matrix<double,num_z,1>  Z;
        std::set<kinematics_info> leg_info;



        Eigen::MatrixXd Mi;
        Eigen::MatrixXd Mj;

        Eigen::MatrixXd prop_primitive_sqrt_info;

        int shared_contact;

        //propagation
        int prop_para0_size ;
        int prop_para1_size ;
        int prop_res_size   ;


        //measurement

        int meas_para_size;

};




typedef struct _ROBOT_STATES_PSEUDO_
{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::Matrix<double, 12, 1> d_v;

    Eigen::Vector3d Bias_Gyro;
    Eigen::Vector3d Bias_Acc;

    Eigen::Matrix<bool, 4, 1> Contact;
    Eigen::Matrix<bool, 4, 1> Hard_Contact;
    Eigen::Matrix<bool, 4, 1> Slip;

}ROBOT_STATES_PSEUDO;


class factor_info_pseudo{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        const static int num_z = 30;
        Eigen::Matrix<double,num_z,1>  Z;

        std::set<kinematics_info> leg_info;

        Eigen::Matrix<double, 15,15> prop_primitive_sqrt_info;

        std::vector<double> contact_cov_array;

};

#endif //ROBOTSTATE_SMOOTHER
