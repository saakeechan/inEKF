//
// Created by junny on 7/2/20.
//

#ifndef SRC_ROBOT_MODEL_mini_cheetah_HPP
#define SRC_ROBOT_MODEL_mini_cheetah_HPP

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include "BasicFunctions_Estimator.hpp"
#include <vector>



// 0: MINI_CHEETAH_REAL, 1: MINI_CHEETAH_SIM, 2: HOUND
#define ROBOT 2


const int leg_num = 4;

using std::cout;
using std::endl;

class Robot_Model_mini_cheetah {

public:
    Robot_Model_mini_cheetah();
    ~Robot_Model_mini_cheetah();


    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Define Variable         /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    /*
    leg order : RH(Right Hind), LH(Left Hind), RF(Right Front), LF(Left Front)
    joint coordinate : ex, ey, ey

    RHHR = Right Hind Hip Roll
    RHHP = Right Hind Hip Pitch
    RHKP = Right Hind Knee Pitch
    RHF  = Right Hind Foot

    LHHR = Left Hind Hip Roll
    LHHP = Left Hind Hip Pitch
    LHKP = Left Hind Knee Pitch
    LHF  = Left Hind Foot

    RFHR = Right Front Hip Roll
    RFHP = Right Front Hip Pitch
    RFKP = Right Front Knee Pitch
    RFF  = Right Front Foot

    LFHR = Left Front Hip Roll
    LFHP = Left Front Hip Pitch
    LFKP = Left Front Knee Pitch
    LFF  = Left Front Foot
    */

    int leg_no=4;
    bool NANNAN;
    double err_max;
    int num_itter;

    double Torso_Length_x;
    double Torso_Length_y;
    double HipRoll2HipPitch;
    double Upper_Leg_Length;
    double Lower_Leg_Length;
    double xlim, ylim;

    Eigen::Vector3d Torso_COM;

    Eigen::Vector3d RHHR_COM, RHHP_COM, RHKP_COM;
    Eigen::Vector3d LHHR_COM, LHHP_COM, LHKP_COM;
    Eigen::Vector3d RFHR_COM, RFHP_COM, RFKP_COM;
    Eigen::Vector3d LFHR_COM, LFHP_COM, LFKP_COM;

    Eigen::Vector3d Torso2RHHR_Offset, Torso2LHHR_Offset, Torso2RFHR_Offset, Torso2LFHR_Offset;
    Eigen::Vector3d RHHR2RHHP_Offset, LHHR2LHHP_Offset, RFHR2RFHP_Offset, LFHR2LFHP_Offset;

    Eigen::Vector3d Upper_Leg_Offset, Lower_Leg_Offset;

    Eigen::Vector3d IMU2BD;


    double dt = 0;

#if ROBOT == 0
    double BASIC_HEIGHT = 0.37;

    enum leg_indexing{
        FR = 1,
        FL,
        RR,
        RL

    };



#elif ROBOT == 1
    double BASIC_HEIGHT = 0.2;

    enum leg_indexing{
        FR = 1,
        FL,
        RR,
        RL

    };

#elif ROBOT == 2
    double BASIC_HEIGHT = 0.48;


    enum leg_indexing{
        RR = 1,
        RL,
        FR,
        FL
    };



#endif


    Eigen::Matrix3d torso_inertia;

    Eigen::Matrix<double,4,2> leg_offset;
    Eigen::Vector3d Gravity;


    int leg_num_pub = leg_num;


    double magnitude = 1.0;

    double cov_gyro_const               = 1e-4;
    double cov_acc_const                = 1e-2;

    double cov_slip_const         = 1e-3;
    double cov_contact_const        = 1e-3;
    double cov_enc_const                = 1e-4;


    double cov_bias_gyro_const          = 1e-6;
    double cov_bias_acc_const           = 1e-6;
    //31888

    double cov_prior_orientation_const  = 1e-3;
    double cov_prior_velocity_const     = 1e-3;
    double cov_prior_position_const     = 1e-3;

    double cov_prior_bias_gyro_const    = 1e-6;
    double cov_prior_bias_acc_const     = 1e-6;






    Eigen::Matrix<double,3,3>   Covariance_Gyro;
    Eigen::Matrix<double,3,3>   Covariance_Acc;
    Eigen::Matrix<double,3,3>   Covariance_Contact;
    Eigen::Matrix<double,3,3>   Covariance_Slip;


    Eigen::Matrix<double,3,3> Covariance_Encoder;

    Eigen::Matrix<double,3,3> Covariance_Bias_Gyro;
    Eigen::Matrix<double,3,3> Covariance_Bias_Acc;

    Eigen::Matrix<double,3,3> Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3> Covariance_Prior_Velocity;
    Eigen::Matrix<double,3,3> Covariance_Prior_Position;
    Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Gyro;
    Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Acc;


    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Gyro             ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Acc              ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Contact      ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Slip      ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Encoder          ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Gyro        ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Acc         ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Velocity   ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Position   ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Gyro  ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Acc   ;


    void Covariance_Reset(Eigen::Matrix<double,12,1> cov_val_setting);
    std::vector<double> Variable_Contact_Cov(Eigen::Matrix<bool,-1,1> Contact, Eigen::Matrix<double,-1,1> dv);
      bool variable_contact_cov_mode=false;
      double cov_amplifier=0;
      bool slip_rejection_mode=false;
      double slip_threshold =0 ;

    double long_term_v_threshold;
    double long_term_a_threshold;


    ///////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////     Define Function     /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////




    Eigen::Vector3d L2R(Eigen::Vector3d in)
    {
        Eigen::Vector3d out = in;
        out(1) = -in(1);
        return out;
    }
    Eigen::Vector3d F2H(Eigen::Vector3d in)
    {
        Eigen::Vector3d out = in;
        out(0) = -in(0);
        return out;
    }
    Eigen::Matrix3d L2R(Eigen::Matrix3d in)
    {
        Eigen::Matrix3d out;
        Eigen::Matrix3d mY = Eigen::Matrix3d::Identity();
        mY(1,1) = -1;
        out = mY*in*mY;
        return out;
    }
    Eigen::Matrix3d F2H(Eigen::Matrix3d in)
    {
        Eigen::Matrix3d out;
        Eigen::Matrix3d mX = Eigen::Matrix3d::Identity();
        mX(0,0) = -1;
        out = mX*in*mX;
        return out;
    }




    Eigen::Matrix<double, 3, 1> Forward_Kinematics_Leg(const Eigen::Matrix<double,3,1> &vec,int i_leg) const
    {

        Eigen::Vector3d dp = Eigen::Vector3d::Zero(3);
        Eigen::Vector3d pHip = Eigen::Vector3d::Zero(3);
        Eigen::Vector3d p = Eigen::Vector3d::Zero(3);

        double q1 = vec(0);
        double q2 = vec(1);
        double q3 = vec(2);

        double s1 = sin(q1);
        double s2 = sin(q2);
        double s3 = sin(q3);
        double c1 = cos(q1);
        double c2 = cos(q2);
        double c3 = cos(q3);

        double c23 = c2 * c3 - s2 * s3;
        double s23 = s2 * c3 + c2 * s3;

#if ROBOT == 0
        double abadLinkLength = 0.062;
        double hipLinkLength = 0.209;
        double kneeLinkLength = 0.18;
        double kneeLinkY_offset = 0.004;

        double bodyLength = 0.2 * 2;
        double bodyWidth = 0.049 * 2;
        double kinemaitcs_direction=1.0;
#elif ROBOT == 1
        double abadLinkLength = 0.062;
        double hipLinkLength = 0.209;
        double kneeLinkLength = 0.195;
        double kneeLinkY_offset = 0.004;

        double bodyLength = 0.19 * 2;
        double bodyWidth = 0.049 * 2;
        double kinemaitcs_direction=1.0;
#elif ROBOT == 2
        double abadLinkLength = 0.1135;
        double hipLinkLength = 0.3279;
        double kneeLinkLength = 0.35;
        double kneeLinkY_offset = 0.0;
        double bodyLength = 0.349*2;
        double bodyWidth = 0.1*2;
        double kinemaitcs_direction=-1.0;
#endif
        Eigen::Vector3d abadLocation;
        abadLocation << 0.5* bodyLength, 0.5* bodyWidth, 0.5* 0;

        if ((i_leg == FL) || (i_leg == FR)){
            pHip(0) = abadLocation(0);
        }else{
            pHip(0) = -abadLocation(0);
        }

        if ((i_leg == FL) || (i_leg == RL)){
            pHip(1) = abadLocation(1);
        }else{
            pHip(1) = -abadLocation(1);
        }

        pHip(2) = abadLocation(2);

//        cout<<" phip is "<<endl<<pHip<<endl;

        Eigen::Vector4d sideSigns;
        sideSigns << -1, 1, -1, 1;

        double sideSign = sideSigns(i_leg-1);


        dp(0) = kinemaitcs_direction*(kneeLinkLength * s23 + hipLinkLength * s2);//x direction
        dp(1) = (abadLinkLength + kneeLinkY_offset) * sideSign * c1
                + kneeLinkLength * (s1 * c23) + hipLinkLength * c2 * s1;
        dp(2) = (abadLinkLength + kneeLinkY_offset) * sideSign * s1
                - kneeLinkLength * (c1 * c23) - hipLinkLength * c1 * c2;

        //cout<<"legnum "<<i_leg<<" dp is "<<endl<<dp<<endl;

        p = pHip + dp;

        //cout<<"legnum "<<i_leg<<" p is "<<endl<<p<<endl;

        p = IMU2BD + p;

        return p;
    }



    Eigen::Matrix<double, 12, 1> Forward_Kinematics(const Eigen::Matrix<double,12,1> &vec) const
    {

        Eigen::Matrix<double, 3, 1> x_foot1;
        Eigen::Matrix<double, 3, 1> x_foot2;
        Eigen::Matrix<double, 3, 1> x_foot3;
        Eigen::Matrix<double, 3, 1> x_foot4;
        Eigen::Matrix<double, 12, 1> x_foots;
        Eigen::Matrix<double, 3, 1> q1;
        Eigen::Matrix<double, 3, 1> q2;
        Eigen::Matrix<double, 3, 1> q3;
        Eigen::Matrix<double, 3, 1> q4;

        q1 = vec.block(0,0,3,1);
        q2 = vec.block(3,0,3,1);
        q3 = vec.block(6,0,3,1);
        q4 = vec.block(9,0,3,1);

        x_foot1 = Forward_Kinematics_Leg(q1,1);
        x_foot2 = Forward_Kinematics_Leg(q2,2);
        x_foot3 = Forward_Kinematics_Leg(q3,3);
        x_foot4 = Forward_Kinematics_Leg(q4,4);
        x_foots << x_foot1,x_foot2,x_foot3,x_foot4;





        return x_foots;
    }






    Eigen::Matrix<double, 3, 3> Jacobian_Leg(const Eigen::Matrix<double,3,1> &vec,int i_leg) const
    {

        Eigen::Matrix<double, 3, 3> J = Eigen::Matrix3d::Zero(3,3);

        double q1 = vec(0);
        double q2 = vec(1);
        double q3 = vec(2);

        double s1 = sin(q1);
        double s2 = sin(q2);
        double s3 = sin(q3);

        double c1 = cos(q1);
        double c2 = cos(q2);
        double c3 = cos(q3);

        double c23 = c2 * c3 - s2 * s3;
        double s23 = s2 * c3 + c2 * s3;

#if ROBOT == 0


        double abadLinkLength = 0.062;
        double hipLinkLength = 0.209;
        double kneeLinkLength = 0.18;
        double kneeLinkY_offset = 0.004;

        double bodyLength = 0.2 * 2;
        double bodyWidth = 0.049 * 2;
        double kinemaitcs_direction=1.0;

#elif ROBOT == 1


        double abadLinkLength = 0.062;
        double hipLinkLength = 0.209;
        double kneeLinkLength = 0.195;
        double kneeLinkY_offset = 0.004;

        double bodyLength = 0.19 * 2;
        double bodyWidth = 0.049 * 2;
        double kinemaitcs_direction=1.0;
#elif ROBOT == 2
        double abadLinkLength = 0.1135;
        double hipLinkLength = 0.3279;
        double kneeLinkLength = 0.35;
        double kneeLinkY_offset = 0.0;

        double bodyLength = 0.349*2;
        double bodyWidth = 0.1*2;
        double kinemaitcs_direction=-1.0;
#endif

        Eigen::Vector4d sideSigns;
        sideSigns << -1, 1, -1, 1;

        double sideSign = sideSigns(i_leg-1);

        J(0, 0) = 0;
        J(0, 1) = kinemaitcs_direction*(kneeLinkLength * c23 + hipLinkLength * c2);
        J(0, 2) = kinemaitcs_direction*(kneeLinkLength * c23);
        J(1, 0) = kneeLinkLength * c1 * c23 + hipLinkLength * c1 * c2
                - (abadLinkLength+kneeLinkY_offset) * sideSign * s1;
        J(1, 1) = -kneeLinkLength * s1 * s23 - hipLinkLength * s1 * s2;
        J(1, 2) = -kneeLinkLength * s1 * s23;
        J(2, 0) = kneeLinkLength * s1 * c23 + hipLinkLength * c2 * s1
                + (abadLinkLength+kneeLinkY_offset) * sideSign * c1;
        J(2, 1) = kneeLinkLength * c1 * s23 + hipLinkLength * c1 * s2;
        J(2, 2) = kneeLinkLength * c1 * s23;

        return J;
    }



    Eigen::Matrix<double, 12, 12> Jacobian(const Eigen::Matrix<double,12,1> &vec) const
    {


        Eigen::Matrix<double, 12, 12> J;

        Eigen::Matrix<double, 3, 3> J1;
        Eigen::Matrix<double, 3, 3> J2;
        Eigen::Matrix<double, 3, 3> J3;
        Eigen::Matrix<double, 3, 3> J4;

        Eigen::Matrix<double, 3, 1> q1;
        Eigen::Matrix<double, 3, 1> q2;
        Eigen::Matrix<double, 3, 1> q3;
        Eigen::Matrix<double, 3, 1> q4;

        q1 = vec.block(0,0,3,1);
        q2 = vec.block(3,0,3,1);
        q3 = vec.block(6,0,3,1);
        q4 = vec.block(9,0,3,1);


        J1 = Jacobian_Leg(q1,1);
        J2 = Jacobian_Leg(q2,2);
        J3 = Jacobian_Leg(q3,3);
        J4 = Jacobian_Leg(q4,4);

        J.setZero();
        J.block(0,0,3,3) = J1.block(0,0,3,3);
        J.block(3,3,3,3) = J2.block(0,0,3,3);
        J.block(6,6,3,3) = J3.block(0,0,3,3);
        J.block(9,9,3,3) = J4.block(0,0,3,3);




        return J;
    }



private:

};





#endif //SRC_ROBOT_MODEL_mini_cheetah_HPP
