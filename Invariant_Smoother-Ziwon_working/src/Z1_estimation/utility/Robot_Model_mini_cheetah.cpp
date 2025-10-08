//
// Created by junny on 7/2/20.
//

#include "Robot_Model_mini_cheetah.hpp"
#include <stdio.h>
#include <iostream>

////////////////////////////////////////////////// HUBODOG4 /////////////////////////////////////////////////////////
////////////////////////////////////////////////// HUBODOG4 /////////////////////////////////////////////////////////



Robot_Model_mini_cheetah::Robot_Model_mini_cheetah() {
    Gravity << 0.0, 0.0, -9.80665;

    err_max = 1e-7;

    //length ->updated
    Torso_Length_x = 0.203 * 2.0;
    Torso_Length_y = 0.08 * 2.0;
    Upper_Leg_Length = 0.20;
    Lower_Leg_Length = 0.20;
    HipRoll2HipPitch = 0.035 - 0.0015;
    xlim = (Upper_Leg_Length + Lower_Leg_Length) / 2.0 * 1.5;
    ylim = (Upper_Leg_Length + Lower_Leg_Length) / 2.0 * 1.5;

    //offsets
    Torso2RHHR_Offset << -Torso_Length_x / 2.0, -Torso_Length_y / 2.0, 0;
    Torso2LHHR_Offset << -Torso_Length_x / 2.0, +Torso_Length_y / 2.0, 0;
    RHHR2RHHP_Offset << 0, -HipRoll2HipPitch, 0;
    LHHR2LHHP_Offset << 0, +HipRoll2HipPitch, 0;

    Torso2RFHR_Offset << Torso_Length_x / 2.0, -Torso_Length_y / 2.0, 0;
    Torso2LFHR_Offset << Torso_Length_x / 2.0, +Torso_Length_y / 2.0, 0;
    RFHR2RFHP_Offset << 0, -HipRoll2HipPitch, 0;
    LFHR2LFHP_Offset << 0, +HipRoll2HipPitch, 0;


    Upper_Leg_Offset << 0, 0, -Upper_Leg_Length;
    Lower_Leg_Offset << 0, 0, -Lower_Leg_Length;

    //centerofmass
    Torso_COM << 0 + 0.1, 0, 0;

    LFHR_COM << 0.00011, 0.00283, 0.00140;
    LFHP_COM << -0.00026, 0.03369, -0.02516;
    LFKP_COM << 0.00144, 0.0, -0.04022; //LLEG only

    LHHR_COM = F2H(LFHR_COM);
    LHHP_COM = (LFHP_COM);
    LHKP_COM = (LFKP_COM);

    RFHR_COM = L2R(LFHR_COM);
    RFHP_COM = L2R(LFHP_COM);
    RFKP_COM = L2R(LFKP_COM);

    RHHR_COM = F2H(RFHR_COM);
    RHHP_COM = (RFHP_COM);
    RHKP_COM = (RFKP_COM);


    num_itter = -1;
    NANNAN = false;


    IMU2BD.setZero();


}

Robot_Model_mini_cheetah::~Robot_Model_mini_cheetah() {

}


void Robot_Model_mini_cheetah::Covariance_Reset(Eigen::Matrix<double, 12, 1> cov_val_setting) {


    cov_gyro_const = cov_val_setting(0);
    cov_acc_const = cov_val_setting(1);
    cov_slip_const = cov_val_setting(2);
    cov_contact_const = cov_val_setting(3);
    cov_enc_const = cov_val_setting(4);
    cov_bias_gyro_const = cov_val_setting(5);
    cov_bias_acc_const = cov_val_setting(6);
    cov_prior_orientation_const = cov_val_setting(7);
    cov_prior_velocity_const = cov_val_setting(8);
    cov_prior_position_const = cov_val_setting(9);
    cov_prior_bias_gyro_const = cov_val_setting(10);
    cov_prior_bias_acc_const = cov_val_setting(11);

    Covariance_Gyro = cov_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Acc = cov_acc_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Slip = cov_slip_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Contact = cov_contact_const * Eigen::Matrix<double, 3, 3>::Identity();


    Covariance_Encoder = cov_enc_const * Eigen::Matrix<double, 3, 3>::Identity();


    Covariance_Bias_Gyro = cov_bias_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Bias_Acc = cov_bias_acc_const * Eigen::Matrix<double, 3, 3>::Identity();

    Covariance_Prior_Orientation = cov_prior_orientation_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Velocity = cov_prior_velocity_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Position = cov_prior_position_const * Eigen::Matrix<double, 3, 3>::Identity();

    Covariance_Prior_Bias_Gyro = cov_prior_bias_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Bias_Acc = cov_prior_bias_acc_const * Eigen::Matrix<double, 3, 3>::Identity();


    SQRT_INFO_Covariance_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_gyro_const);
    SQRT_INFO_Covariance_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_acc_const);
    SQRT_INFO_Covariance_Slip = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_slip_const);
    SQRT_INFO_Covariance_Contact = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_contact_const);
    SQRT_INFO_Covariance_Encoder = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_enc_const);
    SQRT_INFO_Covariance_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_bias_gyro_const);
    SQRT_INFO_Covariance_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_bias_acc_const);
    SQRT_INFO_Covariance_Prior_Orientation =
            Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_orientation_const);
    SQRT_INFO_Covariance_Prior_Velocity = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_velocity_const);
    SQRT_INFO_Covariance_Prior_Position = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_position_const);
    SQRT_INFO_Covariance_Prior_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_gyro_const);
    SQRT_INFO_Covariance_Prior_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_acc_const);

}

std::vector<double>
Robot_Model_mini_cheetah::Variable_Contact_Cov(Eigen::Matrix<bool, -1, 1> Contact, Eigen::Matrix<double, -1, 1> dv) {

    std::vector<double> contact_cov_array;
    contact_cov_array.clear();

    for (int k = 0; k < leg_no; k++) {

        double contact_cov = cov_contact_const;

        if (Contact(k)) {
            if (variable_contact_cov_mode) {
                contact_cov = dv.block(3 * k, 0, 3, 1).norm();
                contact_cov = std::pow(10, log10(cov_contact_const) + contact_cov * cov_amplifier);
                if (contact_cov > cov_slip_const) {
                    contact_cov = cov_slip_const;
                }
            } else if (slip_rejection_mode) {
                if ((dv.block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
                    contact_cov = cov_slip_const;
                } else {
                    contact_cov = cov_contact_const;
                }
            } else {
                contact_cov = cov_contact_const;
            }
        }
        contact_cov_array.push_back(contact_cov);
    }

    //cout<<"contact_cov is "<<endl<<contact_cov_array[0]<<", "<<contact_cov_array[1]<<", "<<contact_cov_array[2]<<", "<<contact_cov_array[3]<<", "<<endl;

    return contact_cov_array;
}



