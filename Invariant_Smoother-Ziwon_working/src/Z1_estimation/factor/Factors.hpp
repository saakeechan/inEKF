//
// Created by Junny on  2020-06-29.
// Last Update on       2022-01-06.
//

#ifndef SRC_FACTORS_HPP
#define SRC_FACTORS_HPP


#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <set>
#include <vector>


#include "../utility/BasicFunctions_Estimator.hpp"
#include "../utility/Robot_Model_mini_cheetah.hpp"
#include "../utility/tic_toc.h"

#include "../estimator/RobotState_Smoother.hpp"

#include "../estimator/Z1_Parameters.hpp"

class Factors {


public:
//Smoother
    void Propagation_Factor(
            Eigen::Matrix<double,9,1> &residual,
            Eigen::Matrix<double,9,15> &partial_i,
            Eigen::Matrix<double,9,15> &partial_j,
            const Eigen::Matrix<double,42,1> &X,
            const Eigen::Matrix<double,30,1> &dX,
            const Eigen::Matrix<double,6,1> &IMU,
            const Robot_Model_mini_cheetah &robot);

    void Measurement_Factor(
            Eigen::Matrix<double,3,1> &residual,
            Eigen::Matrix<double,3,15> &partial_i,
            Eigen::Matrix<double,3,15> &partial_j,
            const Eigen::Matrix<double,42,1> &X,
            const Eigen::Matrix<double,30,1> &dX,
            Eigen::Vector3d ENC_i,
            Eigen::Vector3d ENC_j,
            int leg_num,
            double contact_cov,
            const Robot_Model_mini_cheetah &robot);

    void Prior_Factor(
            Eigen::Matrix<double,15,1> &residual,
            Eigen::Matrix<double,15,15> &partial_0,
            const Eigen::Matrix<double,21,1> &X,
            const Eigen::Matrix<double,21,1> &X_init,
            const Eigen::Matrix<double,15,1> &dX,
            const Robot_Model_mini_cheetah &robot);

    void Bias_Factor(
            Eigen::Matrix<double,6,1> &residual,
            Eigen::Matrix<double,6,15> &partial_i,
            Eigen::Matrix<double,6,15> &partial_j,
            const Eigen::Matrix<double,42,1> &X,
            const Eigen::Matrix<double,30,1> &dX,
            const Robot_Model_mini_cheetah &robot);

    void Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
                                          Eigen::Matrix<double,3,1> &residual,
                                          Eigen::Matrix<double,3,15> &partial_s,
                                          Eigen::Matrix<double,3,15> &partial_e,
                                          const Eigen::Matrix<double,21,1> &X_s,
                                          const Eigen::Matrix<double,21,1> &X_e,
                                          Eigen::Vector3d ENC_s,
                                          Eigen::Vector3d ENC_e,
                                          const Robot_Model_mini_cheetah &robot);

private:


};



class Inv_Factors {


public:


    ROBOT_STATES* RS_temp;
    factor_info* fac_info;


    Eigen::Vector3d gravity;
    double dt;

    Robot_Model_mini_cheetah robot;
    int frame_count;

    int num_z = 30;


    //prior
    Eigen::Matrix<double,-1,-1> X_Prior;
    Eigen::Matrix<double,6,1> Bias_Prior;

    Eigen::Matrix<double,9,9> SQRT_INFO_Prior;
    Eigen::Matrix<double,6,6> SQRT_INFO_Prior_BIAS;


    Eigen::Matrix<double,-1,-1> X_0_bar;
    Eigen::Matrix<double,6,1> Bias_0_bar;



    //gra, hess
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> Hessian_Marg;
    Eigen::Matrix<double,Eigen::Dynamic, 1> gradient_Marg;

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> H;
    Eigen::Matrix<double,Eigen::Dynamic, 1> b;
    double cost_unchanged;

    Eigen::Matrix<double,Eigen::Dynamic, 1> big_jacobian_by_vector;


    ~Inv_Factors(){
//        delete[] RS_temp;
//        delete[] fac_info;
    }


    bool marginalization_flag = false;


    void Batch_Initialize(ROBOT_STATES* _RS, const ROBOT_STATES &RS_Prior,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                      factor_info* _fac_info,
                      const Robot_Model_mini_cheetah _robot);


    void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                         Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                         Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                         double &cost);
    void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                               Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                               Eigen::Matrix<double,4,1> &cost_log, Eigen::MatrixXd &Jacobian_Vector);


    void Marg_Initialize(ROBOT_STATES* _RS,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                      const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                         factor_info* _fac_info,
                      const Robot_Model_mini_cheetah _robot);


    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                         Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                         Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                         double &cost);

    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                              Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                              Eigen::Matrix<double,4,1> &cost_log);



    void Invariant_Propagation_Factor(int j,
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


    void Invariant_Measurement_Factor(int i, int leg_num,
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);

    void Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
                                      Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                      Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);

    void Invariant_Prior_RVP_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);

    void Invariant_Prior_Bias_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


private:





};



class Adv_Inv_Factors {


public:


    ROBOT_STATES* RS_temp;
    factor_info* fac_info;


    Eigen::Vector3d gravity;
    double dt;

    Robot_Model_mini_cheetah robot;
    int frame_count;

    int num_z = 30;


    //prior
    Eigen::Matrix<double,5,5> X_Prior;
    Eigen::Matrix<double,-1,1> d_Prior;
    Eigen::Matrix<double,6,1> Bias_Prior;

    Eigen::Matrix<double,9,9> SQRT_INFO_Prior;
    Eigen::Matrix<double,6,6> SQRT_INFO_Prior_BIAS;


    Eigen::Matrix<double,5,5> X_0_bar;
    Eigen::Matrix<double,-1,1> d_0_bar;
    Eigen::Matrix<double,6,1> Bias_0_bar;



    //gra, hess
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> Hessian_Marg;
    Eigen::Matrix<double,Eigen::Dynamic, 1> gradient_Marg;

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> H;
    Eigen::Matrix<double,Eigen::Dynamic, 1> b;
    double cost_unchanged;

    Eigen::Matrix<double,Eigen::Dynamic, 1> big_jacobian_by_vector;


    ~Adv_Inv_Factors(){
//        delete[] RS_temp;
//        delete[] fac_info;
    }


    bool marginalization_flag = false;


    void Batch_Initialize(ROBOT_STATES* _RS, const ROBOT_STATES &RS_Prior,
                          const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                          factor_info* _fac_info,
                          const Robot_Model_mini_cheetah _robot);


    void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                               Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                               double &cost);

    void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                               Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                               Eigen::Matrix<double,4,1> &cost_log, Eigen::MatrixXd &Jacobian_Vector);


    void Marg_Initialize(ROBOT_STATES* _RS,
                         const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                         const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                         const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                         factor_info* _fac_info,
                         const Robot_Model_mini_cheetah _robot);


    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                              Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                              double &cost);

    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                              Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                              Eigen::Matrix<double,4,1> &cost_log);

    void Invariant_Propagation_Factor(int j,
                                      Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                      Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


    void Invariant_Measurement_Factor(int i, int leg_num,
                                      Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                      Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


    void Invariant_Prior_RVP_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);

    void Invariant_Prior_Bias_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


private:





};



class Pseudo_Inv_Factors {

public:

    ROBOT_STATES_PSEUDO* RS_temp;
    factor_info_pseudo* fac_info;

    int num_z = 30;

    Eigen::Vector3d gravity;
    double dt;

    Robot_Model_mini_cheetah robot;
    int frame_count;


    //prior
    Eigen::Matrix<double,5,5> X_Prior;
    Eigen::Matrix<double,6,1> Bias_Prior;
    Eigen::Matrix<double,9,9> SQRT_INFO_Prior;

    Eigen::Matrix<double,6,6> SQRT_INFO_Prior_BIAS;

    Eigen::Matrix<double,5,5> X_0_bar;
    Eigen::Matrix<double,6,1> Bias_0_bar;

    //gra, hess
    Eigen::Matrix<double,15, 15> Hessian_Marg;
    Eigen::Matrix<double,15, 1> gradient_Marg;

    Eigen::Matrix<double,15, 15> H;
    Eigen::Matrix<double,15, 1> b;
    double cost_unchanged;

    bool marginalization_flag = false;



    ~Pseudo_Inv_Factors(){
//        delete[] RS_temp;
//        delete[] fac_info;
    }




    void Batch_Initialize(ROBOT_STATES_PSEUDO* _RS, const ROBOT_STATES_PSEUDO &RS_Prior,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                      factor_info_pseudo* _fac_info,
                      const Robot_Model_mini_cheetah _robot);


    void Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS,
                                         Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                         Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                         double &cost);


    void Marg_Initialize(ROBOT_STATES_PSEUDO* _RS,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                      const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                      const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                         factor_info_pseudo* _fac_info,
                      const Robot_Model_mini_cheetah _robot);

    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS, Eigen::Matrix<double,15*(WINDOW_SIZE+1),1> Zeta_Xi,
                                              Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                              double &cost);

    void Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS, Eigen::Matrix<double,15*(WINDOW_SIZE+1),1> Zeta_Xi,
                                         Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                         Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                         Eigen::Matrix<double,4,1> &cost_log);



    void Invariant_Propagation_Factor(int j,
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


    void Invariant_Measurement_Factor(int i, int leg_num,
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


    void Invariant_Prior_RVP_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);

    void Invariant_Prior_Bias_Factor(
            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost);


private:








};


#endif //SRC_FACTORS_HPP
