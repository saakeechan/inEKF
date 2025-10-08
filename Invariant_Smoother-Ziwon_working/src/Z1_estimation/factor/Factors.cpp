//
// Created by Junny on  2020-06-29.
// Last Update on       2022-01-06.
//

#include "Factors.hpp"

using namespace Eigen;
using std::cout;
using std::endl;





void Inv_Factors::Batch_Initialize(ROBOT_STATES* _RS, const ROBOT_STATES &RS_Prior,
                                     const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                     factor_info* _fac_info,
                                     const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    fac_info = _fac_info;

    RS_temp = _RS;
//    RS_temp = new ROBOT_STATES[frame_count+1];
//    for(int i=0; i<=frame_count; i++){
//        RS_temp[i]  =_RS[i];
//    }


    //prior_RVP--------------------------------------------------------------------------------------
    X_Prior.resize(5,5);
    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_Prior.Rotation;
    X_Prior.block<3,1>(0,3) = RS_Prior.Velocity;
    X_Prior.block<3,1>(0,4) = RS_Prior.Position;

    SQRT_INFO_Prior.setZero();
    SQRT_INFO_Prior.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Orientation;
    SQRT_INFO_Prior.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Velocity;
    SQRT_INFO_Prior.block<3,3>(6,6) = robot.SQRT_INFO_Covariance_Prior_Position;

    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_Prior.Bias_Gyro, RS_Prior.Bias_Acc;

    SQRT_INFO_Prior_BIAS.setZero();
    SQRT_INFO_Prior_BIAS.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Bias_Gyro;
    SQRT_INFO_Prior_BIAS.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Bias_Acc;

}







void Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                  Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                  double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost != 0){



                for(int p=0; p<frame_count; p++){
                    std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                    int count=0;
                    for (int k=0; k<robot.leg_no; k++){
                        if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                            fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                            count++;

                        }
                    }
                }




        }

    }



    cost = 0;

    if( is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Invariant_Prior_RVP_Factor(Hessian, gradient, cost);

        Invariant_Prior_Bias_Factor(Hessian, gradient, cost);

        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }
        }
    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }





}



void Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                                        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                        Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                        Eigen::Matrix<double,4,1> &cost_log, Eigen::MatrixXd &Jacobian_Vector){



    big_jacobian_by_vector.resize(0,1);
    big_jacobian_by_vector.setZero();

    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost_log(0) != 0){



            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }




        }

    }


    cost_log.setZero();

    if( is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Invariant_Prior_RVP_Factor(Hessian, gradient, cost_log(1));

        Invariant_Prior_Bias_Factor(Hessian, gradient, cost_log(1));

        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost_log(2));
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost_log(3));

                }
            }
        }
    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        double garbage_cost=0;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, garbage_cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, garbage_cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }

    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);

//    //big_jacobian_by_vector = big_jacobian_by_vector/big_jacobian_by_vector.norm();
//
//    Jacobian_Vector.conservativeResize(big_jacobian_by_vector.rows(), Jacobian_Vector.cols()+1);
//    Jacobian_Vector.rightCols(1) = big_jacobian_by_vector;



}









void Inv_Factors::Marg_Initialize(ROBOT_STATES* _RS,
                                     const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                   const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                                   const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                                  factor_info* _fac_info,
                                     const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    marginalization_flag = true;

    fac_info = _fac_info;

    RS_temp = _RS;



    //prior_RVP--------------------------------------------------------------------------------------

    X_Prior.resize(5+RS_temp[0].contact_leg_num, 5+RS_temp[0].contact_leg_num);
    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_Prior.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_Prior.block<3,1>(0,4) = RS_temp[0].Position;


    int count=0;
    for (int k=0; k<robot.leg_no; k++)
    {
        if ( RS_temp[0].Hard_Contact(k)){
            X_Prior.block<3,1>(0,5 + count) = RS_temp[0].d.block(3*k,0, 3,1);
            count++;
        }
    }

    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;


    //Marginalization factor--------------------------------------------------------------------------

    Hessian_Marg.resizeLike(_H);
    Hessian_Marg = _H.transpose()*_H;
    H =_H;

    gradient_Marg.resizeLike(_b);
    gradient_Marg = _H.transpose()*_b;
    b = _b;

}







void Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                  Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                  double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost !=0){

                for(int p=0; p<frame_count; p++){
                    std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                    int count=0;
                    for (int k=0; k<robot.leg_no; k++){
                        if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                            fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                            count++;

                        }
                    }
                }



        }
    }




    cost = 0;

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, -1,1> Xi_0;
        Xi_0.resize(9+3*RS_temp[0].contact_leg_num,1);
        Xi_0 = Zeta_Xi.block(6,0,9+3*RS_temp[0].contact_leg_num,1);

        X_0_bar.resize(5+RS_temp[0].contact_leg_num, 5+RS_temp[0].contact_leg_num);
        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        int count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[0].Hard_Contact(k)){
                X_0_bar.block<3,1>(0,5 + count) = RS_temp[0].d.block(3*k,0, 3,1);
                count++;
            }
        }
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2+RS_temp[0].contact_leg_num)*X_0_bar;


        Eigen::Matrix<double,-1,1> Bias_X_bar_X_vee_inv_retracted;
        Bias_X_bar_X_vee_inv_retracted.resize(6+9+3*RS_temp[0].contact_leg_num,1);
        Bias_X_bar_X_vee_inv_retracted.setZero();
        Bias_X_bar_X_vee_inv_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted.block(6,0,9+3*RS_temp[0].contact_leg_num,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2+RS_temp[0].contact_leg_num);

        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b ;

        double temp_cost = ( b + H * Bias_X_bar_X_vee_inv_retracted ).norm();

//        //sqrt_info = H
//        //residual = H * (Bias_X_bar_X_vee_inv_retracted + r* ), where r* = (H*)^(-1) * (z*) = H^(-1) b
//        //partial = H*I
//        //Hessian = partial.T * partial = Hessian_Marg = H*
//        //gradient = partial.T * residual = (H*) * Bias~ + HT * b, where b = H-T * z*
//
//        Eigen::MatrixXd Inv_Left_Jacobian_e_Marg;
//        Inv_Left_Jacobian_e_Marg.resizeLike(Hessian_Marg);
//        Inv_Left_Jacobian_e_Marg.setZero();
//        Inv_Left_Jacobian_e_Marg.block(0,0, 6,6).setIdentity();
//        Inv_Left_Jacobian_e_Marg.block(6,6, 9+3*RS_temp[0].contact_leg_num,9+3*RS_temp[0].contact_leg_num) = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(Bias_X_bar_X_vee_inv_retracted, 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_e_Marg.transpose() * Hessian_Marg * Inv_Left_Jacobian_e_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_e_Marg.transpose() * (Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b) ;
//
//
//
//        Eigen::Matrix<double,-1,1> Bias_X_bar_X_Marg_inv_Exp_r_retracted;
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.resize(6+9+3*RS_temp[0].contact_leg_num,1);
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.setZero();
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.block(6,0,9+3*RS_temp[0].contact_leg_num,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse()*BasicFunctions_Estimator::Expm_seK_Vec(H.inverse()*b, 2+RS_temp[0].contact_leg_num), 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* Bias_X_bar_X_Marg_inv_Exp_r_retracted ;
//
//
//
//
//        Eigen::MatrixXd Inv_Left_Jacobian_minus_e_Marg_Exp_r;
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.resizeLike(Hessian_Marg);
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.setZero();
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.block(0,0, 6,6).setIdentity();
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.block(6,6, 9+3*RS_temp[0].contact_leg_num,9+3*RS_temp[0].contact_leg_num) = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(Bias_X_bar_X_Marg_inv_Exp_r_retracted, 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_minus_e_Marg_Exp_r.transpose() * Hessian_Marg * Inv_Left_Jacobian_minus_e_Marg_Exp_r;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_minus_e_Marg_Exp_r.transpose() * Hessian_Marg * Bias_X_bar_X_Marg_inv_Exp_r_retracted ;
//
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_e_Marg.transpose() * Hessian_Marg * Inv_Left_Jacobian_e_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_e_Marg.transpose() * (Hessian_Marg* H * (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b );
//
//        double temp_cost = ( b + H * H * Bias_X_bar_X_vee_inv_retracted ).norm();



        cost += temp_cost*temp_cost;

        //cout<<"Now marg res is "<<cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }
        }



        for (int k=0; k<robot.leg_no; k++) {

            for (int xy_z = 1; xy_z <3; xy_z++) {

                double v_threshold = 0;
                double a_threshold = 0;

                if(xy_z == 1){
                    v_threshold = robot.long_term_v_threshold;
                    a_threshold = robot.long_term_a_threshold;
                }else if(xy_z == 2){
                    v_threshold = robot.long_term_v_threshold;
                    a_threshold = robot.long_term_a_threshold;
                }

                int start = 0, end = -1;
                int contact_count = 0;

                for (int i = frame_count; i > 1; i--) {

                    double dv_mag_i = 0;
                    double da_mag_i = 0;
                    if(xy_z == 1){
                        dv_mag_i = fabs(RS_temp[i].d_v.block(3*k,0, 2,1).norm() );
                        da_mag_i = fabs( (RS_temp[i].d_v.block(3*k,0, 2,1) -RS_temp[i-1].d_v.block(3*k,0, 2,1) ).norm()/robot.dt );
                    }else if(xy_z == 2){
                        dv_mag_i = fabs(RS_temp[i].d_v(3 * k + xy_z));
                        da_mag_i = fabs( (RS_temp[i].d_v(3 * k + xy_z) - RS_temp[i-1].d_v(3 * k + xy_z))/robot.dt );
                    }

                    //cout<<i<<endl;
                    if (contact_count == 0 && RS_temp[i].Hard_Contact(k)
                                && (dv_mag_i < v_threshold)
                                && ( da_mag_i < a_threshold)
                                ) {
                        end = i;
                        contact_count++;

                    } else if (contact_count > 0 && RS_temp[i].Hard_Contact(k)
                               && (dv_mag_i < v_threshold)
                               && ( da_mag_i < a_threshold)
                                ) {

                        contact_count++;

                    } else if (contact_count > 0 && (
                            !RS_temp[i].Hard_Contact(k)
                            || (dv_mag_i > v_threshold)
                            || (da_mag_i > a_threshold)
                            ) ) {
                        contact_count = 0;

                        start = i+1;
                    }

                    if ((start > 0) && (end - start > 1) ) {

                        Long_Term_Stationary_Foot_Factor(start, end, xy_z, k, Hessian, gradient, cost);


                        start = 0;
                        end = -1;

                    }

                }
            }

        }



    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }



}




void Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                                       Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                       Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                       Eigen::Matrix<double,4,1> &cost_log){


    big_jacobian_by_vector.resize(0,1);
    big_jacobian_by_vector.setZero();

    if(is_for_marginalization == false){

        RS_temp = _RS;

        if( cost_log(0) !=0){

            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }



        }
    }




    cost_log.setZero();

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, -1,1> Xi_0;
        Xi_0.resize(9+3*RS_temp[0].contact_leg_num,1);
        Xi_0 = Zeta_Xi.block(6,0,9+3*RS_temp[0].contact_leg_num,1);

        X_0_bar.resize(5+RS_temp[0].contact_leg_num, 5+RS_temp[0].contact_leg_num);
        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        int count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[0].Hard_Contact(k)){
                X_0_bar.block<3,1>(0,5 + count) = RS_temp[0].d.block(3*k,0, 3,1);
                count++;
            }
        }
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2+RS_temp[0].contact_leg_num)*X_0_bar;


        Eigen::Matrix<double,-1,1> Bias_X_bar_X_vee_inv_retracted;
        Bias_X_bar_X_vee_inv_retracted.resize(6+9+3*RS_temp[0].contact_leg_num,1);
        Bias_X_bar_X_vee_inv_retracted.setZero();
        Bias_X_bar_X_vee_inv_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted.block(6,0,9+3*RS_temp[0].contact_leg_num,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2+RS_temp[0].contact_leg_num);

        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b ;

        double temp_cost = ( b + H * Bias_X_bar_X_vee_inv_retracted ).norm();

//        //sqrt_info = H
//        //residual = H * (Bias_X_bar_X_vee_inv_retracted + r* ), where r* = (H*)^(-1) * (z*) = H^(-1) b
//        //partial = H*I
//        //Hessian = partial.T * partial = Hessian_Marg = H* = HT * H
//        //gradient = partial.T * residual = (H*) * Bias~ + HT * b, where b = H-T * z*
//
//        Eigen::MatrixXd Inv_Left_Jacobian_e_Marg;
//        Inv_Left_Jacobian_e_Marg.resizeLike(Hessian_Marg);
//        Inv_Left_Jacobian_e_Marg.setZero();
//        Inv_Left_Jacobian_e_Marg.block(0,0, 6,6).setIdentity();
//        Inv_Left_Jacobian_e_Marg.block(6,6, 9+3*RS_temp[0].contact_leg_num,9+3*RS_temp[0].contact_leg_num) = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(Bias_X_bar_X_vee_inv_retracted, 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_e_Marg.transpose() * Hessian_Marg * Inv_Left_Jacobian_e_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_e_Marg.transpose() * (Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b) ;
//
//
//
//        Eigen::Matrix<double,-1,1> Bias_X_bar_X_Marg_inv_Exp_r_retracted;
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.resize(6+9+3*RS_temp[0].contact_leg_num,1);
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.setZero();
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
//        Bias_X_bar_X_Marg_inv_Exp_r_retracted.block(6,0,9+3*RS_temp[0].contact_leg_num,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse()*BasicFunctions_Estimator::Expm_seK_Vec(H.inverse()*b, 2+RS_temp[0].contact_leg_num), 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* Bias_X_bar_X_Marg_inv_Exp_r_retracted ;
//
//
//
//
//        Eigen::MatrixXd Inv_Left_Jacobian_minus_e_Marg_Exp_r;
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.resizeLike(Hessian_Marg);
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.setZero();
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.block(0,0, 6,6).setIdentity();
//        Inv_Left_Jacobian_minus_e_Marg_Exp_r.block(6,6, 9+3*RS_temp[0].contact_leg_num,9+3*RS_temp[0].contact_leg_num) = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(Bias_X_bar_X_Marg_inv_Exp_r_retracted, 2+RS_temp[0].contact_leg_num);
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_minus_e_Marg_Exp_r.transpose() * Hessian_Marg * Inv_Left_Jacobian_minus_e_Marg_Exp_r;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_minus_e_Marg_Exp_r.transpose() * Hessian_Marg * Bias_X_bar_X_Marg_inv_Exp_r_retracted ;
//
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* H * (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b ;
//
//        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Inv_Left_Jacobian_e_Marg.transpose() * Hessian_Marg * Inv_Left_Jacobian_e_Marg;
//        gradient.block(0,0, RS_temp[0].para_size, 1) = Inv_Left_Jacobian_e_Marg.transpose() * (Hessian_Marg* H * (Bias_X_bar_X_vee_inv_retracted) + H.transpose()*b );
//
//
//
//
//        double temp_cost = ( b + H * H * Bias_X_bar_X_vee_inv_retracted ).norm();



        cost_log(1) = temp_cost*temp_cost;


//        double new_rows_no;
//
//        new_rows_no = H.cols()*H.rows();
//        VectorXd temp(Eigen::Map<VectorXd>(H.data(),new_rows_no));
//        big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//        big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

        //cout<<"Now marg res is "<<cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost_log(2));
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost_log(3));

                }
            }
        }



        for (int k=0; k<robot.leg_no; k++) {

            for (int xy_z = 1; xy_z <3; xy_z++) {

                double v_threshold = 0;
                double a_threshold = 0;

                if(xy_z == 1){
                    v_threshold = robot.long_term_v_threshold;
                    a_threshold = robot.long_term_a_threshold;
                }else if(xy_z == 2){
                    v_threshold = robot.long_term_v_threshold;
                    a_threshold = robot.long_term_a_threshold;
                }

                int start = 0, end = -1;
                int contact_count = 0;

                for (int i = frame_count; i > 1; i--) {

                    double dv_mag_i = 0;
                    double da_mag_i = 0;
                    if(xy_z == 1){
                        dv_mag_i = fabs(RS_temp[i].d_v.block(3*k,0, 2,1).norm() );
                        da_mag_i = fabs( (RS_temp[i].d_v.block(3*k,0, 2,1) -RS_temp[i-1].d_v.block(3*k,0, 2,1) ).norm()/robot.dt );
                    }else if(xy_z == 2){
                        dv_mag_i = fabs(RS_temp[i].d_v(3 * k + xy_z));
                        da_mag_i = fabs( (RS_temp[i].d_v(3 * k + xy_z) - RS_temp[i-1].d_v(3 * k + xy_z))/robot.dt );
                    }

                    //cout<<i<<endl;
                    if (contact_count == 0 && RS_temp[i].Hard_Contact(k)
                        && (dv_mag_i < v_threshold)
                        && ( da_mag_i < a_threshold)
                            ) {
                        end = i;
                        contact_count++;

                    } else if (contact_count > 0 && RS_temp[i].Hard_Contact(k)
                               && (dv_mag_i < v_threshold)
                               && ( da_mag_i < a_threshold)
                            ) {

                        contact_count++;

                    } else if (contact_count > 0 && (
                            !RS_temp[i].Hard_Contact(k)
                            || (dv_mag_i > v_threshold)
                            || (da_mag_i > a_threshold)
                    ) ) {
                        contact_count = 0;

                        start = i+1;
                    }

                    if ((start > 0) && (end - start > 1) ) {

                        Long_Term_Stationary_Foot_Factor(start, end, xy_z, k, Hessian, gradient, cost_log(0));


                        start = 0;
                        end = -1;

                    }

                }
            }

        }



    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        double garbage_cost=0;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, garbage_cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, garbage_cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }

    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);


}




void Inv_Factors::Invariant_Propagation_Factor(int i,
        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
        Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    int j=i+1;

    Eigen::MatrixXd A;
    A.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    A.setZero();


    A.block(9,6, 3,3) = BasicFunctions_Estimator::Hat_so3(gravity);
    A.block(12,6, 3,3) = 0.5 * BasicFunctions_Estimator::Hat_so3(gravity) * dt;
    A.block(12,9, 3,3) = Matrix3d::Identity();


    A.block(6, 0, 3,3) = - RS_temp[i].Rotation;

    A.block(9, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Velocity ) * RS_temp[i].Rotation;
    A.block(9, 3, 3,3) = - RS_temp[i].Rotation;

    A.block(12, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Position ) * RS_temp[i].Rotation;


    int count=0;
    int count_i=0;
    for (int k=0; k<robot.leg_no; k++) {
        if ( RS_temp[i].Hard_Contact(k) ){
            if ( RS_temp[j].Hard_Contact(k) ){

                Eigen::Vector3d d;
                d = RS_temp[i].d.block(3*k,0,3,1);
                A.block(15 + 3*count, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( d ) * RS_temp[i].Rotation;
                count++;
            }

            count_i++;
        }
    }

    //Covariance Calculation-------------------------------------------------------------------------------
    //Adjoint of X_bar---------------------------------------------------------------------

    Eigen::MatrixXd Adx_inv_augmented;

    Adx_inv_augmented.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    Adx_inv_augmented.setZero();


    Adx_inv_augmented.block(0,0, 6,6).setIdentity();

    for(int k=0; k<3+fac_info[i].shared_contact; k++){
        Adx_inv_augmented.block<3,3>(6+3*k, 6+3*k) = RS_temp[i].Rotation.transpose();
    }

    Adx_inv_augmented.block<3,3>(9,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Velocity );
    Adx_inv_augmented.block<3,3>(12,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Position );



    count=0;
    count_i=0;
    for (int k=0; k<robot.leg_no; k++) {
        if ( RS_temp[i].Hard_Contact(k) ){
            if ( RS_temp[j].Hard_Contact(k) ){

                Eigen::Vector3d d;
                d = RS_temp[i].d.block(3*k,0,3,1);
                Adx_inv_augmented.block<3,3>(15 + 3 *count, 6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3( d );
                count++;
            }

            count_i++;
        }
    }


    Eigen::MatrixXd sqrt_info;
    sqrt_info.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    sqrt_info = fac_info[i].prop_primitive_sqrt_info;

    if(fac_info[i].prop_primitive_sqrt_info.cols() >15){
//std::cout << "prop sqrt_info "<< sqrt_info(15,15) <<" at "<<i<< std::endl;
    }


    sqrt_info = sqrt_info * Adx_inv_augmented/dt;

    //residual---------------------------------------------------------------------------------------

    Eigen::VectorXd residual;
    residual.resize(fac_info[i].prop_res_size, 1);

    Eigen::MatrixXd f_X_i;
    f_X_i.resize(5+fac_info[i].shared_contact, 5+fac_info[i].shared_contact);
    f_X_i.setIdentity();

    Eigen::Vector3d IMU_Gyro_i, IMU_Acc_i;
    IMU_Gyro_i = fac_info[i].Z.block(0,0,3,1);
    IMU_Acc_i = fac_info[i].Z.block(3,0,3,1);


    Eigen::Matrix<double,-1,-1> jacobian_left_inv;
    jacobian_left_inv.resize(9+3*fac_info[i].shared_contact, 9+3*fac_info[i].shared_contact);
    jacobian_left_inv.setZero();


    if(i==0 && marginalization_flag){

        Eigen::Matrix3d R0;
        Eigen::Vector3d v0, p0, bg0, ba0;
        R0 = X_0_bar.block(0,0,3,3);
        v0 = X_0_bar.block(0,3,3,1);
        p0 = X_0_bar.block(0,4,3,1);
        bg0 = Bias_0_bar.block(0,0,3,1);
        ba0 = Bias_0_bar.block(3,0,3,1);

        f_X_i.block<3,3>(0,0) = R0 * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - bg0)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = v0
                + R0 //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                * (IMU_Acc_i - ba0)* dt
                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  p0 + v0 * dt
                + 0.5*R0 * (IMU_Acc_i - ba0)* dt*dt + 0.5*gravity*dt*dt;

        count_i =0;
        count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k)){
                if ( RS_temp[j].Hard_Contact(k) ){
                    f_X_i.block<3,1>(0,5 + count) = X_0_bar.block(0,5+count_i,3,1);
                    count++;
                }
                count_i++;
            }
        }



        Eigen::MatrixXd X_j_bar;
        X_j_bar.resize(5+fac_info[i].shared_contact, 5+fac_info[i].shared_contact);
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;

        count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){
                X_j_bar.block<3,1>(0,5 + count) = RS_temp[j].d.block(3*k,0, 3,1);
                count++;
            }
        }


        Eigen::Matrix<double,-1,1> Delta;
        Delta.resize(9+3*fac_info[i].shared_contact,1);
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2+fac_info[i].shared_contact);


        //jacobian_left_inv = BasicFunctions_Estimator::Inv_Right_Jacobian_SEk(Delta, 2+fac_info[i].shared_contact);
        jacobian_left_inv = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(-Delta, 2+fac_info[i].shared_contact);



        residual.block(0,0,3,1 ) = bg0 - RS_temp[j].Bias_Gyro;
        residual.block(3,0,3,1 ) = ba0 - RS_temp[j].Bias_Acc;
        residual.block(6,0,fac_info[i].prop_res_size-6,1 ) = Delta;


    }else{


        f_X_i.block<3,3>(0,0) = RS_temp[i].Rotation * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - RS_temp[i].Bias_Gyro)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = RS_temp[i].Velocity
                + RS_temp[i].Rotation //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                * (fac_info[i].Z.block(3,0,3,1) - RS_temp[i].Bias_Acc)* dt
                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  RS_temp[i].Position + RS_temp[i].Velocity * dt
                + 0.5*RS_temp[i].Rotation * (fac_info[i].Z.block(3,0,3,1) - RS_temp[i].Bias_Acc)* dt*dt + 0.5*gravity*dt*dt;


        count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){
                f_X_i.block<3,1>(0,5 + count) = RS_temp[i].d.block(3*k,0, 3,1);
                count++;
            }
        }


        Eigen::MatrixXd X_j_bar;
        X_j_bar.resize(5+fac_info[i].shared_contact, 5+fac_info[i].shared_contact);
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;


        count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){
                X_j_bar.block<3,1>(0,5 + count) = RS_temp[j].d.block(3*k,0, 3,1);
                count++;
            }
        }


        Eigen::Matrix<double,-1,1> Delta;
        Delta.resize(9+3*fac_info[i].shared_contact,1);
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2+fac_info[i].shared_contact);

        //jacobian_left_inv = BasicFunctions_Estimator::Inv_Right_Jacobian_SEk(Delta, 2+fac_info[i].shared_contact);
        jacobian_left_inv = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(-Delta, 2+fac_info[i].shared_contact);


        residual.block(0,0,3,1 ) = RS_temp[i].Bias_Gyro - RS_temp[j].Bias_Gyro;
        residual.block(3,0,3,1 ) = RS_temp[i].Bias_Acc - RS_temp[j].Bias_Acc;
        residual.block(6,0,fac_info[i].prop_res_size-6,1 ) = Delta;

    }

    if(residual.rows() >15){
//std::cout << "prop res "<< residual.block(15,0,3,1).transpose() <<" at "<<i<< std::endl;
    }


    residual = sqrt_info * residual;
    cost += residual.transpose()*residual;


    //cout<<"now prop residual is "<<residual.transpose()*residual<<endl;



    //Jacobian-------------------------------------------------------------------------

    Eigen::MatrixXd I;
    I.setIdentity(fac_info[i].prop_res_size, fac_info[i].prop_res_size);

    Eigen::MatrixXd IAdt;
    IAdt.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    IAdt = (I + A*dt);

    Eigen::MatrixXd partial_i;
    partial_i.resize(fac_info[i].prop_res_size, fac_info[i].prop_para0_size + 6);
    partial_i.block(0,0, fac_info[i].prop_res_size,6) = ( IAdt  ).block(0,0, fac_info[i].prop_res_size,6);
    partial_i.block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_para0_size) = ( IAdt ).block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_res_size-6) * fac_info[i].Mi;
    partial_i = sqrt_info * partial_i;

    Eigen::MatrixXd partial_j;
    partial_j.resize(fac_info[i].prop_res_size, fac_info[i].prop_para1_size + 6);
    partial_j.block(0,0, fac_info[i].prop_res_size,6)=( -I ).block(0,0, fac_info[i].prop_res_size,6);
    partial_j.block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_para1_size)=( -I ).block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_res_size-6) * jacobian_left_inv * fac_info[i].Mj;
    //partial_j.block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_para1_size)=( -I ).block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_res_size-6) * fac_info[i].Mj;
    partial_j = sqrt_info * partial_j;


    //Hessian-------------------------------------------------------------------------------------------

    Hessian.block(RS_temp[i].para_idx,RS_temp[i].para_idx, RS_temp[i].para_size,RS_temp[i].para_size) += partial_i.transpose() * partial_i;
    Hessian.block(RS_temp[j].para_idx,RS_temp[j].para_idx, RS_temp[j].para_size,RS_temp[j].para_size) += partial_j.transpose() * partial_j;
    Hessian.block(RS_temp[i].para_idx,RS_temp[j].para_idx, RS_temp[i].para_size,RS_temp[j].para_size) += partial_i.transpose() * partial_j;
    Hessian.block(RS_temp[j].para_idx,RS_temp[i].para_idx, RS_temp[j].para_size,RS_temp[i].para_size) += partial_j.transpose() * partial_i;

    //gradient-------------------------------------------------------------------------------------------
    gradient.block(RS_temp[i].para_idx,0, RS_temp[i].para_size,1) += partial_i.transpose() * residual;
    gradient.block(RS_temp[j].para_idx,0, RS_temp[j].para_size,1) += partial_j.transpose() * residual;


//    double new_rows_no;
//
//    new_rows_no = partial_i.cols()*partial_i.rows();
//    VectorXd temp(Eigen::Map<VectorXd>(partial_i.data(),new_rows_no));
//    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;
//
//    new_rows_no = partial_j.cols()*partial_j.rows();
//    VectorXd temp2(Eigen::Map<VectorXd>(partial_j.data(),new_rows_no));
//    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//    big_jacobian_by_vector.bottomRows(new_rows_no) = temp2;


}




//***************************Faster Meas factor********************************

void Inv_Factors::Invariant_Measurement_Factor(int j, int leg_num,
                                                    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                    Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    kinematics_info kin_k = *fac_info[j].leg_info.find(kinematics_info(leg_num));
    // Calculating Covariance--------------------------------------------------------------
    Eigen::Matrix3d sqrt_info;
    sqrt_info = kin_k.meas_primitive_sqrt_info * RS_temp[j].Rotation.transpose();
    //residual------------------------------------------------------------------------------
    Eigen::Vector3d residual;



    if(j==0 && marginalization_flag){
        residual = X_0_bar.block(0,0,3,3) * kin_k.fk_kin + X_0_bar.block(0,4,3,1) - X_0_bar.block(0,5+kin_k.leg_num_in_state,3,1);
    }else{
        residual = ( RS_temp[j].Rotation * kin_k.fk_kin + RS_temp[j].Position - RS_temp[j].d.block(3*leg_num,0,3,1) );// = R*hp +p -d
    }
    Eigen::Vector3d residual_t = residual;
    residual = sqrt_info * residual;

    cost += residual.transpose()*residual;
    //cout<<"now meas residual is "<<residual.transpose()*residual<<endl;

    //Jacobians--------------------------------------------------------------------------------------

    Eigen::MatrixXd partial_xi_i;
    partial_xi_i.resize(3, fac_info[j].meas_para_size);
    partial_xi_i.setZero();

    //partial_xi_i.block(0,0, 3,3) = -sqrt_info * BasicFunctions_Estimator::Hat_so3(residual_t);
    partial_xi_i.block(0,6, 3,3) = sqrt_info;
    partial_xi_i.block(0, 9 + 3*(kin_k.leg_num_in_state), 3,3) = -sqrt_info;



    //Hessian and Gradient
    gradient.block(RS_temp[j].para_idx+6,0, RS_temp[j].para_size-6,1) += partial_xi_i.transpose() * residual;
    Hessian.block(RS_temp[j].para_idx+6, RS_temp[j].para_idx+6, RS_temp[j].para_size-6, RS_temp[j].para_size-6) += partial_xi_i.transpose() * partial_xi_i;



//    double new_rows_no;
//
//    new_rows_no = partial_xi_i.cols()*partial_xi_i.rows();
//    VectorXd temp(Eigen::Map<VectorXd>(partial_xi_i.data(),new_rows_no));
//    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

}





//***************************Faster Meas factor********************************

void Inv_Factors::Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
                                                   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost)
{

    kinematics_info kin_k_start = *fac_info[start].leg_info.find(kinematics_info(leg_num));
    kinematics_info kin_k_end = *fac_info[end].leg_info.find(kinematics_info(leg_num));




    // Calculating Covariance--------------------------------------------------------------
    Eigen::Matrix3d sqrt_info;
    sqrt_info.setIdentity();
    sqrt_info = sqrt_info * 0.01;

    double magnitude = 1/sqrt(robot.cov_contact_const)/(robot.dt);
//                      1/sqrt(robot.cov_contact_const)/((end-start)*robot.dt);

    if(xyz == 1){
        sqrt_info(0,0) = magnitude;
        sqrt_info(1,1) = magnitude;
    }else if(xyz == 2){
        sqrt_info(2,2) = magnitude;
    }



    //residual------------------------------------------------------------------------------
    Eigen::Vector3d residual;
    residual.setZero();

    residual = RS_temp[start].d.block(3*leg_num,0,3,1) - RS_temp[end].d.block(3*leg_num,0,3,1);
    residual = sqrt_info * residual;

    cost += residual.transpose()*residual;

    //Jacobians--------------------------------------------------------------------------------------

    Eigen::MatrixXd partial_xi_i;
    partial_xi_i.resize(3, fac_info[start].meas_para_size);
    partial_xi_i.setZero();
    partial_xi_i.block(0, 9 + 3*(kin_k_start.leg_num_in_state), 3,3) = sqrt_info;

    Eigen::MatrixXd partial_xi_j;
    partial_xi_j.resize(3, fac_info[end].meas_para_size);
    partial_xi_j.setZero();
    partial_xi_j.block(0, 9 + 3*(kin_k_end.leg_num_in_state), 3,3) = -sqrt_info;

    //Hessian and Gradient
    gradient.block(RS_temp[start].para_idx+6,0, RS_temp[start].para_size-6,1) += partial_xi_i.transpose() * residual;
    Hessian.block(RS_temp[start].para_idx+6, RS_temp[start].para_idx+6, RS_temp[start].para_size-6, RS_temp[start].para_size-6) += partial_xi_i.transpose() * partial_xi_i;

    gradient.block(RS_temp[end].para_idx+6,0, RS_temp[end].para_size-6,1) += partial_xi_j.transpose() * residual;
    Hessian.block(RS_temp[end].para_idx+6, RS_temp[end].para_idx+6, RS_temp[end].para_size-6, RS_temp[end].para_size-6) += partial_xi_j.transpose() * partial_xi_j;



}




void Inv_Factors::Invariant_Prior_RVP_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                   Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    Eigen::Matrix<double,5,5> X_hat_current_est;
    X_hat_current_est.setIdentity();

    X_hat_current_est.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_hat_current_est.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_hat_current_est.block<3,1>(0,4) = RS_temp[0].Position;

    //----------------------------------------------------------------

    Eigen::Matrix<double,9,1> X_pri_X_hat_inv_retracted;
    X_pri_X_hat_inv_retracted.setZero();
    X_pri_X_hat_inv_retracted = BasicFunctions_Estimator::Logm_seK_Vec(X_hat_current_est*X_Prior.inverse(), 2);

    Eigen::Matrix<double,9,9> jacobian_left_inv;
    jacobian_left_inv.setZero();

    jacobian_left_inv = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(X_pri_X_hat_inv_retracted, 2);

    //residual---------------------------------------------------------------------------------------
    Eigen::Matrix<double,9,1> residual;
    residual.setZero();
    residual = SQRT_INFO_Prior * (X_pri_X_hat_inv_retracted);



    cost += residual.transpose()*residual;
    //std::cout<<"now prior residual is "<<residual.norm()<<std::endl;

    //Jacobians---------------------------------------------------------------------------------
    Eigen::Matrix<double,9,9> partial_xi_0;
    partial_xi_0.setZero();
    partial_xi_0 = SQRT_INFO_Prior * jacobian_left_inv;
    //partial_xi_0 = SQRT_INFO_Prior;



    //Hessian, gradient---------------------------------------------------------------------------
    Hessian.block(6,6, 9, 9) += partial_xi_0.transpose() * partial_xi_0;
    gradient.block(6,0,9,1) += partial_xi_0.transpose() * residual;



//    double new_rows_no;
//
//    new_rows_no = partial_xi_0.cols()*partial_xi_0.rows();
//    VectorXd temp(Eigen::Map<VectorXd>(partial_xi_0.data(),new_rows_no));
//
//    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

}









void Inv_Factors::Invariant_Prior_Bias_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                    Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    //residual---------------------------------------------------------------------------
    Eigen::Matrix<double,6,1> residual;
    residual.setZero();

    residual << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;
    residual = residual - Bias_Prior;

    residual = SQRT_INFO_Prior_BIAS * ( residual);

    cost += residual.transpose()*residual;
    //cout<<"now prior bias residual is "<<esidual.norm()<<endl;

    Eigen::Matrix<double,6,6> partial_bias_0;
    partial_bias_0.setIdentity();
    partial_bias_0 = SQRT_INFO_Prior_BIAS * partial_bias_0;

    //Hessian, gradient-----------------------------------------------------------------------------
    Hessian.block(0,0,6,6) += partial_bias_0.transpose() * partial_bias_0;
    gradient.block(0,0, 6,1) += partial_bias_0.transpose() * residual;


//    double new_rows_no;
//
//    new_rows_no = partial_bias_0.cols()*partial_bias_0.rows();
//    VectorXd temp(Eigen::Map<VectorXd>(partial_bias_0.data(),new_rows_no));
//    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
//    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

}












void Adv_Inv_Factors::Batch_Initialize(ROBOT_STATES* _RS, const ROBOT_STATES &RS_Prior,
                                   const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                   factor_info* _fac_info,
                                   const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    fac_info = _fac_info;

    RS_temp = _RS;
//    RS_temp = new ROBOT_STATES[frame_count+1];
//    for(int i=0; i<=frame_count; i++){
//        RS_temp[i]  =_RS[i];
//    }


    //prior_RVP--------------------------------------------------------------------------------------
    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_Prior.Rotation;
    X_Prior.block<3,1>(0,3) = RS_Prior.Velocity;
    X_Prior.block<3,1>(0,4) = RS_Prior.Position;

    SQRT_INFO_Prior.setZero();
    SQRT_INFO_Prior.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Orientation;
    SQRT_INFO_Prior.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Velocity;
    SQRT_INFO_Prior.block<3,3>(6,6) = robot.SQRT_INFO_Covariance_Prior_Position;


    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_Prior.Bias_Gyro, RS_Prior.Bias_Acc;

    SQRT_INFO_Prior_BIAS.setZero();
    SQRT_INFO_Prior_BIAS.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Bias_Gyro;
    SQRT_INFO_Prior_BIAS.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Bias_Acc;

}







void Adv_Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                                        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                        Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                        double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost != 0){



            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }




        }

    }



    cost = 0;

    if( is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Invariant_Prior_RVP_Factor(Hessian, gradient, cost);

        Invariant_Prior_Bias_Factor(Hessian, gradient, cost);

        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }
        }
    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }





}




void Adv_Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS,
                                                            Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                            Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                            Eigen::Matrix<double,4,1> &cost_log, Eigen::MatrixXd &Jacobian_Vector){

    big_jacobian_by_vector.resize(0,1);
    big_jacobian_by_vector.setZero();

    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost_log(0) != 0){



            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }




        }

    }



    cost_log.setZero();

    if( is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Invariant_Prior_RVP_Factor(Hessian, gradient, cost_log(1));

        Invariant_Prior_Bias_Factor(Hessian, gradient, cost_log(1));

        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost_log(2));
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost_log(3));

                }
            }
        }
    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        double garbage_cost=0;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, garbage_cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, garbage_cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }



    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);


//    //big_jacobian_by_vector = big_jacobian_by_vector/big_jacobian_by_vector.norm();
//
//    Jacobian_Vector.conservativeResize(big_jacobian_by_vector.rows(), Jacobian_Vector.cols()+1);
//    Jacobian_Vector.rightCols(1) = big_jacobian_by_vector;

}







void Adv_Inv_Factors::Marg_Initialize(ROBOT_STATES* _RS,
                                  const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                  const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                                  const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                                  factor_info* _fac_info,
                                  const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    marginalization_flag = true;

    fac_info = _fac_info;

    RS_temp = _RS;



    //prior_RVP--------------------------------------------------------------------------------------
    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_Prior.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_Prior.block<3,1>(0,4) = RS_temp[0].Position;

    //prior_d--------------------------------------------------------------------------------------
    d_Prior.resize(3*RS_temp[0].contact_leg_num,1);
    d_Prior.setZero();

    int count=0;
    for (int k=0; k<robot.leg_no; k++)
    {
        if ( RS_temp[0].Hard_Contact(k)){
            d_Prior.block<3,1>(3*count,0) = RS_temp[0].d.block(3*k,0, 3,1);
            count++;
        }
    }

    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;


    //Marginalization factor--------------------------------------------------------------------------

    Hessian_Marg.resizeLike(_H);
    Hessian_Marg = _H.transpose()*_H;
    H =_H;

    gradient_Marg.resizeLike(_b);
    gradient_Marg = _H.transpose()*_b;
    b = _b;

}







void Adv_Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                                       Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                       Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                       double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost !=0){

            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }



        }
    }




    cost = 0;

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, 9,1> Xi_0;
        Xi_0 = Zeta_Xi.block(6,0, 9,1);

        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2)*X_0_bar;

        d_0_bar.resize(3*RS_temp[0].contact_leg_num,1);
        int count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[0].Hard_Contact(k)){
                d_0_bar.block<3,1>(3*count,0) = RS_temp[0].d.block(3*k,0, 3,1);
                count++;
            }
        }
        d_0_bar = d_0_bar + Zeta_Xi.block(15,0, 3*RS_temp[0].contact_leg_num,1);


        Eigen::Matrix<double,-1,1> Bias_X_bar_X_vee_inv_retracted_d;
        Bias_X_bar_X_vee_inv_retracted_d.resize(6+9+3*RS_temp[0].contact_leg_num,1);
        Bias_X_bar_X_vee_inv_retracted_d.setZero();
        Bias_X_bar_X_vee_inv_retracted_d.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted_d.block(6,0,9,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2);
        Bias_X_bar_X_vee_inv_retracted_d.block(15,0,3*RS_temp[0].contact_leg_num,1) = d_0_bar - d_Prior;

        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted_d) + H.transpose()*b ;

        double temp_cost = ( b + H * Bias_X_bar_X_vee_inv_retracted_d ).norm();



        cost += temp_cost*temp_cost;

        //cout<<"Now marg cost is "<<cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }
        }


    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }



}





void Adv_Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES* _RS, Eigen::Matrix<double,-1,1> Zeta_Xi,
                                                           Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                           Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                           Eigen::Matrix<double,4,1> &cost_log){

    big_jacobian_by_vector.resize(0,1);
    big_jacobian_by_vector.setZero();


    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost_log(0) !=0){

            for(int p=0; p<frame_count; p++){
                std::vector<double> contact_cov_array_temp = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                int count=0;
                for (int k=0; k<robot.leg_no; k++){
                    if( RS_temp[p].Hard_Contact(k) && RS_temp[p+1].Hard_Contact(k) ){

                        fac_info[p].prop_primitive_sqrt_info.block<3,3>(15 + 3*count, 15 + 3*count) = Eigen::MatrixXd::Identity(3,3) / sqrt(contact_cov_array_temp[k]);
                        count++;

                    }
                }
            }



        }
    }




    cost_log.setZero();

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, 9,1> Xi_0;
        Xi_0 = Zeta_Xi.block(6,0, 9,1);

        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2)*X_0_bar;

        d_0_bar.resize(3*RS_temp[0].contact_leg_num,1);
        int count=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[0].Hard_Contact(k)){
                d_0_bar.block<3,1>(3*count,0) = RS_temp[0].d.block(3*k,0, 3,1);
                count++;
            }
        }
        d_0_bar = d_0_bar + Zeta_Xi.block(15,0, 3*RS_temp[0].contact_leg_num,1);


        Eigen::Matrix<double,-1,1> Bias_X_bar_X_vee_inv_retracted_d;
        Bias_X_bar_X_vee_inv_retracted_d.resize(6+9+3*RS_temp[0].contact_leg_num,1);
        Bias_X_bar_X_vee_inv_retracted_d.setZero();
        Bias_X_bar_X_vee_inv_retracted_d.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted_d.block(6,0,9,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2);
        Bias_X_bar_X_vee_inv_retracted_d.block(15,0,3*RS_temp[0].contact_leg_num,1) = d_0_bar - d_Prior;

        Hessian.block(0,0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
        gradient.block(0,0, RS_temp[0].para_size, 1) = Hessian_Marg* (Bias_X_bar_X_vee_inv_retracted_d) + H.transpose()*b ;

        double temp_cost = ( b + H * Bias_X_bar_X_vee_inv_retracted_d ).norm();

        cost_log(1) = temp_cost*temp_cost;


        double new_rows_no;

        new_rows_no = H.cols()*H.rows();
        VectorXd temp(Eigen::Map<VectorXd>(H.data(),new_rows_no));
        big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
        big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

        //cout<<"Now marg cost is "<<cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost_log(2));
        }

        for (int i=0; i<=frame_count; i++){
            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost_log(3));

                }
            }
        }


    }else{

        Hessian = -Hessian;
        gradient = -gradient;
        double garbage_cost=0;

        for (int k=0; k<robot.leg_no; k++)
        {
            if (RS_temp[1].Hard_Contact(k) ){

                Invariant_Measurement_Factor(1, k, Hessian, gradient, garbage_cost);

            }
        }

        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, garbage_cost);

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }

    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);


}









void Adv_Inv_Factors::Invariant_Propagation_Factor(int i,
                                               Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    int j=i+1;

    Eigen::MatrixXd A;
    A.setZero(fac_info[i].prop_res_size, fac_info[i].prop_res_size);


    A.block(9,6, 3,3) = BasicFunctions_Estimator::Hat_so3(gravity);
    A.block(12,6, 3,3) = 0.5 * BasicFunctions_Estimator::Hat_so3(gravity) * dt;
    A.block(12,9, 3,3) = Matrix3d::Identity();


    A.block(6, 0, 3,3) = - RS_temp[i].Rotation;

    A.block(9, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Velocity ) * RS_temp[i].Rotation;
    A.block(9, 3, 3,3) = - RS_temp[i].Rotation;

    A.block(12, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Position ) * RS_temp[i].Rotation;


    //Covariance Calculation-------------------------------------------------------------------------------
    //Adjoint of X_bar---------------------------------------------------------------------

    Eigen::MatrixXd Adx_inv_augmented;

    Adx_inv_augmented.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    Adx_inv_augmented.setIdentity();

    for(int k=0; k<3; k++){
        Adx_inv_augmented.block<3,3>(6+3*k, 6+3*k) = RS_temp[i].Rotation.transpose();
    }

    Adx_inv_augmented.block<3,3>(9,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Velocity );
    Adx_inv_augmented.block<3,3>(12,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Position );




    Eigen::MatrixXd sqrt_info;
    sqrt_info.setZero(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    sqrt_info = fac_info[i].prop_primitive_sqrt_info;


    sqrt_info = sqrt_info * Adx_inv_augmented/dt;

    //residual---------------------------------------------------------------------------------------

    Eigen::VectorXd residual;
    residual.setZero(fac_info[i].prop_res_size, 1);

    Eigen::Matrix<double,5,5> f_X_i;
    f_X_i.setIdentity();

    Eigen::Vector3d IMU_Gyro_i, IMU_Acc_i;
    IMU_Gyro_i = fac_info[i].Z.block(0,0,3,1);
    IMU_Acc_i = fac_info[i].Z.block(3,0,3,1);

    int count_i =0;
    int count=0;

    if(i==0 && marginalization_flag){

        Eigen::Matrix3d R0;
        Eigen::Vector3d v0, p0, bg0, ba0;
        Eigen::VectorXd d0;

        R0 = X_0_bar.block(0,0,3,3);
        v0 = X_0_bar.block(0,3,3,1);
        p0 = X_0_bar.block(0,4,3,1);
        bg0 = Bias_0_bar.block(0,0,3,1);
        ba0 = Bias_0_bar.block(3,0,3,1);
        d0.resize(3*RS_temp[0].contact_leg_num, 1); //i basis
        d0.setZero();
        d0 = d_0_bar;

        f_X_i.block<3,3>(0,0) = R0 * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - bg0)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = v0
                                + R0 //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                                  * (IMU_Acc_i - ba0)* dt
                                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  p0 + v0 * dt
                                 + 0.5*R0 * (IMU_Acc_i - ba0)* dt*dt + 0.5*gravity*dt*dt;


        Eigen::Matrix<double,5,5> X_j_bar;
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;

        Eigen::Matrix<double,9,1> Delta;
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2);



        residual.block(0,0, 3,1) = bg0 - RS_temp[j].Bias_Gyro;
        residual.block(3,0, 3,1) = ba0 - RS_temp[j].Bias_Acc;
        residual.block(6,0, 9,1) = Delta;

        count=0;
        count_i=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k)){
                if ( RS_temp[j].Hard_Contact(k) ) {

                    residual.block<3, 1>(15 + 3 * count, 0) = d0.block<3, 1>(3 * count_i, 0) - RS_temp[j].d.block<3,1>(3 * k, 0);
                    count++;
                }
                count_i++;
            }
        }


    }else{


        f_X_i.block<3,3>(0,0) = RS_temp[i].Rotation * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - RS_temp[i].Bias_Gyro)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = RS_temp[i].Velocity
                                + RS_temp[i].Rotation //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                                  * (fac_info[i].Z.block(3,0,3,1) - RS_temp[i].Bias_Acc)* dt
                                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  RS_temp[i].Position + RS_temp[i].Velocity * dt
                                 + 0.5*RS_temp[i].Rotation * (fac_info[i].Z.block(3,0,3,1) - RS_temp[i].Bias_Acc)* dt*dt + 0.5*gravity*dt*dt;



        Eigen::Matrix<double,5,5> X_j_bar;
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;


        Eigen::Matrix<double,9,1> Delta;
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2);



        residual.block(0,0,3,1 ) = RS_temp[i].Bias_Gyro - RS_temp[j].Bias_Gyro;
        residual.block(3,0,3,1 ) = RS_temp[i].Bias_Acc - RS_temp[j].Bias_Acc;
        residual.block(6,0,9,1 ) = Delta;

        count=0;
        count_i=0;
        for (int k=0; k<robot.leg_no; k++)
        {
            if ( RS_temp[i].Hard_Contact(k)){
                if ( RS_temp[j].Hard_Contact(k) ) {

                    residual.block<3,1>(15 + 3 * count, 0) = RS_temp[i].d.block<3,1>(3*k,0) - RS_temp[j].d.block<3,1>(3 * k, 0);
                    count++;
                }
            }
        }


    }

    double testttttt=0;
    residual = sqrt_info * residual;
    cost += residual.transpose()*residual;


    //cout<<"now prop residual is "<<residual.transpose()*residual<<endl;



    //Jacobian-------------------------------------------------------------------------

    Eigen::MatrixXd I;
    I.setIdentity(fac_info[i].prop_res_size, fac_info[i].prop_res_size);

    Eigen::MatrixXd IAdt;
    IAdt.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
    IAdt = (I + A*dt);

    Eigen::MatrixXd partial_i;
    partial_i.resize(fac_info[i].prop_res_size, fac_info[i].prop_para0_size + 6);
    partial_i.block(0,0, fac_info[i].prop_res_size,6) = ( IAdt  ).block(0,0, fac_info[i].prop_res_size,6);
    partial_i.block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_para0_size) = ( IAdt ).block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_res_size-6) * fac_info[i].Mi;
    partial_i = sqrt_info * partial_i;

    Eigen::MatrixXd partial_j;
    partial_j.resize(fac_info[i].prop_res_size, fac_info[i].prop_para1_size + 6);
    partial_j.block(0,0, fac_info[i].prop_res_size,6)=( -I ).block(0,0, fac_info[i].prop_res_size,6);
    partial_j.block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_para1_size)=( -I ).block(0,6, fac_info[i].prop_res_size,fac_info[i].prop_res_size-6) * fac_info[i].Mj;
    partial_j = sqrt_info * partial_j;


    //Hessian-------------------------------------------------------------------------------------------

    Hessian.block(RS_temp[i].para_idx,RS_temp[i].para_idx, RS_temp[i].para_size,RS_temp[i].para_size) += partial_i.transpose() * partial_i;
    Hessian.block(RS_temp[j].para_idx,RS_temp[j].para_idx, RS_temp[j].para_size,RS_temp[j].para_size) += partial_j.transpose() * partial_j;
    Hessian.block(RS_temp[i].para_idx,RS_temp[j].para_idx, RS_temp[i].para_size,RS_temp[j].para_size) += partial_i.transpose() * partial_j;
    Hessian.block(RS_temp[j].para_idx,RS_temp[i].para_idx, RS_temp[j].para_size,RS_temp[i].para_size) += partial_j.transpose() * partial_i;

    //gradient-------------------------------------------------------------------------------------------
    gradient.block(RS_temp[i].para_idx,0, RS_temp[i].para_size,1) += partial_i.transpose() * residual;
    gradient.block(RS_temp[j].para_idx,0, RS_temp[j].para_size,1) += partial_j.transpose() * residual;

    double new_rows_no;

    new_rows_no = partial_i.cols()*partial_i.rows();
    VectorXd temp(Eigen::Map<VectorXd>(partial_i.data(),new_rows_no));
    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

    new_rows_no = partial_j.cols()*partial_j.rows();
    VectorXd temp2(Eigen::Map<VectorXd>(partial_j.data(),new_rows_no));
    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
    big_jacobian_by_vector.bottomRows(new_rows_no) = temp2;

}







//***************************Faster Meas factor********************************

void Adv_Inv_Factors::Invariant_Measurement_Factor(int j, int leg_num,
                                               Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    kinematics_info kin_k = *fac_info[j].leg_info.find(kinematics_info(leg_num));

    // Calculating Covariance--------------------------------------------------------------

    Eigen::Matrix3d sqrt_info;
    sqrt_info = kin_k.meas_primitive_sqrt_info * RS_temp[j].Rotation.transpose();

    //residual------------------------------------------------------------------------------
    Eigen::Vector3d residual;
    Eigen::Vector3d d_from_p;

    if(j==0 && marginalization_flag){
        d_from_p = X_0_bar.block(0,0,3,3) * kin_k.fk_kin + X_0_bar.block(0,4,3,1);
        residual = d_from_p - d_0_bar.block(3*kin_k.leg_num_in_state,0,3,1);
    }else{
        d_from_p = RS_temp[j].Rotation * kin_k.fk_kin + RS_temp[j].Position;
        residual = d_from_p - RS_temp[j].d.block(3*leg_num,0,3,1) ;// = R*fk+p -d = d_from_p - d
    }

    d_from_p = RS_temp[j].Rotation * kin_k.fk_kin + RS_temp[j].Position;

    residual = sqrt_info * residual;

    cost += residual.transpose()*residual;

    //cout<<"now meas residual is "<<residual.transpose()*residual<<endl;

    //Jacobians--------------------------------------------------------------------------------------

    Eigen::MatrixXd partial_xi_i;
    partial_xi_i.resize(3, fac_info[j].meas_para_size);
    partial_xi_i.setZero();

    partial_xi_i.block(0,0, 3,3) = -sqrt_info*BasicFunctions_Estimator::Hat_so3(d_from_p);
    partial_xi_i.block(0,6, 3,3) = sqrt_info;
    partial_xi_i.block(0, 9 + 3*(kin_k.leg_num_in_state), 3,3) = -sqrt_info;



    //Hessian and Gradient
    gradient.block(RS_temp[j].para_idx+6,0, RS_temp[j].para_size-6,1) += partial_xi_i.transpose() * residual;
    Hessian.block(RS_temp[j].para_idx+6, RS_temp[j].para_idx+6, RS_temp[j].para_size-6, RS_temp[j].para_size-6) += partial_xi_i.transpose() * partial_xi_i;


    double new_rows_no;

    new_rows_no = partial_xi_i.cols()*partial_xi_i.rows();
    VectorXd temp(Eigen::Map<VectorXd>(partial_xi_i.data(),new_rows_no));
    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;


}








void Adv_Inv_Factors::Invariant_Prior_RVP_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                             Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    Eigen::Matrix<double,5,5> X_hat_current_est;
    X_hat_current_est.setIdentity();

    X_hat_current_est.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_hat_current_est.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_hat_current_est.block<3,1>(0,4) = RS_temp[0].Position;

    //----------------------------------------------------------------

    Eigen::Matrix<double,9,1> X_pri_X_hat_inv_retracted;
    X_pri_X_hat_inv_retracted.setZero();
    X_pri_X_hat_inv_retracted = BasicFunctions_Estimator::Logm_seK_Vec(X_hat_current_est*X_Prior.inverse(), 2);

    Eigen::Matrix<double,9,9> jacobian_left_inv;
    jacobian_left_inv.setZero();

    jacobian_left_inv = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(X_pri_X_hat_inv_retracted, 2);

    //residual---------------------------------------------------------------------------------------
    Eigen::Matrix<double,9,1> residual;
    residual.setZero();
    residual = SQRT_INFO_Prior * (X_pri_X_hat_inv_retracted);



    cost += residual.transpose()*residual;
    //std::cout<<"now prior residual is "<<residual.norm()<<std::endl;

    //Jacobians---------------------------------------------------------------------------------
    Eigen::Matrix<double,9,9> partial_xi_0;
    partial_xi_0.setZero();
    partial_xi_0 = SQRT_INFO_Prior;// * jacobian_left_inv;



    //Hessian, gradient---------------------------------------------------------------------------
    Hessian.block(6,6, 9, 9) += partial_xi_0.transpose() * partial_xi_0;
    gradient.block(6,0,9,1) += partial_xi_0.transpose() * residual;


    double new_rows_no;

    new_rows_no = partial_xi_0.cols()*partial_xi_0.rows();
    VectorXd temp(Eigen::Map<VectorXd>(partial_xi_0.data(),new_rows_no));
    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;


}









void Adv_Inv_Factors::Invariant_Prior_Bias_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    //residual---------------------------------------------------------------------------
    Eigen::Matrix<double,6,1> residual;
    residual.setZero();

    residual << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;
    residual = residual - Bias_Prior;

    residual = SQRT_INFO_Prior_BIAS * ( residual);

    cost += residual.transpose()*residual;
    //cout<<"now prior bias residual is "<<esidual.norm()<<endl;

    Eigen::Matrix<double,6,6> partial_bias_0;
    partial_bias_0.setIdentity();
    partial_bias_0 = SQRT_INFO_Prior_BIAS * partial_bias_0;

    //Hessian, gradient-----------------------------------------------------------------------------
    Hessian.block(0,0,6,6) += partial_bias_0.transpose() * partial_bias_0;
    gradient.block(0,0, 6,1) += partial_bias_0.transpose() * residual;


    double new_rows_no;

    new_rows_no = partial_bias_0.cols()*partial_bias_0.rows();
    VectorXd temp(Eigen::Map<VectorXd>(partial_bias_0.data(),new_rows_no));
    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;


}

















































void Pseudo_Inv_Factors::Batch_Initialize(ROBOT_STATES_PSEUDO* _RS, const ROBOT_STATES_PSEUDO &RS_Prior,
                                     const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                     factor_info_pseudo* _fac_info,
                                     const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    fac_info = _fac_info;

    RS_temp = _RS;
//    RS_temp = new ROBOT_STATES_PSEUDO[frame_count+1];
//    for(int i=0; i<=frame_count; i++){
//        RS_temp[i]  =_RS[i];
//    }


    //prior_RVP--------------------------------------------------------------------------------------
    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_Prior.Rotation;
    X_Prior.block<3,1>(0,3) = RS_Prior.Velocity;
    X_Prior.block<3,1>(0,4) = RS_Prior.Position;

    SQRT_INFO_Prior.setZero();
    SQRT_INFO_Prior.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Orientation;
    SQRT_INFO_Prior.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Velocity;
    SQRT_INFO_Prior.block<3,3>(6,6) = robot.SQRT_INFO_Covariance_Prior_Position;

    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_Prior.Bias_Gyro, RS_Prior.Bias_Acc;

    SQRT_INFO_Prior_BIAS.setZero();
    SQRT_INFO_Prior_BIAS.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Prior_Bias_Gyro;
    SQRT_INFO_Prior_BIAS.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Prior_Bias_Acc;

}







void Pseudo_Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS,
                  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                  Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                  double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost !=0) {



            for (int p = 0; p < frame_count; p++) {

                fac_info[p].contact_cov_array = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
            }
        }

    }



    cost = 0;

    if( is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Invariant_Prior_RVP_Factor(Hessian, gradient, cost);

        Invariant_Prior_Bias_Factor(Hessian, gradient, cost);

        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);
            int j=i+1;

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }

        }

    }else{

        Hessian = -Hessian;
        gradient = -gradient;



        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[1].Hard_Contact(k) && RS_temp[2].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

                }
            }

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }





}








void Pseudo_Inv_Factors::Marg_Initialize(ROBOT_STATES_PSEUDO* _RS,
                                     const Eigen::Matrix<double,Eigen::Dynamic,1> &_Estimation_Z,
                                   const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _H,
                                   const Eigen::Matrix<double,Eigen::Dynamic,1> _b,
                                  factor_info_pseudo* _fac_info,
                                     const Robot_Model_mini_cheetah _robot){


    gravity << 0, 0, -9.80665;
    frame_count = _Estimation_Z.rows()/num_z -1;
    robot = _robot;
    dt = robot.dt;

    marginalization_flag = true;

    fac_info = _fac_info;

    RS_temp = _RS;



    //prior_RVP--------------------------------------------------------------------------------------

    X_Prior.setIdentity();

    X_Prior.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_Prior.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_Prior.block<3,1>(0,4) = RS_temp[0].Position;


    //prior_bias--------------------------------------------------------------------------------------
    Bias_Prior << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;


    //Marginalization factor--------------------------------------------------------------------------

    Hessian_Marg.resizeLike(_H);
    Hessian_Marg = _H.transpose()*_H;
    H =_H;

    gradient_Marg.resizeLike(_b);
    gradient_Marg = _H.transpose()*_b;
    b = _b;

}







void Pseudo_Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS, Eigen::Matrix<double,15*(WINDOW_SIZE+1),1> Zeta_Xi,
                  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                  Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                  double &cost){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost !=0){




                for(int p=0; p<frame_count; p++){

                    fac_info[p].contact_cov_array = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
                }

        }
    }




    cost = 0;

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, 9,1> Xi_0;
        Xi_0 = Zeta_Xi.block(6,0,9,1);

        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2)*X_0_bar;


        Eigen::Matrix<double,15,1> Bias_X_bar_X_vee_inv_retracted;
        Bias_X_bar_X_vee_inv_retracted.setZero();
        Bias_X_bar_X_vee_inv_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted.block(6,0,9,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2);

        Hessian.block(0,0, 15, 15) = Hessian_Marg;
        gradient.block(0,0, 15, 1) = Hessian_Marg * (Bias_X_bar_X_vee_inv_retracted) + gradient_Marg ;

        double temp_cost =( H * Bias_X_bar_X_vee_inv_retracted + b ).norm();

        cost += temp_cost*temp_cost;

        //cout<<"Now marg res is "<< cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost);

            int j=i+1;

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

                }
            }
        }


    }else{

        Hessian = -Hessian;
        gradient = -gradient;


        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, cost);

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[1].Hard_Contact(k) && RS_temp[2].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

                }
            }

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }



}





void Pseudo_Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES_PSEUDO* _RS, Eigen::Matrix<double,15*(WINDOW_SIZE+1),1> Zeta_Xi,
                                                              Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                              Eigen::Matrix<double,Eigen::Dynamic,1> &gradient,
                                                              Eigen::Matrix<double,4,1> &cost_log){



    if(is_for_marginalization == false){

        RS_temp = _RS;

        if(cost_log(0) !=0){




            for(int p=0; p<frame_count; p++){

                fac_info[p].contact_cov_array = robot.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
            }

        }
    }




    cost_log.setZero();

    if(is_for_marginalization == false){

        Hessian.setZero();
        gradient.setZero();

        Bias_0_bar.block(0,0,3,1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0,0,3,1);
        Bias_0_bar.block(3,0,3,1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3,0,3,1);

        Eigen::Matrix<double, 9,1> Xi_0;
        Xi_0 = Zeta_Xi.block(6,0,9,1);

        X_0_bar.setIdentity();
        X_0_bar.block<3,3>(0,0) = RS_temp[0].Rotation;
        X_0_bar.block<3,1>(0,3) = RS_temp[0].Velocity;
        X_0_bar.block<3,1>(0,4) = RS_temp[0].Position;
        X_0_bar = BasicFunctions_Estimator::Expm_seK_Vec(Xi_0, 2)*X_0_bar;


        Eigen::Matrix<double,15,1> Bias_X_bar_X_vee_inv_retracted;
        Bias_X_bar_X_vee_inv_retracted.setZero();
        Bias_X_bar_X_vee_inv_retracted.block(0,0,6,1) = Bias_0_bar - Bias_Prior;
        Bias_X_bar_X_vee_inv_retracted.block(6,0,9,1) = BasicFunctions_Estimator::Logm_seK_Vec(X_0_bar*X_Prior.inverse(), 2);

        Hessian.block(0,0, 15, 15) = Hessian_Marg;
        gradient.block(0,0, 15, 1) = Hessian_Marg * (Bias_X_bar_X_vee_inv_retracted) + gradient_Marg ;

        double temp_cost =( H * Bias_X_bar_X_vee_inv_retracted + b ).norm();

        cost_log(1) = temp_cost*temp_cost;

        //cout<<"Now marg res is "<< cost<<endl;


        for (int i=0; i<frame_count; i++){
            Invariant_Propagation_Factor(i, Hessian, gradient, cost_log(2));

            int j=i+1;

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(i, k, Hessian, gradient, cost_log(3));

                }
            }
        }


    }else{

        Hessian = -Hessian;
        gradient = -gradient;

        double garbage_cost=0;


        if(frame_count > 1){

            Invariant_Propagation_Factor(1, Hessian, gradient, garbage_cost);

            for (int k=0; k<robot.leg_no; k++)
            {
                if (RS_temp[1].Hard_Contact(k) && RS_temp[2].Hard_Contact(k) ){

                    Invariant_Measurement_Factor(1, k, Hessian, gradient, garbage_cost);

                }
            }

        }

        Hessian = -Hessian;
        gradient = -gradient;


    }


    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);

}









void Pseudo_Inv_Factors::Invariant_Propagation_Factor(int i,
        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
        Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    int j=i+1;

    Eigen::Matrix<double, 15,15> A;
    A.setZero();


    A.block(9,6, 3,3) = BasicFunctions_Estimator::Hat_so3(gravity);
    A.block(12,6, 3,3) = 0.5 * BasicFunctions_Estimator::Hat_so3(gravity) * dt;
    A.block(12,9, 3,3) = Matrix3d::Identity();


    A.block(6, 0, 3,3) = - RS_temp[i].Rotation;

    A.block(9, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Velocity ) * RS_temp[i].Rotation;
    A.block(9, 3, 3,3) = - RS_temp[i].Rotation;

    A.block(12, 0, 3,3) = - BasicFunctions_Estimator::Hat_so3( RS_temp[i].Position ) * RS_temp[i].Rotation;


    //Covariance Calculation-------------------------------------------------------------------------------
    //Adjoint of X_bar---------------------------------------------------------------------

    Eigen::Matrix<double, 15,15> Adx_inv_augmented;
    Adx_inv_augmented.setZero();


    Adx_inv_augmented.block(0,0, 6,6).setIdentity();

    for(int k=0; k<3; k++){
        Adx_inv_augmented.block<3,3>(6+3*k, 6+3*k) = RS_temp[i].Rotation.transpose();
    }

    Adx_inv_augmented.block<3,3>(9,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Velocity );
    Adx_inv_augmented.block<3,3>(12,6) = - RS_temp[i].Rotation.transpose() * BasicFunctions_Estimator::Hat_so3(RS_temp[i].Position );


    Eigen::Matrix<double, 15,15> sqrt_info;
    sqrt_info = fac_info[i].prop_primitive_sqrt_info* Adx_inv_augmented/dt;
//std::cout << "Test prop sqrt_info: "<< (sqrt_info.transpose()*sqrt_info*cov - Eigen::MatrixXd::Identity(res_size, res_size)).norm() << std::endl;



    //residual---------------------------------------------------------------------------------------

    Eigen::Matrix<double, 15, 1> residual;

    Eigen::Matrix<double, 5,5> f_X_i;
    f_X_i.setIdentity();

    Eigen::Vector3d IMU_Gyro_i, IMU_Acc_i;
    IMU_Gyro_i = fac_info[i].Z.block(0,0,3,1);
    IMU_Acc_i = fac_info[i].Z.block(3,0,3,1);




    if(i==0 && marginalization_flag){

        Eigen::Matrix3d R0;
        Eigen::Vector3d v0, p0, bg0, ba0;
        R0 = X_0_bar.block(0,0,3,3);
        v0 = X_0_bar.block(0,3,3,1);
        p0 = X_0_bar.block(0,4,3,1);
        bg0 = Bias_0_bar.block(0,0,3,1);
        ba0 = Bias_0_bar.block(3,0,3,1);

        f_X_i.block<3,3>(0,0) = R0 * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - bg0)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = v0
                + R0 //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                * (IMU_Acc_i - ba0)* dt
                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  p0 + v0 * dt
                + 0.5*R0 * (IMU_Acc_i - ba0)* dt*dt + 0.5*gravity*dt*dt;



        Eigen::Matrix<double, 5,5> X_j_bar;
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;


        Eigen::Matrix<double,9,1> Delta;
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2);


        residual.block(0,0,3,1 ) = bg0 - RS_temp[j].Bias_Gyro;
        residual.block(3,0,3,1 ) = ba0 - RS_temp[j].Bias_Acc;
        residual.block(6,0,9,1 ) = Delta;

    }else{

        f_X_i.block<3,3>(0,0) = RS_temp[i].Rotation * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_i - RS_temp[i].Bias_Gyro)*dt);
        JacobiSVD<Matrix3d> svd(f_X_i.block<3,3>(0,0), ComputeFullU|ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();

        f_X_i.block<3,1>(0,3) = RS_temp[i].Velocity
                + RS_temp[i].Rotation //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                * (IMU_Acc_i - RS_temp[i].Bias_Acc)* dt
                + gravity * dt;

        f_X_i.block<3,1>(0,4) =  RS_temp[i].Position + RS_temp[i].Velocity * dt
                + 0.5*RS_temp[i].Rotation * (IMU_Acc_i - RS_temp[i].Bias_Acc)* dt*dt + 0.5*gravity*dt*dt;



        Eigen::Matrix<double, 5,5> X_j_bar;
        X_j_bar.setIdentity();

        X_j_bar.block<3,3>(0,0) = RS_temp[j].Rotation;
        X_j_bar.block<3,1>(0,3) = RS_temp[j].Velocity;
        X_j_bar.block<3,1>(0,4) = RS_temp[j].Position;


        Eigen::Matrix<double,9,1> Delta;
        Delta.setZero();
        Delta = BasicFunctions_Estimator::Logm_seK_Vec(f_X_i*X_j_bar.inverse(), 2);

        residual.block(0,0,3,1 ) = RS_temp[i].Bias_Gyro - RS_temp[j].Bias_Gyro;
        residual.block(3,0,3,1 ) = RS_temp[i].Bias_Acc - RS_temp[j].Bias_Acc;
        residual.block(6,0,9,1 ) = Delta;


    }






    residual = sqrt_info * residual;
    cost += residual.transpose()*residual;


    //cout<<"now prop residual is "<<residual.norm()<<endl;



    //Jacobian-------------------------------------------------------------------------

    Eigen::Matrix<double,15,15> I;
    I.setIdentity(15,15);

    Eigen::Matrix<double,15,15> partial_i;
    partial_i = sqrt_info * (I + A*dt);

    Eigen::Matrix<double,15,15> partial_j;
    partial_j = sqrt_info * (-I);


    //Hessian-------------------------------------------------------------------------------------------

    Hessian.block(15*i,15*i, 15,15) += partial_i.transpose() * partial_i;
    Hessian.block(15*j,15*j, 15,15) += partial_j.transpose() * partial_j;
    Hessian.block(15*i,15*j, 15,15) += partial_i.transpose() * partial_j;
    Hessian.block(15*j,15*i, 15,15) += partial_j.transpose() * partial_i;

    //gradient-------------------------------------------------------------------------------------------
    gradient.block(15*i,0, 15,1) += partial_i.transpose() * residual;
    gradient.block(15*j,0, 15,1) += partial_j.transpose() * residual;


}






//***************************Faster Meas factor********************************

void Pseudo_Inv_Factors::Invariant_Measurement_Factor(int i, int leg_num,
                                                    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                    Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{

    int j=i+1;

    kinematics_info kin_i_k = *fac_info[i].leg_info.find(kinematics_info(leg_num));
    kinematics_info kin_j_k = *fac_info[j].leg_info.find(kinematics_info(leg_num));



    // Calculating Covariance--------------------------------------------------------------
    Eigen::Matrix3d cov;
    cov.setZero();
    cov = RS_temp[j].Rotation * kin_j_k.meas_primitive_sqrt_info * RS_temp[j].Rotation.transpose()
            + RS_temp[i].Rotation * kin_i_k.meas_primitive_sqrt_info * RS_temp[i].Rotation.transpose()
            + dt*RS_temp[i].Rotation*(fac_info[i].contact_cov_array[leg_num])*RS_temp[i].Rotation.transpose()*dt;

    //cov = dt*RS_temp[i].Rotation*(fac_info[i].contact_cov_array[leg_num])*RS_temp[i].Rotation.transpose()*dt;
    //Here, meas_primitive_sqrt_info doesn't follow its nominal name: instead, it is J*Cov*J'

    Eigen::Matrix3d sqrt_info;
    sqrt_info.setZero();

    double eps = 1e-30;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(cov);
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    sqrt_info = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose();

    //residual------------------------------------------------------------------------------
    Eigen::Vector3d d_j, d_i, residual;

    if(i==0 && marginalization_flag){

        d_i = X_0_bar.block(0,0,3,3) * kin_i_k.fk_kin + X_0_bar.block(0,4,3,1);

    }else{

        d_i = RS_temp[i].Rotation * kin_i_k.fk_kin + RS_temp[i].Position;

    }

    d_j = RS_temp[j].Rotation * kin_j_k.fk_kin + RS_temp[j].Position;

    residual = sqrt_info * (d_j - d_i);

    cost += residual.transpose()*residual;
    //cout<<"now meas residual is "<<esidual.norm()<<endl;

    //Jacobians--------------------------------------------------------------------------------------

    Eigen::Matrix<double, 3,9> partial_xi_i;
    partial_xi_i.setZero();
    Eigen::Vector3d d_i_jac;
    d_i_jac = RS_temp[i].Rotation * kin_i_k.fk_kin + RS_temp[i].Position;

    partial_xi_i.block(0,0, 3,3) = sqrt_info*BasicFunctions_Estimator::Hat_so3(d_i_jac);
    partial_xi_i.block(0, 6, 3,3) = -sqrt_info;


    Eigen::Matrix<double, 3,9> partial_xi_j;
    partial_xi_j.setZero();

    partial_xi_j.block(0,0, 3,3) = -sqrt_info*BasicFunctions_Estimator::Hat_so3(d_j);
    partial_xi_j.block(0, 6, 3,3) = sqrt_info;



    Hessian.block(15*i+6, 15*i+6, 9, 9) += partial_xi_i.transpose() * partial_xi_i;
    Hessian.block(15*i+6, 15*j+6, 9, 9) += partial_xi_i.transpose() * partial_xi_j;
    Hessian.block(15*j+6, 15*i+6, 9, 9) += partial_xi_j.transpose() * partial_xi_i;
    Hessian.block(15*j+6, 15*j+6, 9, 9) += partial_xi_j.transpose() * partial_xi_j;

    gradient.block(15*i+6,0, 9,1) += partial_xi_i.transpose() * residual;
    gradient.block(15*j+6,0, 9,1) += partial_xi_j.transpose() * residual;



}






void Pseudo_Inv_Factors::Invariant_Prior_RVP_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                   Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    Eigen::Matrix<double,5,5> X_hat_current_est;
    X_hat_current_est.setIdentity();

    X_hat_current_est.block<3,3>(0,0) = RS_temp[0].Rotation;
    X_hat_current_est.block<3,1>(0,3) = RS_temp[0].Velocity;
    X_hat_current_est.block<3,1>(0,4) = RS_temp[0].Position;

    //----------------------------------------------------------------

    Eigen::Matrix<double,9,1> X_pri_X_hat_inv_retracted;
    X_pri_X_hat_inv_retracted.setZero();
    X_pri_X_hat_inv_retracted = BasicFunctions_Estimator::Logm_seK_Vec(X_hat_current_est*X_Prior.inverse(), 2);

    Eigen::Matrix<double,9,9> jacobian_left_inv;
    jacobian_left_inv.setZero();

    jacobian_left_inv = BasicFunctions_Estimator::Inv_Left_Jacobian_SEk(X_pri_X_hat_inv_retracted, 2);

    //residual---------------------------------------------------------------------------------------
    Eigen::Matrix<double,9,1> residual;
    residual = SQRT_INFO_Prior * (X_pri_X_hat_inv_retracted);



    cost += residual.transpose()*residual;
    //std::cout<<"now prior residual is "<<residual.transpose()*residual<<std::endl;

    //Jacobians---------------------------------------------------------------------------------
    Eigen::Matrix<double,9,9> partial_xi_0;
    partial_xi_0 = SQRT_INFO_Prior;// * jacobian_left_inv;



    //Hessian, gradient---------------------------------------------------------------------------
    Hessian.block(6,6, 9, 9) += partial_xi_0.transpose() * partial_xi_0;
    gradient.block(6,0,9,1) += partial_xi_0.transpose() * residual;


}









void Pseudo_Inv_Factors::Invariant_Prior_Bias_Factor(Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &Hessian,
                                                    Eigen::Matrix<double,Eigen::Dynamic,1> &gradient, double &cost)
{


    //residual---------------------------------------------------------------------------
    Eigen::Matrix<double,6,1> residual;

    residual << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;
    residual = residual - Bias_Prior;

    residual = SQRT_INFO_Prior_BIAS * ( residual);

    cost += residual.transpose()*residual;
    //cout<<"now prior bias residual is "<<esidual.norm()<<endl;

    Eigen::Matrix<double,6,6> partial_bias_0;
    partial_bias_0.setIdentity();
    partial_bias_0 = SQRT_INFO_Prior_BIAS * partial_bias_0;

    //Hessian, gradient-----------------------------------------------------------------------------
    Hessian.block(0,0,6,6) += partial_bias_0.transpose() * partial_bias_0;
    gradient.block(0,0, 6,1) += partial_bias_0.transpose() * residual;


}





















//**************************Factors for Smoother******************************

void Factors::Propagation_Factor(
        Eigen::Matrix<double,9,1> &residual,
        Eigen::Matrix<double,9,15> &partial_i,
        Eigen::Matrix<double,9,15> &partial_j,
        const Eigen::Matrix<double,42,1> &X,
        const Eigen::Matrix<double,30,1> &dX,
        const Eigen::Matrix<double,6,1> &IMU,
        const Robot_Model_mini_cheetah &robot)
{

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.80665;

    double dt = robot.dt;


    //Assigning State variable----------------------------------------------

    Eigen::Matrix3d R_i_bar,R_j_bar;
    Eigen::Vector3d V_i_bar, V_j_bar, P_i_bar, P_j_bar;

    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R_i_bar(3*i+j) = X(6+ 3*i+j);
            R_j_bar(3*i+j) = X(21 +6 +3*i+j);
        }
    }

    V_i_bar = X.block(15,0,3,1);
    P_i_bar = X.block(18,0,3,1);

    V_j_bar = X.block(21 +15,0, 3,1);
    P_j_bar = X.block(21 +18,0, 3,1);

    Eigen::Vector3d bg_i_bar = X.block(0,0,3,1);
    Eigen::Vector3d ba_i_bar = X.block(3,0,3,1);
    Eigen::Vector3d IMU_Gyro = IMU.block(0,0,3,1);
    Eigen::Vector3d IMU_Acc = IMU.block(3,0,3,1);


    //assigning parameter variable---------------------

    Eigen::Matrix<double, 3, 1> del_bg = dX.block(0,0,3,1);
    Eigen::Matrix<double, 3, 1> del_ba = dX.block(3,0,3,1);

    Eigen::Matrix<double, 3, 1> del_phi_i = dX.block(6,0,3,1);
    Eigen::Matrix<double, 3, 1> del_v_i = dX.block(9,0,3,1);
    Eigen::Matrix<double, 3, 1> del_p_i = dX.block(12,0,3,1);

    Eigen::Matrix<double, 3, 1> del_phi_j = dX.block(21,0,3,1);
    Eigen::Matrix<double, 3, 1> del_v_j = dX.block(24,0,3,1);
    Eigen::Matrix<double, 3, 1> del_p_j = dX.block(27,0,3,1);

    Eigen::Matrix3d R_i, R_j;
    Eigen::Vector3d P_i, P_j, V_i, V_j, bg_i, ba_i;

    R_i = R_i_bar * BasicFunctions_Estimator::Expm_Vec(del_phi_i);
    R_j = R_j_bar * BasicFunctions_Estimator::Expm_Vec(del_phi_j);

    P_i = P_i_bar + del_p_i;
    P_j = P_j_bar + del_p_j;

    V_i = V_i_bar + del_v_i;
    V_j = V_j_bar + del_v_j;

    bg_i = bg_i_bar + del_bg;
    ba_i = ba_i_bar + del_ba;


    //Cov-----------------------------------------
    Eigen::Matrix<double, 9, 9> cov;
    cov.setZero();

    //--------------------------cov_ori--------------

    Eigen::Matrix3d r_del_R_for_jac;
    Eigen::Vector3d phi_diff_for_jac;
    r_del_R_for_jac = BasicFunctions_Estimator::Expm_Vec( (IMU_Gyro-bg_i_bar)*dt).transpose();
    phi_diff_for_jac = BasicFunctions_Estimator::Logm_Vec( (r_del_R_for_jac*R_i_bar.transpose()*R_j_bar) );
    Eigen::Matrix3d A = BasicFunctions_Estimator::Right_Jacobian_SO3(phi_diff_for_jac).inverse()*BasicFunctions_Estimator::Right_Jacobian_SO3((IMU_Gyro-bg_i_bar)*dt)*dt;

    cov.block(0,0,3,3) = A * robot.Covariance_Gyro * A.transpose();

    //--------------------------cov_vel--------------
    cov.block(3,3,3,3) = (dt)*robot.Covariance_Acc*(dt);

    //--------------------------cov_pos--------------
    cov.block(6,6,3,3) = (0.5*dt*dt)*robot.Covariance_Acc*(0.5*dt*dt);

    Eigen::Matrix<double, 9, 9> sqrt_info;
    sqrt_info.setZero();

    double eps = 1e-30;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(cov);
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    sqrt_info = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose();
//std::cout << "Test prop sqrt_info: "<< (sqrt_info.transpose()*sqrt_info*cov - Eigen::MatrixXd::Identity(9,9)).norm() << std::endl;



    //-------------------Big jacobian block--------------------------------
    Eigen::Matrix<double, 9,30> big_jacobian;
    big_jacobian.setZero();

    big_jacobian.block(0,0,3,3) = dt*BasicFunctions_Estimator::Right_Jacobian_SO3( BasicFunctions_Estimator::Logm_Vec(R_i_bar.transpose()*R_j_bar) ).inverse()* R_j_bar.transpose() * R_i_bar;
    big_jacobian.block(0,6,3,3) = -BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(r_del_R_for_jac*R_i_bar.transpose()*R_j_bar)).inverse()*R_j_bar.transpose()*R_i_bar;
    big_jacobian.block(0,21,3,3) = BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(r_del_R_for_jac*R_i_bar.transpose()*R_j_bar)).inverse();

    big_jacobian.block(3,3,3,3) = dt * Eigen::Matrix3d::Identity();
    big_jacobian.block(3,6,3,3) = BasicFunctions_Estimator::Hat_so3( R_i_bar.transpose()*(V_j_bar-V_i_bar-gravity*dt) );
    big_jacobian.block(3,9,3,3) = -Eigen::Matrix3d::Identity()*R_i_bar.transpose();
    big_jacobian.block(3,24,3,3) = R_i_bar.transpose();

    big_jacobian.block(6,3,3,3) = 0.5*dt*dt*Eigen::Matrix3d::Identity();
    big_jacobian.block(6,6,3,3) = BasicFunctions_Estimator::Hat_so3( R_i_bar.transpose()*(P_j_bar-P_i_bar-V_i_bar*dt-0.5*gravity*dt*dt) );
    big_jacobian.block(6,9,3,3) = -dt*Eigen::Matrix3d::Identity()*R_i_bar.transpose();
    big_jacobian.block(6,12,3,3) = -Eigen::Matrix3d::Identity()*R_i_bar.transpose();
    big_jacobian.block(6,27,3,3) = R_i_bar.transpose();

    //--------------------------residual------------------
    residual.setZero();

    Eigen::Matrix3d r_del_R;
    Eigen::Vector3d phi_diff;
    r_del_R = BasicFunctions_Estimator::Expm_Vec( (IMU_Gyro-bg_i)*dt).transpose();
    phi_diff = BasicFunctions_Estimator::Logm_Vec( (r_del_R*R_i.transpose()*R_j) );

    residual.block(0,0,3,1) = phi_diff;
    residual.block(3,0,3,1) = R_i.transpose() * (V_j-V_i-gravity*dt) - (IMU_Acc-ba_i)*dt;
    residual.block(6,0,3,1) = R_i.transpose() * (P_j - P_i - V_i*dt - 0.5*gravity*dt*dt) - 0.5*(IMU_Acc-ba_i)*dt*dt;

    residual = sqrt_info * residual;


    //std::cout<<"now prop residual is "<<residual.transpose()*residual<<std::endl;

    //Jacobians-------------------------------------------------------------------------------------------


        if (partial_i.data()) {
            //del_phi_i
            partial_i.setZero();
            partial_i = big_jacobian.leftCols(15);


            partial_i = sqrt_info * partial_i;

        }

        if (partial_j.data()) {
            //del_phi_i
            partial_j.setZero();
            partial_j = big_jacobian.rightCols(15);

            partial_j = sqrt_info * partial_j;

        }



}
















void Factors::Measurement_Factor(
        Eigen::Matrix<double,3,1> &residual,
        Eigen::Matrix<double,3,15> &partial_i,
        Eigen::Matrix<double,3,15> &partial_j,
        const Eigen::Matrix<double,42,1> &X,
        const Eigen::Matrix<double,30,1> &dX,
        Eigen::Vector3d ENC_i,
        Eigen::Vector3d ENC_j,
        int leg_num,
        double contact_cov,
        const Robot_Model_mini_cheetah &robot)
{

    //Both X and dX contain the info of frame i, except bias info
    //ENC_i transfers only 3*1



    //Assigning State variable----------------------------------------------

    Eigen::Matrix3d R_i_bar, R_j_bar;
    Eigen::Vector3d P_i_bar, P_j_bar;

    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R_i_bar(3*i+j) = X(6+ 3*i+j);
            R_j_bar(3*i+j) = X(21 +6 +3*i+j);
        }
    }

    P_i_bar = X.block(18,0,3,1);
    P_j_bar = X.block(21 +18,0, 3,1);


    //Assigning Parameter variable----------------------------------------------
    Eigen::Matrix<double, 3, 1> del_phi_i = dX.block(6,0,3,1);
    Eigen::Matrix<double, 3, 1> del_p_i = dX.block(12,0,3,1);

    Eigen::Matrix<double, 3, 1> del_phi_j = dX.block(21,0,3,1);
    Eigen::Matrix<double, 3, 1> del_p_j = dX.block(27,0,3,1);

    Eigen::Matrix3d R_i, R_j;
    Eigen::Vector3d P_i, P_j;

    R_i = R_i_bar * BasicFunctions_Estimator::Expm_Vec(del_phi_i);
    R_j = R_j_bar * BasicFunctions_Estimator::Expm_Vec(del_phi_j);
    P_i = P_i_bar + del_p_i;
    P_j = P_j_bar + del_p_j;


    // Calculating Covariance-----------------------------------------------------------------------------
    Eigen::Matrix<double, 3, 3> cov;
    cov.setZero();
    cov = (R_j_bar * robot.Jacobian_Leg(ENC_j, leg_num)) * robot.cov_enc_const * (R_j_bar * robot.Jacobian_Leg(ENC_j, leg_num)).transpose()
            + (R_i_bar * robot.Jacobian_Leg(ENC_i, leg_num)) * robot.cov_enc_const * (R_i_bar * robot.Jacobian_Leg(ENC_i, leg_num)).transpose()
            + robot.dt*R_i_bar*contact_cov*R_i_bar.transpose()*robot.dt;

    //cov = robot.dt*R_i_bar*contact_cov*R_i_bar.transpose()*robot.dt;

    //cout<<"cov_kin"<<endl<<cov<<endl;
    //cov=robot.Covariance_Kinematics_vel*Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 3> sqrt_info;
    sqrt_info.setZero();

    double eps = 1e-30;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(cov);
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    sqrt_info = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose();


//std::cout << "Test meas sqrt_info: "<< (sqrt_info.transpose()*sqrt_info*cov - Eigen::MatrixXd::Identity(3,3)).norm() << std::endl;

    //-------------------Big jacobian block--------------------------------
    Eigen::Matrix<double, 3,30> big_jacobian;
    big_jacobian.setZero();
    big_jacobian.block(0,6,3,3) = R_i_bar * BasicFunctions_Estimator::Hat_so3( robot.Forward_Kinematics_Leg(ENC_i, leg_num) );

    big_jacobian.block(0,21,3,3) = - R_j_bar * BasicFunctions_Estimator::Hat_so3( robot.Forward_Kinematics_Leg(ENC_j, leg_num) );

    big_jacobian.block(0,12,3,3) = -Eigen::Matrix3d::Identity();

    big_jacobian.block(0,27,3,3) = Eigen::Matrix3d::Identity();

    //--------------------------residual------------------
    Eigen::Vector3d d_j = P_j + R_j* robot.Forward_Kinematics_Leg(ENC_j, leg_num);
    Eigen::Vector3d d_i = P_i + R_i* robot.Forward_Kinematics_Leg(ENC_i, leg_num);

    residual.setZero();
    residual = d_j-d_i;
//cout<<"res before"<<endl<<residual<<endl;
    //cout<<"P_i "<<P_i.transpose()<<" P_j "<<P_j.transpose()<<endl;
    //cout<<"fk i "<<(R_i*robot.Forward_Kinematics_Leg(ENC_i, leg_num)).transpose()<<" fk j "<<(R_j*robot.Forward_Kinematics_Leg(ENC_j, leg_num)).transpose()<<endl;
    //cout<<"dj -di "<<residual.transpose()<<endl;
    residual = sqrt_info * residual;


   //std::cout<<"now meas residual is "<<residual.transpose()*residual<<std::endl;

    //Jacobians-------------------------------------------------------------------------------------------


    if (partial_i.data()) {
        //del_phi_i
        partial_i.setZero();
        partial_i = big_jacobian.leftCols(15);

        partial_i = sqrt_info * partial_i;

        //std::cout<<"now meas residual is "<<residual.transpose()*residual<<std::endl;


        //std::cout<<"cov is "<<cov<<std::endl;

    }

    if (partial_j.data()) {
        //del_phi_i
        partial_j.setZero();
        partial_j = big_jacobian.rightCols(15);

        partial_j = sqrt_info * partial_j;



    }





}








//***************************version 1************************************

void Factors::Prior_Factor(
        Eigen::Matrix<double,15,1> &residual,
        Eigen::Matrix<double,15,15> &partial_0,
        const Eigen::Matrix<double,21,1> &X,
        const Eigen::Matrix<double,21,1> &X_init,
        const Eigen::Matrix<double,15,1> &dX,
        const Robot_Model_mini_cheetah &robot)
{


    //State Components
    Eigen::Matrix3d R_0_hat,R_0_bar;
    Eigen::Vector3d P_0_hat, P_0_bar, V_0_hat, V_0_bar, bg_0_bar, ba_0_bar, bg_0_hat, ba_0_hat;

    //Assigning State variable----------------------------------------------
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R_0_bar(3*i+j) = X_init(6+3*i+j);
            R_0_hat(3*i+j) = X(6+3*i+j);
        }
    }

    V_0_bar = X_init.block(15,0,3,1);
    P_0_bar = X_init.block(18,0,3,1);

    V_0_hat = X.block(15,0,3,1);
    P_0_hat = X.block(18,0,3,1);

    bg_0_bar = X_init.block(0,0,3,1);
    ba_0_bar = X_init.block(3,0,3,1);

    bg_0_hat = X.block(0,0,3,1);
    ba_0_hat = X.block(3,0,3,1);


    //Asigning parameter variable----------------------------------------------


    Eigen::Matrix<double, 3, 1> del_bg = dX.block(0,0,3,1);
    Eigen::Matrix<double, 3, 1> del_ba = dX.block(3,0,3,1);

    Eigen::Matrix<double, 3, 1> del_phi = dX.block(6,0,3,1);
    Eigen::Matrix<double, 3, 1> del_v = dX.block(9,0,3,1);
    Eigen::Matrix<double, 3, 1> del_p = dX.block(12,0,3,1);

    Eigen::Matrix3d R_0;
    Eigen::Vector3d P_0, V_0, bg_0, ba_0;

    R_0 = R_0_hat * BasicFunctions_Estimator::Expm_Vec(del_phi);

    P_0 = P_0_hat + del_p;

    V_0 = V_0_hat + del_v;

    bg_0 = bg_0_hat + del_bg;
    ba_0 = ba_0_hat + del_ba;


    // Calculating Covariance-----------------------------------------------------------------------------
    Eigen::Matrix<double, 15, 15> cov;
    cov.setZero();


    Eigen::Matrix3d A = BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(R_0_bar.transpose() * R_0)).inverse();
    cov.block(0,0,3,3) = robot.Covariance_Prior_Bias_Gyro;
    cov.block(3,3,3,3) = robot.Covariance_Prior_Bias_Acc;
    cov.block(6,6,3,3) = A * robot.Covariance_Prior_Orientation * A.transpose();
    cov.block(9,9,3,3) = robot.Covariance_Prior_Velocity;
    cov.block(12,12,3,3) =  robot.Covariance_Prior_Position;

//cout<<"cov_pri"<<endl<<cov<<endl;
    Eigen::Matrix<double, 15, 15> sqrt_info;
    sqrt_info.setZero();

    double eps = 1e-30;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(cov);
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    sqrt_info = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose();
    //std::cout << "Test pri sqrt_info: "<< (sqrt_info.transpose()*sqrt_info*cov - Eigen::MatrixXd::Identity(15,15)).norm() << std::endl;


    //big jacobian----------------------------------------------------------
    Eigen::Matrix<double, 15,15> big_jacobian;
    big_jacobian.setIdentity();

    big_jacobian.block(6,6,3,3) = BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(R_0_bar.transpose() * R_0_hat)).inverse();


    //residual-----------------------------------------------------------------------------

    residual.setZero();

    residual.block(0,0, 3,1) = bg_0 - bg_0_bar;
    residual.block(3,0, 3,1) = ba_0 - ba_0_bar;
    residual.block(6,0, 3,1) = BasicFunctions_Estimator::Logm_Vec(R_0_bar.transpose() * R_0);
    residual.block(9,0, 3,1) = V_0 - V_0_bar;
    residual.block(12,0, 3,1) = P_0 - P_0_bar;
    residual = sqrt_info * residual;

    //std::cout<<"now pior residual is "<<residual.transpose()*residual<<std::endl;



    //Jacobians---------------------------------------------------------------------------------

    if (partial_0.data()){
        partial_0.setZero();
        partial_0 = sqrt_info * big_jacobian;

        //std::cout<<"now prior residual is "<<residual.transpose()*residual<<std::endl;
//
    }



}














void Factors::Bias_Factor(
        Eigen::Matrix<double,6,1> &residual,
        Eigen::Matrix<double,6,15> &partial_i,
        Eigen::Matrix<double,6,15> &partial_j,
        const Eigen::Matrix<double,42,1> &X,
        const Eigen::Matrix<double,30,1> &dX,
        const Robot_Model_mini_cheetah &robot)
{




    //Assigning state-----------------------------------------------------------------------------
    Eigen::Vector3d bg_i_bar, ba_i_bar, bg_j_bar, ba_j_bar;
    bg_i_bar = X.block(0,0,3,1);
    ba_i_bar = X.block(3,0,3,1);

    bg_j_bar = X.block(21,0,3,1);
    ba_j_bar = X.block(24,0,3,1);



    Eigen::Matrix<double, 3, 1> del_bg_i = dX.block(0,0,3,1);
    Eigen::Matrix<double, 3, 1> del_ba_i = dX.block(3,0,3,1);

    Eigen::Matrix<double, 3, 1> del_bg_j = dX.block(15,0,3,1);
    Eigen::Matrix<double, 3, 1> del_ba_j = dX.block(18,0,3,1);

    Eigen::Vector3d bg_i, ba_i, bg_j, ba_j;

    bg_i = bg_i_bar + del_bg_i;
    ba_i = ba_i_bar + del_ba_i;
    bg_j = bg_j_bar + del_bg_j;
    ba_j = ba_j_bar + del_ba_j;

    // Calculating Covariance-----------------------------------------------------------------------------
    Eigen::Matrix<double, 6, 6> cov;
    cov.setZero();

    cov.block(0,0,3,3) = robot.Covariance_Bias_Gyro*robot.dt*robot.dt;
    cov.block(3,3,3,3) = robot.Covariance_Bias_Acc*robot.dt*robot.dt;

    Eigen::Matrix<double, 6, 6> sqrt_info;
    sqrt_info.setZero();

    double eps = 1e-30;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(cov);
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    sqrt_info = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose();
    //std::cout << "Test bias sqrt_info: "<< (sqrt_info.transpose()*sqrt_info*cov - Eigen::MatrixXd::Identity(6,6)).norm() << std::endl;


    //-------------------Big jacobian block--------------------------------
    Eigen::Matrix<double, 6,30> big_jacobian;
    big_jacobian.setZero();
    big_jacobian.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    big_jacobian.block(3,3,3,3) = Eigen::Matrix3d::Identity();
    big_jacobian.block(0,15,3,3) = -Eigen::Matrix3d::Identity();
    big_jacobian.block(3,18,3,3) = -Eigen::Matrix3d::Identity();


    //residual-----------------------------------------------------------------------------

    residual.setZero();
    residual.block(0,0,3,1) = bg_i - bg_j;
    residual.block(3,0,3,1) = ba_i - ba_j;

    residual = sqrt_info * residual;


    //jacobian----------------------------------------------------------------------------
        if (partial_i.data()) {
            //del_phi_i
            partial_i.setZero();
            partial_i = big_jacobian.leftCols(15);

            partial_i = sqrt_info * partial_i;

        }

        if (partial_j.data()) {
            //del_phi_i
            partial_j.setZero();
            partial_j = big_jacobian.rightCols(15);

            partial_j = sqrt_info * partial_j;

        }




}




void Factors::Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
                                               Eigen::Matrix<double,3,1> &residual,
                                               Eigen::Matrix<double,3,15> &partial_s,
                                               Eigen::Matrix<double,3,15> &partial_e,
                                               const Eigen::Matrix<double,21,1> &X_s,
                                               const Eigen::Matrix<double,21,1> &X_e,
                                               Eigen::Vector3d ENC_s,
                                               Eigen::Vector3d ENC_e,
                                               const Robot_Model_mini_cheetah &robot)
{


    //Assignment---------------------------------------------------------------------------
    Eigen::Matrix3d R_s,R_e;
    Eigen::Vector3d P_s, P_e;

    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            R_s(3*i+j) = X_s(6+ 3*i+j);
            R_e(3*i+j) = X_e(6+ 3*i+j);
        }
    }

    P_s = X_s.block(18,0,3,1);
    P_e = X_e.block(18,0,3,1);

    //cout<<"hihi!!!!"<<endl;
    // Calculating Covariance--------------------------------------------------------------
    Eigen::Matrix3d sqrt_info;
    sqrt_info.setIdentity();
    sqrt_info = sqrt_info * 0.01;

    double magnitude = 1/sqrt(robot.cov_contact_const)/(robot.dt);

    if(xyz == 1){
        sqrt_info(0,0) = magnitude;
        sqrt_info(1,1) = magnitude;
    }else if(xyz == 2){
        sqrt_info(2,2) = magnitude;
    }


    //cout<<"hihi!!!!!!!!!!"<<endl;
    //residual------------------------------------------------------------------------------
    residual.setZero();
    residual = (R_s*robot.Forward_Kinematics_Leg(ENC_s, leg_num+1) + P_s)
                - ( R_e*robot.Forward_Kinematics_Leg(ENC_e, leg_num+1) + P_e);
    residual = sqrt_info * residual;

    //Jacobians--------------------------------------------------------------------------------------

    if (partial_s.data()) {
        partial_s.setZero();
        partial_s.block(0,6,3,3) = -R_s * BasicFunctions_Estimator::Hat_so3( robot.Forward_Kinematics_Leg(ENC_s, leg_num+1) );
        partial_s.block(0,12,3,3) = Eigen::Matrix3d::Identity();

        partial_s = sqrt_info * partial_s;

    }

    if (partial_e.data()) {
        partial_e.setZero();
        partial_e.block(0,6,3,3) = R_e * BasicFunctions_Estimator::Hat_so3( robot.Forward_Kinematics_Leg(ENC_e, leg_num+1) );
        partial_e.block(0,12,3,3) = -Eigen::Matrix3d::Identity();

        partial_e = sqrt_info * partial_e;

    }

}