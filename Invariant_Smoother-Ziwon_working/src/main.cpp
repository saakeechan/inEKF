
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include "Z1_JH_estimator.hpp"
#include "Z1_estimator.hpp"
#include "Pseudo_Invariant_estimator.hpp"
#include "Traditional_estimator.hpp"
#include "InEKF_estimator.hpp"
#include <sstream>
#include <iomanip>

//#define filechecking



using std::cout;
using std::endl;

InEKFilter estimator_IEKF;
InEKFilter estimator_IEKF_SR;
Z1_estimator estimator_IS_1;
Z1_JH_estimator estimator_AIS_1;
Pseudo_Invariant_estimator estimator_PIS_1;
Traditional_estimator estimator_NIS_1;
Z1_estimator estimator_IS_f;
Z1_JH_estimator estimator_AIS_f;
Z1_JH_estimator estimator_AIS_1f;
Z1_JH_estimator estimator_AIS_f1;
Pseudo_Invariant_estimator estimator_PIS_f;
Traditional_estimator estimator_NIS_f;

int main() {

    double dt = 0.005;
    int starting_point = 2000;
    std::string file_name = "20220623_IEKF_01";
    int time_length = 27000;
    bool SR = true;
    double slip_thr = 0.3;
    double lt_v_th = 0.3;
    double lt_a_th = 40;
    //    double slip_thr = 0.4;
//    double slip_thr = 0.5;
    bool VCC = false;
    double cov_amplifier = 5;
    bool retraction_flag = false;


    int max_backpp_no = 3;
    double backpp_rate = 0.3;
    int max_it_no = 10;
    double convergence_cond = 1e-3;

    Eigen::Matrix<double, 12, 1> Estimator_Covariances;


    double gyro_exp = -5, acc_exp = -1, slip_exp = -1, contact_exp = -4, encoder_exp = -8; // for paper
    double bg_exp = -10, ba_exp = -7;
//    double bg_exp = -7, ba_exp = -3;
    double pri_ori_exp = -10, pri_vel_exp = -10, pri_pos_exp = -10;
    double pri_bg_exp = -10, pri_ba_exp = -10;


    Estimator_Covariances << pow(10, gyro_exp), pow(10, acc_exp), pow(10, slip_exp), pow(10, contact_exp), pow(10,encoder_exp), //gyro, acc, contact, encoder
            pow(10, bg_exp), pow(10, ba_exp), //bias gyro, bias acc
            pow(10, pri_ori_exp), pow(10, pri_vel_exp), pow(10, pri_pos_exp), //prior ori, vel, pos, contact
            pow(10, pri_bg_exp), pow(10, pri_ba_exp); //prior bias gyro, acc

    Eigen::Matrix<double, 16, 1> initial_condition;
    initial_condition << 0.0, 0.0, 0, // Px,Py,Pz
            0, 0, 0, 0, // roll, pitch, yaw, (No meaning)
            0, 0, 0,     //Vx, Vy, Vz
            0, 0, 0,     //bgx, bgy, bgz
            0, 0, 0;    //bax, bay, baz


    int sample_no = 1;
    int data_no = 5;
    bool IEKF_flag = true;
    bool IS_f_flag = false, NIS_f_flag = false, AIS_ff_flag = false;
    bool IS_1_flag = false, NIS_1_flag = false, AIS_1f_flag = false;
    bool AIS_f1_flag = false, AIS_11_flag = false;

    bool PIS_f_flag = false, PIS_1_flag = false;

    std::string cov_info;
    cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(contact_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";
//
    Eigen::Matrix<double, 30, 1> Sensor_;
    Sensor_.setZero();
    Eigen::Matrix<bool, 4, 1> Contact_;
    Contact_.setZero();
    ROBOT_STATES state_;

    bool varying_cov = true;
    double var_cov_start = contact_exp, var_cov_end = contact_exp;
    //var start랑 end 둘다 log10(5*pow(10,-5)) 로 하고, initial sample no랑 txt파일 바꾸고, pri cov = -10 /-5, contact = -3 /  pri cov = -5/7, contact = -log10(5*pow(10,-5))






    // //Running data
    // ifstream myfile;
    // myfile.open("../initial_condition_0_0.txt");

    // if (myfile.is_open()) {
    //     cout << "File opened" << endl;
    // } else {
    //     cout << "No" << endl;
    //     return 1;
    // }

    // //string trash;
    // //std::getline(myfile, trash);

    // string line;
    // string temp = "";
    // int column_index, row_index;
    // row_index = 0;

    // while(std::getline(myfile, line)) { //while there is a line

    //     if (row_index >= sample_no) {
    //         break;
    //     }

    //     column_index = 0;
    //     for (int i = 0; i < line.size(); i++) { // for each character in rowstring
    //         if (!isblank(line[i])) { // if it is not blank, do this
    //             string d(1, line[i]); // convert character to string
    //             temp.append(d); // append the two strings
    //         } else {
    //             initial_condition(column_index) = stod(temp);  // convert string to double
    //             temp = ""; // reset the capture
    //             column_index++; // increment b cause we have a new number
    //         }
    //     }

    //     initial_condition(column_index) = stod(temp);
    //     temp = "";
    //     row_index++; // onto next row
    //     cout << "init : " << row_index << endl;





    //     ifstream myfile_data;
    //     myfile_data.open("../data_list.txt");

    //     //string trash;
    //     //std::getline(myfile, trash);

    //     string line_data;
    //     file_name = "";
    //     int column_index_data, row_index_data;
    //     row_index_data = 0;

    //     while (std::getline(myfile_data, line_data)) { //while there is a line

    //         if (row_index_data >= data_no) {
    //             break;
    //         }
    //         file_name = "";
    //         column_index_data = 0;
    //         for (int i = 0; i < line_data.size(); i++) { // for each character in rowstring
    //             if (!isblank(line_data[i])) { // if it is not blank, do this
    //                 string d(1, line_data[i]); // convert character to string
    //                 file_name.append(d); // append the two strings
    //             } else {
    //                 column_index_data++; // increment b cause we have a new number
    //             }
    //         }
    //         row_index_data++; // onto next row

    //         cout << "file name : " << file_name << endl;



    if (varying_cov == true) {

        for (int var_cov_exp = int(var_cov_start); var_cov_exp <= int(var_cov_end); var_cov_exp++) {

            if (var_cov_start == var_cov_end) {

                Estimator_Covariances(3) = pow(10, var_cov_start);

                cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(var_cov_start, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
                            + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
                            + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";
            }else{

                Estimator_Covariances(3) = pow(10, var_cov_exp);

                cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(var_cov_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
                            + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
                            + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
                            BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";
            }


            if (IEKF_flag) {

                estimator_IEKF.robot.leg_no = 4;
                estimator_IEKF.Optimization_Epsilon = 1e-3;
                estimator_IEKF.Max_Iteration = max_it_no;

                estimator_IEKF.NUM_OF_TRASH_DATA = starting_point;
                estimator_IEKF.Call_File(file_name);
                estimator_IEKF.slip_rejection_mode = SR;
                estimator_IEKF.slip_threshold = slip_thr;
                estimator_IEKF.variable_contact_cov_mode = VCC;
                estimator_IEKF.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_IEKF.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_IEKF.Onestep(Sensor_, Contact_, state_);
                }

                estimator_IEKF.do_SAVE_Z1_all(cov_info);
                cout << "Reached after estimator_IEKF.do_SAVE_Z1_all(cov_info)" << endl;

            }


            if (IS_f_flag) {

                estimator_IS_f.robot.leg_no = 4;
                estimator_IS_f.Optimization_Epsilon = convergence_cond;
                estimator_IS_f.Max_Iteration = max_it_no;
                estimator_IS_f.Max_backpropagate_num = max_backpp_no;
                estimator_IS_f.backppgn_rate = backpp_rate;

                estimator_IS_f.NUM_OF_TRASH_DATA = starting_point;
                estimator_IS_f.Call_File(file_name);
                estimator_IS_f.slip_rejection_mode = SR;
                estimator_IS_f.slip_threshold = slip_thr;
                estimator_IS_f.variable_contact_cov_mode = VCC;
                estimator_IS_f.cov_amplifier = cov_amplifier;

                estimator_IS_f.long_term_v_threshold = lt_v_th;
                estimator_IS_f.long_term_a_threshold = lt_a_th;

                estimator_IS_f.Retract_All_flag = retraction_flag;
                //robot.define(REAL_ROBOT);
                estimator_IS_f.Initialize(dt, Estimator_Covariances, initial_condition);

                for (int time = 0; time < time_length; time++) {
                    if(time%1000==0)
                    {
                        cout<<"now step "<<time<<endl;
                    }
                    estimator_IS_f.Onestep(Sensor_, Contact_, state_);
                }

                estimator_IS_f.do_SAVE_Z1_all(cov_info);

            }


            if (PIS_f_flag) {

                estimator_PIS_f.robot.leg_no = 4;
                estimator_PIS_f.Optimization_Epsilon = convergence_cond;
                estimator_PIS_f.Max_Iteration = max_it_no;
                estimator_PIS_f.Max_backpropagate_num = max_backpp_no;
                estimator_PIS_f.backppgn_rate = backpp_rate;

                estimator_PIS_f.NUM_OF_TRASH_DATA = starting_point;
                estimator_PIS_f.Call_File(file_name);
                estimator_PIS_f.Retract_All_flag = retraction_flag;
                estimator_PIS_f.slip_rejection_mode = SR;
                estimator_PIS_f.slip_threshold = slip_thr;
                estimator_PIS_f.variable_contact_cov_mode = VCC;
                estimator_PIS_f.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_PIS_f.Initialize(dt, Estimator_Covariances, initial_condition);

                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_PIS_f.Onestep(Sensor_, Contact_, state_);
                }

                estimator_PIS_f.do_SAVE_Z1_all(cov_info);

            }


            if (NIS_f_flag) {

                estimator_NIS_f.robot.leg_no = 4;
                estimator_NIS_f.Optimization_Epsilon = convergence_cond;
                estimator_NIS_f.Max_Iteration = max_it_no;
                estimator_NIS_f.Max_backpropagate_num = max_backpp_no;
                estimator_NIS_f.backppgn_rate = backpp_rate;

                estimator_NIS_f.NUM_OF_TRASH_DATA = starting_point;
                estimator_NIS_f.Call_File(file_name);
                estimator_NIS_f.slip_rejection_mode = SR;
                estimator_NIS_f.slip_threshold = slip_thr;
                estimator_NIS_f.variable_contact_cov_mode = VCC;
                estimator_NIS_f.cov_amplifier = cov_amplifier;

                estimator_NIS_f.long_term_v_threshold = lt_v_th;
                estimator_NIS_f.long_term_a_threshold = lt_a_th;

                //robot.define(REAL_ROBOT);
                estimator_NIS_f.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_NIS_f.Onestep(Sensor_, Contact_, state_);
                }

                estimator_NIS_f.do_SAVE_Z1_all(cov_info);
            }

            if (AIS_ff_flag) {

                estimator_AIS_f.robot.leg_no = 4;
                estimator_AIS_f.Optimization_Epsilon = convergence_cond;
                estimator_AIS_f.Max_Iteration = max_it_no;
                estimator_AIS_f.Max_backpropagate_num = max_backpp_no;
                estimator_AIS_f.backppgn_rate = backpp_rate;

                estimator_AIS_f.NUM_OF_TRASH_DATA = starting_point;
                estimator_AIS_f.Call_File(file_name);
                estimator_AIS_f.Retract_All_flag = retraction_flag;
                estimator_AIS_f.slip_rejection_mode = SR;
                estimator_AIS_f.slip_threshold = slip_thr;
                estimator_AIS_f.variable_contact_cov_mode = VCC;
                estimator_AIS_f.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_AIS_f.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    if(time%10==0)
                    {
                        cout<<"now step "<<time<<endl;
                    }
                    estimator_AIS_f.Onestep(Sensor_, Contact_, state_);
                }

                estimator_AIS_f.do_SAVE_Z1_all(cov_info);

            }


            if (IS_1_flag) {

                estimator_IS_1.robot.leg_no = 4;
                estimator_IS_1.Optimization_Epsilon = convergence_cond;
                estimator_IS_1.Max_Iteration = 1;
                estimator_IS_1.Max_backpropagate_num = 1000;
                estimator_IS_1.backppgn_rate = backpp_rate;

                estimator_IS_1.NUM_OF_TRASH_DATA = starting_point;
                estimator_IS_1.Call_File(file_name);
                estimator_IS_1.slip_rejection_mode = SR;
                estimator_IS_1.slip_threshold = slip_thr;
                estimator_IS_1.variable_contact_cov_mode = VCC;
                estimator_IS_1.cov_amplifier = cov_amplifier;

                estimator_IS_1.long_term_v_threshold = lt_v_th;
                estimator_IS_1.long_term_a_threshold = lt_a_th;

                estimator_IS_1.Retract_All_flag = false;
                //robot.define(REAL_ROBOT);
                estimator_IS_1.Initialize(dt, Estimator_Covariances, initial_condition);

                for (int time = 0; time < time_length; time++) {
                    if(time%10==0)
                    {
                        cout<<"now step "<<time<<endl;
                    }
                    estimator_IS_1.Onestep(Sensor_, Contact_, state_);
                }

                estimator_IS_1.do_SAVE_Z1_all(cov_info);

            }
            //

            if (PIS_1_flag) {

                estimator_PIS_1.robot.leg_no = 4;
                estimator_PIS_1.Optimization_Epsilon = convergence_cond;
                estimator_PIS_1.Max_Iteration = 1;
                estimator_PIS_1.Max_backpropagate_num = 1000;
                estimator_PIS_1.backppgn_rate = backpp_rate;

                estimator_PIS_1.NUM_OF_TRASH_DATA = starting_point;
                estimator_PIS_1.Call_File(file_name);
                estimator_PIS_1.Retract_All_flag = false;
                estimator_PIS_1.slip_rejection_mode = SR;
                estimator_PIS_1.slip_threshold = slip_thr;
                estimator_PIS_1.variable_contact_cov_mode = VCC;
                estimator_PIS_1.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_PIS_1.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    if(time%10==0)
                    {
                        cout<<"now step "<<time<<endl;
                    }
                    //cout<<"now step "<<time<<endl;
                    estimator_PIS_1.Onestep(Sensor_, Contact_, state_);
                }

                estimator_PIS_1.do_SAVE_Z1_all(cov_info);

            }

            if (AIS_11_flag) {

                estimator_AIS_1.robot.leg_no = 4;
                estimator_AIS_1.Optimization_Epsilon = convergence_cond;
                estimator_AIS_1.Max_Iteration = 1;
                estimator_AIS_1.Max_backpropagate_num = 1000;
                estimator_AIS_1.backppgn_rate = backpp_rate;

                estimator_AIS_1.NUM_OF_TRASH_DATA = starting_point;
                estimator_AIS_1.Call_File(file_name);
                estimator_AIS_1.Retract_All_flag = retraction_flag;
                estimator_AIS_1.slip_rejection_mode = SR;
                estimator_AIS_1.slip_threshold = slip_thr;
                estimator_AIS_1.variable_contact_cov_mode = VCC;
                estimator_AIS_1.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_AIS_1.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_AIS_1.Onestep(Sensor_, Contact_, state_);
                }

                estimator_AIS_1.do_SAVE_Z1_all(cov_info);

            }

            if (AIS_1f_flag) {

                estimator_AIS_1f.robot.leg_no = 4;
                estimator_AIS_1f.Optimization_Epsilon = convergence_cond;
                estimator_AIS_1f.Max_Iteration = 1;
                estimator_AIS_1f.Max_backpropagate_num = max_backpp_no;
                estimator_AIS_1f.backppgn_rate = backpp_rate;

                estimator_AIS_1f.NUM_OF_TRASH_DATA = starting_point;
                estimator_AIS_1f.Call_File(file_name);
                estimator_AIS_1f.Retract_All_flag = retraction_flag;
                estimator_AIS_1f.slip_rejection_mode = SR;
                estimator_AIS_1f.slip_threshold = slip_thr;
                estimator_AIS_1f.variable_contact_cov_mode = VCC;
                estimator_AIS_1f.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_AIS_1f.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_AIS_1f.Onestep(Sensor_, Contact_, state_);
                }

                estimator_AIS_1f.do_SAVE_Z1_all(cov_info);

            }

            if (AIS_f1_flag) {

                estimator_AIS_f1.robot.leg_no = 4;
                estimator_AIS_f1.Optimization_Epsilon = convergence_cond;
                estimator_AIS_f1.Max_Iteration = max_it_no;
                estimator_AIS_f1.Max_backpropagate_num = 1000;
                estimator_AIS_f1.backppgn_rate = backpp_rate;

                estimator_AIS_f1.NUM_OF_TRASH_DATA = starting_point;
                estimator_AIS_f1.Call_File(file_name);
                estimator_AIS_f1.Retract_All_flag = retraction_flag;
                estimator_AIS_f1.slip_rejection_mode = SR;
                estimator_AIS_f1.slip_threshold = slip_thr;
                estimator_AIS_f1.variable_contact_cov_mode = VCC;
                estimator_AIS_f1.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_AIS_f1.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_AIS_f1.Onestep(Sensor_, Contact_, state_);
                }

                estimator_AIS_f1.do_SAVE_Z1_all(cov_info);

            }


            if (NIS_1_flag) {

                estimator_NIS_1.robot.leg_no = 4;
                estimator_NIS_1.Optimization_Epsilon = convergence_cond;
                estimator_NIS_1.Max_Iteration = 1;
                estimator_NIS_1.Max_backpropagate_num = 1000;
                estimator_NIS_1.backppgn_rate = backpp_rate;

                estimator_NIS_1.NUM_OF_TRASH_DATA = starting_point;
                estimator_NIS_1.Call_File(file_name);
                estimator_NIS_1.slip_rejection_mode = SR;
                estimator_NIS_1.slip_threshold = slip_thr;

                estimator_NIS_1.long_term_v_threshold = lt_v_th;
                estimator_NIS_1.long_term_a_threshold = lt_a_th;

                estimator_NIS_1.variable_contact_cov_mode = VCC;
                estimator_NIS_1.cov_amplifier = cov_amplifier;

                //robot.define(REAL_ROBOT);
                estimator_NIS_1.Initialize(dt, Estimator_Covariances, initial_condition);


                for (int time = 0; time < time_length; time++) {
                    //cout<<"now step "<<time<<endl;
                    estimator_NIS_1.Onestep(Sensor_, Contact_, state_);
                }

                estimator_NIS_1.do_SAVE_Z1_all(cov_info);
            }
        }
    }
    return 0;

}

    //     myfile_data.close();
    // }

    // myfile.close();




