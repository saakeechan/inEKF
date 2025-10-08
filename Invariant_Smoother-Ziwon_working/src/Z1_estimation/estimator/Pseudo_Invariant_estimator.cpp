
// Created by Ziwon Yoon on  2021-06-29.
// Last Update on       2021-06-30

#include "Pseudo_Invariant_estimator.hpp"
#include <string>

#include <malloc.h>

//---------------------------------




void Pseudo_Invariant_estimator::Call_File(std::string file_name)
{

    frame_count=0;
    //literally, counting frame number(fulfilment) in the window
    time_count=0;
    sliding_window_flag = false;


    file_info = file_name;


    ifstream myfile;

    myfile.open("../src/raisin_hound/houndlib/Invariant_Smoother.git/polished_data/" + file_name + "_sensordata.txt");

    //myfile.open("/home/rainbow/Desktop/Minicheetah_ws/Z1_JH_Estimator_Minicheetah_Test/third-party/Z1_JH_Estimator/src/Z1_estimation/test_data/" + file_name + "_sensordata.txt");
    //sensordata is sensor measurement 0~5 : IMU gyro/bias, 6~17 : 3 encoder values for 4 legs

    string line;
    string temp = "";

    string trash;
    for(int k=0; k<NUM_OF_TRASH_DATA; k++){
        std::getline(myfile, trash);
    }

    row_index = 0;
    while (std::getline(myfile, line)) { //while there is a line
        column_index = 0;
        for (int i = 0; i < line.size(); i++) { // for each character in rowstring
            if (!isblank(line[i])) { // if it is not blank, do this
                string d(1, line[i]); // convert character to string
                temp.append(d); // append the two strings
            } else {
                SensorData[row_index][column_index] = stod(temp);  // convert string to double
                temp = ""; // reset the capture
                column_index++; // increment b cause we have a new number
            }
        }

        SensorData[row_index][column_index] = stod(temp);
        if (row_index >= max_time){
            break;
        }
        temp = "";
        row_index++; // onto next row

    }
    myfile.close();




    myfile.open("../src/raisin_hound/houndlib/Invariant_Smoother.git/polished_data/" + file_name + "_groundtruth.txt");
    //groundtruth is true state value (motion capture)
    //0~2 is body position, 3~6 is quaternion


    gt_sd = 8;


    for(int k=0; k<NUM_OF_TRASH_DATA; k++){
        std::getline(myfile, trash);
    }

    row_index = 0; // row index
    string line2;
    while (std::getline(myfile, line2)) { //while there is a line
        column_index = 0;
        for (int i = 0; i<line2.size(); i++) { // for each character in rowstring
            if (!isblank(line2[i])) { // if it is not blank, do this
                string d(1, line2[i]); // convert character to string
                temp.append(d); // append the two strings

            } else {
                GroundTruth[row_index][column_index] = stod(temp);  // convert string to double
                temp = ""; // reset the capture
                column_index++; // increment b cause we have a new number
            }
        }

        GroundTruth[row_index][column_index] = stod(temp);
        if (row_index >= max_time){
            break;
        }
        temp = "";
        row_index++; // onto next row
    }
    myfile.close();


    cout<<"groundtruth row, col "<<row_index<<","<<column_index<<endl;


    textfile_flag = true;

}







void Pseudo_Invariant_estimator::Initialize(double _dt, Eigen::Matrix<double,12,1> &cov_val_setting, Eigen::Matrix<double, 16, 1> &initial_condition)
{

    sliding_window_flag = false;
    marginalization_flag = false;
    frame_count = 0;
    time_count = 0;
    robot.Covariance_Reset(cov_val_setting);
    robot.dt = _dt;

    robot.variable_contact_cov_mode = variable_contact_cov_mode;
    robot.cov_amplifier = cov_amplifier;
    robot.slip_rejection_mode = slip_rejection_mode;
    robot.slip_threshold = slip_threshold;

    cout<<robot.cov_enc_const<<endl;
    dt = robot.dt;
    gravity << 0,0, -9.80665;


    std::string convergence_cond;
    convergence_cond = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(Optimization_Epsilon, 1) + "_"
                       + BasicFunctions_Estimator::to_string_n_signficant_figures(Max_Iteration, 2) + ")"
                       + "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(backppgn_rate, 2) + "_"
                       + BasicFunctions_Estimator::to_string_n_signficant_figures(Max_backpropagate_num, 2) + ")";

    std::string mode;
    if(variable_contact_cov_mode == true){
        if(Retract_All_flag){
            mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")(RetO)";
        }else{
            mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")(RetX)";
        }
    }else if(slip_rejection_mode == true){
        if(Retract_All_flag){
            mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")(RetO)";
        }else{
            mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")(RetX)";
        }
    }else{
        if(Retract_All_flag){
            mode="(RetO)";
        }else{
            mode="(RetX)";
        }
    }



    estimator_info = mode + convergence_cond + "PIS";

    initial_info = "(" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[0], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[1], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[2], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[3], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[4], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[5], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[6], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[7], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[8], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[9], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[10], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[11], 2) + ")";




    if(textfile_flag)
    {



        RS[0].Position << GroundTruth[1+gt_sd][12]+initial_condition(0),GroundTruth[1+gt_sd][13]+initial_condition(1),GroundTruth[1+gt_sd][14]+initial_condition(2);
        RS[0].Velocity << GroundTruth[1+gt_sd][9]+initial_condition(7),GroundTruth[1+gt_sd][10]+initial_condition(8),GroundTruth[1+gt_sd][11]+initial_condition(9);

        RS[0].Bias_Gyro<< initial_condition(10), initial_condition(11), initial_condition(12);
        RS[0].Bias_Acc << initial_condition(13), initial_condition(14), initial_condition(15);

        RS[0].Rotation <<GroundTruth[1+gt_sd][0],GroundTruth[1+gt_sd][1],GroundTruth[1+gt_sd][2]
                ,GroundTruth[1+gt_sd][3],GroundTruth[1+gt_sd][4],GroundTruth[1+gt_sd][5]
                ,GroundTruth[1+gt_sd][6],GroundTruth[1+gt_sd][7],GroundTruth[1+gt_sd][8];


        Vector3d Euler = BasicFunctions_Estimator::Rotation_to_EulerZYX(RS[0].Rotation);
        Euler(0) = Euler(0)+ initial_condition[3];
        Euler(1) = Euler(1)+ initial_condition[4];
        Euler(2) = Euler(2)+ initial_condition[5];
        RS[0].Rotation = BasicFunctions_Estimator::EulerZYX_to_R_bw(Euler);



    }
    else
    {
        RS[0].Position = initial_condition.block(0,0,3,1);
        RS[0].Velocity.setZero();
        Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);

        RS[0].Rotation = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);
        RS[0].Bias_Acc.setZero();
        RS[0].Bias_Gyro.setZero();

    }



}





void Pseudo_Invariant_estimator::new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i)
{




    //sensor measurement reading
    if(textfile_flag)
    {

        int j=time_count + 1;


        //sensor measurement storage
        Estimation_Z.block(num_z*frame_count,0, num_z,1) << SensorData[j][0],SensorData[j][1],SensorData[j][2], SensorData[j][3],SensorData[j][4],SensorData[j][5],
                SensorData[j][6],SensorData[j][7],SensorData[j][8],SensorData[j][9],SensorData[j][10],SensorData[j][11],
                SensorData[j][12],SensorData[j][13],SensorData[j][14],SensorData[j][15],SensorData[j][16],SensorData[j][17],

                SensorData[j][18],SensorData[j][19],SensorData[j][20],SensorData[j][21],SensorData[j][22],SensorData[j][23],
                SensorData[j][24],SensorData[j][25],SensorData[j][26],SensorData[j][27],SensorData[j][28],SensorData[j][29];


        RS[frame_count].Contact << GroundTruth[j][19],GroundTruth[j][20],GroundTruth[j][21],GroundTruth[j][22];

    }else{


        //sensor measurement storage
        Estimation_Z.block(num_z*frame_count,0, num_z,1) <<Sensor_i.block(0,0,30,1);


        RS[frame_count].Contact = Contact_i;

    }






    //making state
    if(frame_count==0){//Initializing states which require sensor measurement necessarily

        for (int k=0; k<robot.leg_no; k++){

            RS[0].Slip(k) = false;
            //RS[0].Hard_Contact(k) = RS[0].Contact(k) - RS[0].Slip(k);
        }



    }else{//At inner timestep larger than 0

        Eigen::Vector3d IMU_Gyro_Previous, IMU_Acc_Previous, IMU_Gyro;
        Eigen::Matrix<double,12,1> ENCODER, ENCODERDOT;
        IMU_Gyro_Previous = Estimation_Z.block(num_z*(frame_count-1), 0, 3,1);
        IMU_Acc_Previous = Estimation_Z.block(num_z*(frame_count-1)+3, 0, 3,1);
        IMU_Gyro = Estimation_Z.block(num_z*(frame_count), 0, 3,1);
        ENCODER = Estimation_Z.block(num_z*(frame_count)+6, 0, 12,1);
        ENCODERDOT = Estimation_Z.block(num_z*(frame_count)+num_z_imu + num_z_encoder, 0, num_z_encoderdot,1);

        //Propagating state using IMU
        RS[frame_count].Bias_Gyro <<RS[frame_count-1].Bias_Gyro;
        RS[frame_count].Bias_Acc << RS[frame_count-1].Bias_Acc ;

        RS[frame_count].Rotation = RS[frame_count-1].Rotation * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt);
        JacobiSVD<Matrix3d> svd(RS[frame_count].Rotation, ComputeFullU|ComputeFullV);
        RS[frame_count].Rotation = svd.matrixU() * svd.matrixV().transpose();

        RS[frame_count].Velocity = RS[frame_count-1].Velocity
                + RS[frame_count-1].Rotation //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
                * (IMU_Acc_Previous - RS[frame_count-1].Bias_Acc)* dt
                + gravity * dt;
        //
        RS[frame_count].Position = RS[frame_count-1].Position + RS[frame_count-1].Velocity * dt
                + 0.5*RS[frame_count-1].Rotation * (IMU_Acc_Previous - RS[frame_count-1].Bias_Acc)* dt*dt + 0.5*gravity*dt*dt;



        for (int k=0; k<robot.leg_no; k++){

            //Estimating foot velocity
            RS[frame_count].d_v.block(3*k,0,3,1) = RS[frame_count].Velocity
                    + RS[frame_count].Rotation*robot.Jacobian_Leg(ENCODER.block<3,1>(3*k,0), k+1)*ENCODERDOT.block<3,1>(3*k,0)
                    + RS[frame_count].Rotation*BasicFunctions_Estimator::Hat_so3(IMU_Gyro - RS[frame_count].Bias_Gyro)*robot.Forward_Kinematics_Leg(ENCODER.block<3,1>(3*k,0), k+1);

            RS[frame_count].Slip(k) = false;

            //Slip rejection
            if (slip_rejection_mode == true && RS[frame_count].Contact(k) == true &&
                    RS[frame_count].d_v.block(3*k,0, 3,1).norm() > slip_threshold){
                RS[frame_count].Slip(k) = true;
            }
            RS[frame_count].Hard_Contact(k)  = RS[frame_count].Contact(k);
//            RS[frame_count].Hard_Contact(k)  = RS[frame_count].Contact(k)-RS[frame_count].Slip(k);


        }




    }



    //prior info storage
    if(frame_count==0){
        RS_Pri = RS[0];
    }


    delta_Zeta_Xi.setZero();


    int i = frame_count;

    //propagation covariance-------------------------------------------------------

    Eigen::MatrixXd prop_primitive_sqrt_info;
    prop_primitive_sqrt_info.resize(15, 15);
    prop_primitive_sqrt_info.setZero();

    prop_primitive_sqrt_info.block<3,3>(6,6) = robot.SQRT_INFO_Covariance_Gyro;
    prop_primitive_sqrt_info.block<3,3>(9,9) = robot.SQRT_INFO_Covariance_Acc;
    prop_primitive_sqrt_info.block<3,3>(12,12) = robot.SQRT_INFO_Covariance_Acc/(dt/sqrt(2));

    prop_primitive_sqrt_info.block<3,3>(0,0) = robot.SQRT_INFO_Covariance_Bias_Gyro;
    prop_primitive_sqrt_info.block<3,3>(3,3) = robot.SQRT_INFO_Covariance_Bias_Acc;

    fac_info[i].prop_primitive_sqrt_info = prop_primitive_sqrt_info;


    fac_info[i].Z = Estimation_Z.block(num_z*i,0, num_z,1);


    fac_info[i].leg_info.clear();

    int count=0;
    for (int k=0; k<robot.leg_no; k++){
        if (RS[i].Hard_Contact(k)==true){

            kinematics_info kin_k(k);

            kin_k.leg_num_in_state = count;
            Eigen::Vector3d ENC = fac_info[i].Z.block(6+3*k,0, 3,1);
            kin_k.fk_kin = robot.Forward_Kinematics_Leg(ENC, k+1);
            kin_k.meas_primitive_sqrt_info = robot.Jacobian_Leg(ENC,k+1) * robot.Covariance_Encoder * robot.Jacobian_Leg(ENC,k+1).transpose();


            fac_info[i].leg_info.insert(kin_k);

            count++;
        }
    }


    for(int p=0; p<=frame_count; p++){

        for (int k=0; k<robot.leg_no; k++){

            RS[p].d_v.block(3*k,0,3,1) = RS[p].Velocity
                    + RS[p].Rotation*robot.Jacobian_Leg(Estimation_Z.block<3,1>(num_z*p+6+3*k,0), k+1)*Estimation_Z.block<3,1>(num_z*p+num_z_imu+num_z_encoder+3*k,0)
                    + RS[p].Rotation*BasicFunctions_Estimator::Hat_so3(Estimation_Z.block<3,1>(num_z*p,0) - RS[p].Bias_Gyro)*robot.Forward_Kinematics_Leg(Estimation_Z.block<3,1>(num_z*p+6+3*k,0), k+1);

            RS[p].Slip(k) = false;

            //Slip rejection
            if (slip_rejection_mode == true && RS[p].Contact(k) == true &&
                    RS[p].d_v.block(3*k,0, 3,1).norm() > slip_threshold){
                RS[p].Slip(k) = true;
            }


        }
    }


    for(int p=0; p<frame_count; p++){

        fac_info[p].contact_cov_array = robot.Variable_Contact_Cov(RS[p].Hard_Contact, RS[p].d_v);

    }


}







void Pseudo_Invariant_estimator::Optimization_Solve()
{

    cost_array.setZero();

    double cost_before = 0;
    double cost = 0;
    total_backppgn_number=0;
    iteration_number=0;

    Eigen::Matrix<double,Eigen::Dynamic, 1> gradient;
    gradient.resize(15*(frame_count+1),1);

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> hessian;
    hessian.resize(15*(frame_count+1), 15*(frame_count+1));

    Pseudo_Inv_Factors factor;

    //Pseudo_Inv_Factors factor;
    if(marginalization_flag){


        factor.Marg_Initialize(RS, Estimation_Z.block(0,0,num_z*(frame_count+1),1), Marginalized_H, Marginalized_b, fac_info, robot);

        cost_array.setZero();
        cost_array(0) = cost;
        factor.Marg_Update_n_Get_Gradient_Hess_Cost(0, RS, delta_Zeta_Xi, hessian, gradient, cost_array);
        cost = cost_array(0);

        for(int iter=1; iter<=Max_Iteration; iter++)
        {

            cost_before = cost;


//            perturbation.block(0,0,15*(frame_count+1),1) = hessian.householderQr().solve(-gradient);
            perturbation.block(0,0,15*(frame_count+1),1) = hessian.llt().solve(-gradient);
            if(iter>=1)
            {

                Eigen::Matrix<double, -1,1> delta_Zeta_Xi_temp;
                delta_Zeta_Xi_temp.resizeLike(delta_Zeta_Xi);
                ROBOT_STATES_PSEUDO RS_temp [WINDOW_SIZE+1];
                double cost_temp = 0;
                double t = ALPHA;
                int backpropagate_count = 0;

                while(true) {


                    perturbation = t * perturbation;

                    for (int k=0; k<=WINDOW_SIZE; k++){
                        RS_temp[k] = RS[k];
                    }

                    delta_Zeta_Xi_temp = delta_Zeta_Xi;
                    delta_Zeta_Xi += perturbation;

                    if(Retract_All_flag){
                        retract_manifold(0);
                    }else if(marginalization_flag){
                        retract_manifold(1);
                    }else{
                        retract_manifold(0);
                    }

                    cost_array.setZero();
                    cost_array(0) = cost_temp;
                    factor.Marg_Update_n_Get_Gradient_Hess_Cost(0, RS, delta_Zeta_Xi, hessian, gradient, cost_array);
                    cost_temp = cost_array(0);
                    backpropagate_count++;

                    if(Max_backpropagate_num == 1000){
                        break;
                    }
                    else if(backpropagate_count>=Max_backpropagate_num)
                    {
                        delta_Zeta_Xi = delta_Zeta_Xi_temp;
                        for (int k=0; k<=WINDOW_SIZE; k++){
                            RS[k] = RS_temp[k];
                        }
                        //total_backppgn_number++;
                        break;
                    }
                    else if (cost_temp > (cost)){

                        delta_Zeta_Xi = delta_Zeta_Xi_temp;
                        for (int k=0; k<=WINDOW_SIZE; k++){
                            RS[k] = RS_temp[k];
                        }

                        t = t * backppgn_rate;
                        cost_temp =0;

                        if(backpropagate_count == 1) {
                            total_backppgn_number++;
                        }
                    }
                    else{
                        break;
                    }
                }

            }else{

                delta_Zeta_Xi +=perturbation;


                if(Retract_All_flag){
                    retract_manifold(0);
                }else if(marginalization_flag){
                    retract_manifold(1);
                }else{
                    retract_manifold(0);
                }

            }


            //update_dv(0);
            cost_array.setZero();
            cost_array(0) = cost;
            factor.Marg_Update_n_Get_Gradient_Hess_Cost(0, RS, delta_Zeta_Xi, hessian, gradient, cost_array);
            cost = cost_array(0);



            if((iter>1) && (sqrt(pow(cost-cost_before,2))/cost_before<Optimization_Epsilon || iter == Max_Iteration)) //Max_Itertation) )
            {
                iteration_number = iter;
                //cout<<"cost at marg is "<<cost<<endl;
                break;
            }else if(Max_Iteration ==1){
                iteration_number = iter;
                //cout<<"cost at marg is "<<cost<<endl;
                break;
            }


        }



    }else{



        factor.Batch_Initialize(RS, RS_Pri, Estimation_Z.block(0,0,num_z*(frame_count+1),1), fac_info, robot);

        factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost);
        //cout<< "opt time per iteration is "<<(double)(finish - start) / CLOCKS_PER_SEC<<" at it 1"<<endl;

        for(int iter=1; iter<=Max_Iteration; iter++)
        {

            //cout<<"cost at step "<<iter<<" is "<<cost<<endl;
            cost_before = cost;


//            delta_Zeta_Xi.block(0,0,15*(frame_count+1),1) = hessian.householderQr().solve(-gradient);
            delta_Zeta_Xi.block(0,0,15*(frame_count+1),1) = hessian.llt().solve(-gradient);
            if(iter>=1)
            {

                Eigen::Matrix<double, -1,1> delta_Zeta_Xi_temp;
                delta_Zeta_Xi_temp.resizeLike(delta_Zeta_Xi);
                ROBOT_STATES_PSEUDO RS_temp [WINDOW_SIZE+1];
                double cost_temp = 0;
                double t = ALPHA;
                int backpropagate_count = 0;

                while(true) {

                    delta_Zeta_Xi = t * delta_Zeta_Xi;

                    delta_Zeta_Xi_temp = delta_Zeta_Xi;
                    for (int k=0; k<=WINDOW_SIZE; k++){
                        RS_temp[k] = RS[k];
                    }
                    retract_manifold(0);

                    factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost_temp);
                    backpropagate_count++;

                    if(Max_backpropagate_num == 1000){
                        break;
                    }
                    else if(backpropagate_count>=Max_backpropagate_num)
                    {
                        delta_Zeta_Xi.setZero();
                        for (int k=0; k<=WINDOW_SIZE; k++){
                            RS[k] = RS_temp[k];
                        }
                        //total_backppgn_number++;
                        break;
                    }
                    else if (cost_temp > (cost)){

                        delta_Zeta_Xi = delta_Zeta_Xi_temp;
                        for (int k=0; k<=WINDOW_SIZE; k++){
                            RS[k] = RS_temp[k];
                        }

                        t = t * backppgn_rate;
                        cost_temp = 0;

                        if(backpropagate_count == 1) {
                            total_backppgn_number++;
                        }

                    }
                    else{
                        break;
                    }
                }

            }else{
                retract_manifold(0);

            }

            //update_dv(0);
            factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost);


            if((iter>1) && (sqrt(pow(cost-cost_before,2))/cost_before<Optimization_Epsilon || iter == Max_Iteration)) //Max_Itertation) )
            {
                iteration_number = iter;
                //cout<<"cost at marg is "<<cost<<endl;
                break;
            }else if(Max_Iteration ==1){
                iteration_number = iter;
                //cout<<"cost at marg is "<<cost<<endl;
                break;
            }


        }



    }





    /// Schur Complement Method for Marginalization
    if(frame_count == WINDOW_SIZE)
    {


        SAVE_Z1[idx_cost][time_count] = cost_array(0);
        SAVE_Z1[idx_cost+1][time_count] = cost_array(1);
        SAVE_Z1[idx_cost+2][time_count] = cost_array(2);
        SAVE_Z1[idx_cost+3][time_count] = cost_array(3);

        if(marginalization_flag){
            factor.Marg_Update_n_Get_Gradient_Hess_Cost(1, RS, delta_Zeta_Xi, hessian, gradient, cost_array);
        }else{
            factor.Batch_Update_n_Get_Gradient_Hess_Cost(1, RS, hessian, gradient, cost);
        }

        Eigen::Matrix<double,15, 15> HMM;
        HMM.setZero();
        Eigen::Matrix<double,15, 15> HMR;
        HMR.setZero();
        Eigen::Matrix<double,15, 15> HRR;
        HRR.setZero();


        HMM = hessian.block(0,0,15, 15);
        HMR = hessian.block(0,15, 15, 15);
        HRR = hessian.block(15, 15, 15, 15);

//        Marginalized_H = HRR - HMR.transpose()*HMM.householderQr().solve(HMR);
        Marginalized_H = HRR - HMR.transpose()*HMM.llt().solve(HMR);

        Marginalized_b = gradient.block(15, 0, 15,1)
//                - HMR.transpose()*HMM.householderQr().solve(gradient.block(0,0, 15,1));
        - HMR.transpose()*HMM.llt().solve(gradient.block(0,0, 15,1));


        double eps = 1e-20;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Marginalized_H);
        Eigen::VectorXd S = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        ///Notwithstanding its name, Marginalized_H is not H, but J s.t. J'*J=H
        Marginalized_H = S_sqrt.asDiagonal() * saes.eigenvectors().transpose();
        Marginalized_b = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * Marginalized_b;

        marginalization_flag = true;

    }


}





void Pseudo_Invariant_estimator::retract_manifold(int start_frame) {



    ////////////////////////////////////////////////////////////////////////////

    for (int i=start_frame; i<=frame_count; i++)
    {

        Eigen::Matrix<double,5,5> X_s;
        X_s.setIdentity();

        X_s.block<3,3>(0,0) = RS[i].Rotation;
        X_s.block<3,1>(0,3) = RS[i].Velocity;
        X_s.block<3,1>(0,4) = RS[i].Position;
        X_s = BasicFunctions_Estimator::Expm_seK_Vec(delta_Zeta_Xi.block(15*i+6,0, 9,1), 2)*X_s;

        RS[i].Rotation = X_s.block<3,3>(0,0);
        JacobiSVD<Matrix3d> svd(RS[i].Rotation, ComputeFullU|ComputeFullV);
        RS[i].Rotation = svd.matrixU() * svd.matrixV().transpose();

        RS[i].Velocity = X_s.block<3,1>(0,3);
        RS[i].Position = X_s.block<3,1>(0,4);


        RS[i].Bias_Gyro = RS[i].Bias_Gyro + delta_Zeta_Xi.block(15*i,0, 3,1);
        RS[i].Bias_Acc = RS[i].Bias_Acc + delta_Zeta_Xi.block(15*i+3,0, 3,1);

        delta_Zeta_Xi.block(15*i,0,15,1).setZero();



    }


}



void Pseudo_Invariant_estimator::update_dv(int start_frame) {



    for (int i=start_frame; i<=frame_count; i++)
    {


        for (int k=0; k<robot.leg_no; k++){

            //Estimating foot velocity
            RS[i].d_v.block(3*k,0,3,1) = RS[i].Velocity
                    + RS[i].Rotation*robot.Jacobian_Leg(Estimation_Z.block<3,1>(num_z*i+6+3*k,0), k+1)*Estimation_Z.block<3,1>(num_z*i+num_z_imu+num_z_encoder+3*k,0)
                    + RS[i].Rotation*BasicFunctions_Estimator::Hat_so3(Estimation_Z.block<3,1>(num_z*i,0) - RS[i].Bias_Gyro)*robot.Forward_Kinematics_Leg(Estimation_Z.block<3,1>(num_z*i+6+3*k,0), k+1);


            if (slip_rejection_mode == true && RS[i].Contact(k) == true &&
                RS[i].d_v.block(3*k,0, 3,1).norm() > slip_threshold){
                RS[i].Slip(k) = true;
            }
            else
            {
                RS[i].Slip(k) = false;
            }

        }




    }



}

void Pseudo_Invariant_estimator::send_states(ROBOT_STATES &state_)
{


    //Sending Estimated States----------------------------------------------------------------------------

    Eigen::Vector3d W_pos_imu2bd = RS[frame_count].Rotation * robot.IMU2BD;

    state_.Position = RS[frame_count].Position + W_pos_imu2bd;

    Eigen::Vector3d w_gyro;
    w_gyro = RS[frame_count].Rotation*(Estimation_Z.block(num_z*(frame_count), 0, 3,1)-RS[frame_count].Bias_Gyro);

    state_.Velocity = RS[frame_count].Velocity + w_gyro.cross(W_pos_imu2bd);
    state_.Bias_Gyro = RS[frame_count].Bias_Gyro;
    state_.Bias_Acc = RS[frame_count].Bias_Acc;
    state_.Rotation= RS[frame_count].Rotation;



    if(frame_count==0)
    {
        state_.Hard_Contact = RS[frame_count].Hard_Contact;
        state_.Contact = RS[frame_count].Contact;
        state_.Slip = RS[frame_count].Slip;
        state_.d_v = RS[frame_count].d_v;
    }
    else
    {
        state_.Hard_Contact = RS[frame_count-1].Hard_Contact;
        state_.Contact = RS[frame_count-1].Contact;
        state_.Slip = RS[frame_count-1].Slip;
        state_.d_v = RS[frame_count-1].d_v;
    }



}









void Pseudo_Invariant_estimator::sliding_window()
{

    Estimation_Z.block(0,0,num_z*WINDOW_SIZE,1) = Estimation_Z.block(num_z,0,num_z*WINDOW_SIZE,1);
    delta_Zeta_Xi.setZero();

    for(int i=0; i<WINDOW_SIZE; i++)
    {

        std::swap(RS[i],RS[i+1]);

        std::swap(fac_info[i],fac_info[i+1]);


    }

}



void Pseudo_Invariant_estimator::SAVE_onestep_Z1(int cnt)
{
    if(cnt<SAVEMAXCNT)
    {
        if(textfile_flag){
            //Sending True States----------------------------------------------------------------------------
            int j=time_count + 1;

            Eigen::Matrix3d temp_R;
            temp_R << GroundTruth[j+gt_sd][0],GroundTruth[j+gt_sd][1],GroundTruth[j+gt_sd][2]
                    ,GroundTruth[j+gt_sd][3],GroundTruth[j+gt_sd][4],GroundTruth[j+gt_sd][5]
                    ,GroundTruth[j+gt_sd][6],GroundTruth[j+gt_sd][7],GroundTruth[j+gt_sd][8];

            Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);
            for(int i=0;i<3;i++)
            {
                for(int k=0;k<3;k++)
                {
                    SAVE_Z1[idx_TRUE_Rotation+i*3+k][cnt] = GroundTruth[j+gt_sd][3*i+k];
                }
            }

            for(int i=0;i<3;i++)
            {
                SAVE_Z1[idx_TRUE_Position+i][cnt] = GroundTruth[j+gt_sd][12+i];
                SAVE_Z1[idx_TRUE_Velocity+i][cnt] = GroundTruth[j+gt_sd][9+i];
                SAVE_Z1[idx_TRUE_Bias_Gyro+i][cnt] =0;
                SAVE_Z1[idx_TRUE_Bias_Acc+i][cnt] = 0;
                SAVE_Z1[idx_TRUE_rpy+i][cnt] = temp_eul(i);
            }


            for(int i=0;i<robot.leg_no;i++)
            {
                SAVE_Z1[idx_TRUE_Contact+i][cnt] = GroundTruth[j+gt_sd][19+i];
                SAVE_Z1[idx_TRUE_Slip+i][cnt] = RS[frame_count].Slip(i);
                SAVE_Z1[idx_TRUE_Hard_Contact+i][cnt] = RS[frame_count].Hard_Contact(i);
            }

        }else{

            ///True state saving
        }


        //Sending Estimated States----------------------------------------------------------------------------

        for(int i=0;i<robot.leg_no;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Contact+i][cnt] = RS[frame_count].Contact(i);
            SAVE_Z1[idx_ESTIMATED_Slip+i][cnt] = RS[frame_count].Slip(i);
            SAVE_Z1[idx_ESTIMATED_Hard_Contact+i][cnt] = RS[frame_count].Hard_Contact(i);

        }



        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(RS[frame_count].Rotation);
        for(int i=0;i<3;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Position+i][cnt] = RS[frame_count].Position(i);
            SAVE_Z1[idx_ESTIMATED_Velocity+i][cnt] = RS[frame_count].Velocity(i);
            SAVE_Z1[idx_ESTIMATED_Bias_Gyro+i][cnt] = RS[frame_count].Bias_Gyro(i);
            SAVE_Z1[idx_ESTIMATED_Bias_Acc+i][cnt] = RS[frame_count].Bias_Acc(i);
            SAVE_Z1[idx_ESTIMATED_rpy+i][cnt] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_Z1[idx_ESTIMATED_Rotation+i*3+j][cnt] = RS[frame_count].Rotation(i,j);
            }
        }


        for(int i=0;i<robot.leg_no;i++){

            for(int k=0; k<3; k++){

                SAVE_Z1[idx_ESTIMATED_dv+ 3*i+k][cnt] =RS[frame_count].d_v(3*i+k);
            }
        }

        SAVE_Z1[idx_iteration_No][cnt] = iteration_number;
        SAVE_Z1[idx_backppgn_No][cnt] = total_backppgn_number;


    }
    else
    {
        cnt = SAVEMAXCNT-1;
        cout<<"over Max SAVE_cnt!!"<<endl;
    }
}




void Pseudo_Invariant_estimator::do_SAVE_Z1_all(std::string cov_info)
{


    cout<<"DO SAVE_Z1 ALL"<<endl;




    FILE* ffp = NULL;
    std::string str("../src/raisin_hound/houndlib/Invariant_Smoother.git/result/");



    str = str + estimator_info +"_"+ cov_info + "_" + initial_info + "_" + file_info + "x" + std::to_string(time_count) + "_" + std::to_string(WINDOW_SIZE) + ".txt";






    ffp= fopen(str.c_str(),"w");

    for(int i=0;i<time_count;i++)
    {
        for(int j=0;j<SAVEMAX;j++)
        {

            fprintf(ffp,"%f\t",SAVE_Z1[j][i]);
        }
        fprintf(ffp,"\n");
    }


    fclose(ffp);
    SAVE_cnt=0;

    cout<<str<<endl;
    cout<<"*** SAVE_Z1 DONE ***"<<endl;

}



void Pseudo_Invariant_estimator::Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i, ROBOT_STATES &state_){

    //std::cout<<endl<<"Now step "<<time_count<<" starts!"<<endl;



    clock_t start, finish;
    start = clock();

    new_measurement(Sensor_i,Contact_i);

    if(frame_count>0){
        Optimization_Solve();
    }


    //clock_t opt = clock();
    //cout<< "opt time is "<<(double)(opt - start) / CLOCKS_PER_SEC<<endl<<endl;

    if( RS[frame_count].Velocity.norm() > 100 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
       dt=0;
    }

    send_states(state_);

    if(textfile_flag){
        SAVE_onestep_Z1(time_count);
    }

    frame_count++;
    if(frame_count>WINDOW_SIZE)
    {
        frame_count = WINDOW_SIZE;
        sliding_window_flag = true;
    }
    if(textfile_flag)
    {
        time_count++;
    }


    if (sliding_window_flag){
        sliding_window();
    }


    //clock_t others = clock();
    //cout<< "others time is "<<(double)(others - opt) / CLOCKS_PER_SEC<<endl<<endl;


    finish = clock();
    time_per_step = (double)(finish - start) / CLOCKS_PER_SEC;

    if(textfile_flag){
        SAVE_Z1[idx_time_per_step][time_count-1] = time_per_step;
    }
    //cout<< "Operating time is "<<time_per_step<<", iter no is "<<iteration_number<<endl<<endl;


}
