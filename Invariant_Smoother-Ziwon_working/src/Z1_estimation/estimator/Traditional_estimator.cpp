
// Created by Ziwon Yoon on  2021-06-29.
// Last Update on       2021-06-30

#include "Traditional_estimator.hpp"
#include <string>

#include <malloc.h>

//---------------------------------



void Traditional_estimator::Call_File(std::string file_name)
{
    frame_count=0;
    //literally, counting frame number(fulfilment) in the window
    time_count=0;
    sliding_window_flag = false;


    file_info = file_name;


    ifstream myfile;
    std::string sensor_data_path = "../../polished_data/" + file_name + "_sensordata.txt";
    std::string groundtruth_data_path = "../../polished_data/" + file_name + "_groundtruth.txt";
    myfile.open(sensor_data_path);
        if (!myfile) {
            cerr << "Unable to open file: " << file_name << "_sensordata.txt" << endl;
            return;
        }
    cout << "file is read properly" << endl;
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




    myfile.open(groundtruth_data_path);
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
        for (int i = 0; i < line2.size(); i++) { // for each character in rowstring
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







void Traditional_estimator::Initialize(double _dt, Eigen::Matrix<double,12,1> cov_val_setting, Eigen::Matrix<double, 16, 1> initial_condition)
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

    robot.long_term_v_threshold = long_term_v_threshold;
    robot.long_term_a_threshold = long_term_a_threshold;
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
            mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + "_"
                 + BasicFunctions_Estimator::to_string_n_signficant_figures(long_term_a_threshold,2) + ")(RetO)";
            //mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")(RetO)";
        }else{
            mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + "_"
                 + BasicFunctions_Estimator::to_string_n_signficant_figures(long_term_a_threshold,2) + ")(RetX)";
            //mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")(RetX)";
        }
    }else{
        if(Retract_All_flag){
            mode="(RetO)";
        }else{
            mode="(RetX)";
        }
    }




    estimator_info = mode + convergence_cond + "NIS";
    initial_info = "(" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[0], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[1], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[2], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[3], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[4], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[5], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[6], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[7], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[8], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[9], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[10], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[11], 2) + ")";



    if(textfile_flag)
    {



        Position_s[0] << GroundTruth[1+gt_sd][12]+initial_condition(0),GroundTruth[1+gt_sd][13]+initial_condition(1),GroundTruth[1+gt_sd][14]+initial_condition(2);
        Velocity_s[0] << GroundTruth[1+gt_sd][9]+initial_condition(7),GroundTruth[1+gt_sd][10]+initial_condition(8),GroundTruth[1+gt_sd][11]+initial_condition(9);

        Bias_Gyro_s[0] << initial_condition(10), initial_condition(11), initial_condition(12);
        Bias_Acc_s[0] << initial_condition(13), initial_condition(14), initial_condition(15);

        Rotation_s[0] <<GroundTruth[1+gt_sd][0],GroundTruth[1+gt_sd][1],GroundTruth[1+gt_sd][2]
                ,GroundTruth[1+gt_sd][3],GroundTruth[1+gt_sd][4],GroundTruth[1+gt_sd][5]
                ,GroundTruth[1+gt_sd][6],GroundTruth[1+gt_sd][7],GroundTruth[1+gt_sd][8];

        Vector3d Euler = BasicFunctions_Estimator::Rotation_to_EulerZYX(Rotation_s[0]);
        Euler(0) = Euler(0)+ initial_condition[3];
        Euler(1) = Euler(1)+ initial_condition[4];
        Euler(2) = Euler(2)+ initial_condition[5];
        Rotation_s[0]= BasicFunctions_Estimator::EulerZYX_to_R_bw(Euler);


    }
    else
    {
        Position_s[0] = initial_condition.block(0,0,3,1);
        Velocity_s[0].setZero();
        Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);

        Rotation_s[0] = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);
        Bias_Acc_s[0].setZero();
        Bias_Gyro_s[0].setZero();

    }


}





void Traditional_estimator::new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i)
{

    //sensor measurement reading
    if(textfile_flag)
    {

        int j=time_count + 1;

        IMU_Gyro[frame_count] << SensorData[j][0],SensorData[j][1],SensorData[j][2];
        IMU_Acc[frame_count] << SensorData[j][3],SensorData[j][4],SensorData[j][5];

        ENCODER[frame_count] <<SensorData[j][6],SensorData[j][7],SensorData[j][8],SensorData[j][9],SensorData[j][10],SensorData[j][11],
                SensorData[j][12],SensorData[j][13],SensorData[j][14],SensorData[j][15],SensorData[j][16],SensorData[j][17];

        ENCODERDOT[frame_count] <<SensorData[j][18],SensorData[j][19],SensorData[j][20],SensorData[j][21],SensorData[j][22],SensorData[j][23],
                SensorData[j][24],SensorData[j][25],SensorData[j][26],SensorData[j][27],SensorData[j][28],SensorData[j][29];

        CONTACT_t[frame_count] << GroundTruth[j][19],GroundTruth[j][20],GroundTruth[j][21],GroundTruth[j][22];

    }else{

        IMU_Gyro[frame_count] = Sensor_i.block(0,0,3,1);
        IMU_Acc[frame_count] = Sensor_i.block(3,0,3,1);

        ENCODER[frame_count] = Sensor_i.block(6,0,12,1);
        //ENCODER[frame_count] << Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1);

        ENCODERDOT[frame_count] =Sensor_i.block(18,0,12,1);
        CONTACT_t[frame_count] = Contact_i;

    }






    //making state
    if(frame_count==0){//Initializing states which require sensor measurement necessarily

        for (int k=0; k<robot.leg_no; k++){

            SLIP_t[0][k] = false;
            HARD_CONTACT_t[0][k] = CONTACT_t[0](k)-SLIP_t[0](k);
        }

        //estimating foot position
        for (int k=0; k<robot.leg_no; k++){
            d_s[frame_count].block(3*k,0,3,1) = Position_s[frame_count] + Rotation_s[frame_count]*robot.Forward_Kinematics_Leg(ENCODER[frame_count].block<3,1>(3*k,0), k+1);
        }



    }else{//At inner timestep larger than 0

        //Propagating state using IMU
        Bias_Gyro_s[frame_count] << Bias_Gyro_s[frame_count-1];
        Bias_Acc_s[frame_count] << Bias_Acc_s[frame_count-1];

        Rotation_s[frame_count] = Rotation_s[frame_count-1] * BasicFunctions_Estimator::Expm_Vec((IMU_Gyro[frame_count-1] - Bias_Gyro_s[frame_count-1])*dt);
        JacobiSVD<Matrix3d> svd(Rotation_s[frame_count], ComputeFullU|ComputeFullV);
        Rotation_s[frame_count] = svd.matrixU() * svd.matrixV().transpose();

        Velocity_s[frame_count] = Velocity_s[frame_count-1]
                + Rotation_s[frame_count-1] //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro[frame_count-1] - Bias_Gyro_s[frame_count-1])*dt)
                * (IMU_Acc[frame_count-1] - Bias_Acc_s[frame_count-1])* dt
                + gravity * dt;
        //
        Position_s[frame_count] = Position_s[frame_count-1]
                + Velocity_s[frame_count-1] * dt
                + 0.5* Rotation_s[frame_count-1] //* BasicFunctions_Estimator::Gamma_2((IMU_Gyro[frame_count-1] - Bias_Gyro_s[frame_count-1])*dt)
                * (IMU_Acc[frame_count-1] - Bias_Acc_s[frame_count-1])* dt*dt + 0.5*gravity*dt*dt;



        for (int k=0; k<robot.leg_no; k++){

            //Estimating foot velocity
            dv_s[frame_count].block(3*k,0,3,1) = Velocity_s[frame_count]
                    + Rotation_s[frame_count]*robot.Jacobian_Leg(ENCODER[frame_count].block<3,1>(3*k,0), k+1)*ENCODERDOT[frame_count].block<3,1>(3*k,0)
                    + Rotation_s[frame_count]*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - Bias_Gyro_s[frame_count])*robot.Forward_Kinematics_Leg(ENCODER[frame_count].block<3,1>(3*k,0), k+1);


            SLIP_t[frame_count](k) = false;

            //Slip rejection
            if (slip_rejection_mode == true && CONTACT_t[frame_count](k) == true &&
                    dv_s[frame_count].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[frame_count](k) = true;
            }
            HARD_CONTACT_t[frame_count](k)  = CONTACT_t[frame_count](k);
//            HARD_CONTACT_t[frame_count](k)  = CONTACT_t[frame_count](k)-SLIP_t[frame_count](k);
        }



        //estimating foot position
        for (int k=0; k<robot.leg_no; k++){
            if (HARD_CONTACT_t[frame_count](k) && HARD_CONTACT_t[frame_count-1](k)){
                d_s[frame_count].block(3*k,0,3,1) = d_s[frame_count-1].block(3*k,0,3,1);

            }else{
                d_s[frame_count].block(3*k,0,3,1) = Position_s[frame_count] + Rotation_s[frame_count]*robot.Forward_Kinematics_Leg(ENCODER[frame_count].block<3,1>(3*k,0), k+1);
            }
        }


    }






    //sensor measurement storage
    Estimation_Z.block(num_z*frame_count,0, num_z_imu,1) <<IMU_Gyro[frame_count], IMU_Acc[frame_count];
    Estimation_Z.block(num_z*frame_count + num_z_imu,0, num_z_encoder,1) <<ENCODER[frame_count];
    Estimation_Z.block(num_z*frame_count + num_z_imu + num_z_encoder,0, num_z_encoderdot,1) <<ENCODERDOT[frame_count];


    //contact number storage
    contact_leg_num_array[frame_count]=0;
    for (int k=0; k<4; k++)
    {
        if (HARD_CONTACT_t[frame_count](k)==true)
        {
            contact_leg_num_array[frame_count]++;
        }
    }



    //state storage
    state_size[frame_count] = 6+15;
    if(frame_count==0){
        state_idx[frame_count]=0;
    }else{
        state_idx[frame_count] = state_idx[frame_count-1]+state_size[frame_count-1];
    }

    Estimation_X.conservativeResize(state_idx[frame_count]+state_size[frame_count],1);

    Estimation_X.block(state_idx[frame_count],0, 6,1) << Bias_Gyro_s[frame_count], Bias_Acc_s[frame_count];
    for(int i=0;i<3;i++)
    {
        for(int k=0;k<3;k++)
        {
            Estimation_X(state_idx[frame_count]+6 +3*i+k) = Rotation_s[frame_count](3*i+k);
            //Rowmajor
        }
    }
    Estimation_X.block(state_idx[frame_count]+6+9,0, 3,1) = Velocity_s[frame_count];
    Estimation_X.block(state_idx[frame_count]+6+9+3,0, 3,1) = Position_s[frame_count];



    if(frame_count==0){
        Estimation_X_init.resize(state_size[0],1);
        Estimation_X_init = Estimation_X.block(0,0, state_size[0],1);
    }



    //para_size might be smaller than it corresponding state_size by 6, since rotation parameter lives in 3d vectorspace, while state info stores rotation using 3D matrix.
    para_size[frame_count] = 6+9;
    if(frame_count==0){
        para_idx[frame_count]=0;
    }else{
        para_idx[frame_count] = para_idx[frame_count-1]+para_size[frame_count-1];
    }

    Estimation_dX.conservativeResize(para_idx[frame_count]+para_size[frame_count],1);
    Estimation_dX.block(para_idx[frame_count],0,15,1).setZero();



    for(int p=0; p<=frame_count; p++){

        for (int k=0; k<robot.leg_no; k++){

            //Estimating foot velocity
            dv_s[p].block(3*k,0,3,1) = Velocity_s[p]
                    + Rotation_s[p]*robot.Jacobian_Leg(Estimation_Z.block<3,1>(num_z*p+6+3*k,0), k+1)*Estimation_Z.block<3,1>(num_z*p+num_z_imu+num_z_encoder+3*k,0)
                    + Rotation_s[p]*BasicFunctions_Estimator::Hat_so3(Estimation_Z.block<3,1>(num_z*p,0) - Bias_Gyro_s[p])*robot.Forward_Kinematics_Leg(Estimation_Z.block<3,1>(num_z*p+6+3*k,0), k+1);


            SLIP_t[p](k) = false;

            //Slip rejection
            if (slip_rejection_mode == true && CONTACT_t[frame_count](k) == true &&
                dv_s[p].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[p](k) = true;
            }


        }

    }



//    for(int i=0; i<=frame_count; i++){
//        cout<<"contact_leg_num for frame "<<i<<" is "<<contact_leg_num_array[i]<<endl;
//        cout<<"para xi size for frame "<<i<<" is "<<para_size[frame_count]<<endl;
//        cout<<"X_s size for frame "<<i<<" is "<<state_size[frame_count]<<endl<<endl;
//    }
//    for(int i=0; i<=frame_count; i++){
//        cout<<"Contact at "<<i<<" is "<<endl<<CONTACT_t[i]<<endl;
//        cout<<"Hard Contact at "<<i<<" is "<<endl<<HARD_CONTACT_t[i]<<endl;
//    }


    //To check sparsity pattern
//    if(time_count == WINDOW_SIZE+1){
//        Hessian_S.resize(para_idx[frame_count]+para_size[frame_count],para_idx[frame_count]+para_size[frame_count]);
//        Hessian_S.setZero();
//    }


}







void Traditional_estimator::Optimization_Solve()
{

    cost_array.setZero();

    Eigen::MatrixXd Jacobian_Vector;
    Jacobian_Vector.resize(0,0);
    Jacobian_Vector.setZero();

    double cost_before = 0;
    double cost = 0;
    total_backppgn_number=0;
    iteration_number=0;

    Eigen::Matrix<double,Eigen::Dynamic, 1> gradient;
    gradient.resize(para_idx[frame_count]+para_size[frame_count],1);

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> hessian;
    hessian.resize(para_idx[frame_count]+para_size[frame_count],para_idx[frame_count]+para_size[frame_count]);


    Eigen::Matrix<double,-1,1> Estimation_dX_delta;
    Estimation_dX_delta.resizeLike(Estimation_dX);

    cost_array.setZero();
    cost_array(0) = cost;
    Estimation_Gradient_Hessian_Cost(Estimation_X,Estimation_X_init,Estimation_dX,Estimation_Z,Marginalized_H,Marginalized_b,hessian,gradient,cost_array, Jacobian_Vector);
    cost = cost_array(0);

    // Solving Nonlinear Optimization by Iterative Proximal Gauss Newton Method While Utilizing Manifold Structure
    for(int iter=1; iter<=Max_Iteration; iter++)
    {
        //cout<<"cost at step "<<iter<<" is "<<cost<<endl<<endl;

        cost_before = cost;

//        Estimation_dX_delta = hessian.householderQr().solve(-gradient);
        Estimation_dX_delta = hessian.llt().solve(-gradient);

        if(iter>=1)
        {

            double cost_temp=0;
            double t = ALPHA;
            int backpropagate_count = 0;

            while(true) {

                Estimation_dX_delta = t * Estimation_dX_delta;

                //store state
                Eigen::Matrix<double,-1,1> Estimation_dX_temp, Estimation_dX_delta_temp;
                Estimation_dX_temp.resizeLike(Estimation_dX); Estimation_dX_delta_temp.resizeLike(Estimation_dX_delta);
                Estimation_dX_temp = Estimation_dX; Estimation_dX_delta_temp = Estimation_dX_delta;


                Estimation_dX = Estimation_dX + Estimation_dX_delta;
                if(Retract_All_flag){
                    retract_manifold(0);
                }else if(marginalization_flag){
                    retract_manifold(1);
                }else{
                    retract_manifold(0);
                }

                cost_array.setZero();
                cost_array(0) = cost_temp;
                Estimation_Gradient_Hessian_Cost(Estimation_X,Estimation_X_init,Estimation_dX,Estimation_Z,Marginalized_H,Marginalized_b, hessian,gradient, cost_array, Jacobian_Vector);
                cost_temp = cost_array(0);

                backpropagate_count++;

                if(Max_backpropagate_num == 1000){
                    break;
                }
                else if(backpropagate_count>=Max_backpropagate_num)
                {

                    Estimation_dX= - (Estimation_dX_temp + Estimation_dX_delta_temp);

                    if(Retract_All_flag){
                        retract_manifold(0);
                    }else if(marginalization_flag){
                        retract_manifold(1);
                    }else{
                        retract_manifold(0);
                    }

                    Estimation_dX = Estimation_dX_temp;
                    Estimation_dX_delta = Estimation_dX_delta_temp;
                    //total_backppgn_number++;
                    break;
                }
                else if (cost_temp > (cost)){

                    Estimation_dX= - (Estimation_dX_temp + Estimation_dX_delta_temp);

                    if(Retract_All_flag){
                        retract_manifold(0);
                    }else if(marginalization_flag){
                        retract_manifold(1);
                    }else{
                        retract_manifold(0);
                    }

                    Estimation_dX = Estimation_dX_temp;
                    Estimation_dX_delta = Estimation_dX_delta_temp;
                    t = t * backppgn_rate;

                    if(backpropagate_count == 1) {
                        total_backppgn_number++;
                    }
                }
                else{
                    break;
                }
            }

        }else{

            Estimation_dX = Estimation_dX + Estimation_dX_delta;


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
        Estimation_Gradient_Hessian_Cost(Estimation_X,Estimation_X_init,Estimation_dX,Estimation_Z,Marginalized_H,Marginalized_b, hessian,gradient, cost_array, Jacobian_Vector);
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

    if (Jacobian_Vector.data()) {
        Eigen::VectorXd mean_Jac = Jacobian_Vector.rowwise().mean();
        Eigen::VectorXd std_jac;
        std_jac.resize(Jacobian_Vector.rows(), 1);
        std_jac.setZero();

        for (int k = 0; k < Jacobian_Vector.cols(); k++) {
            std_jac += (Jacobian_Vector.middleCols(k, 1) - mean_Jac).cwiseProduct(
                    Jacobian_Vector.middleCols(k, 1) - mean_Jac);
        }
        std_jac = (std_jac / Jacobian_Vector.cols()).cwiseSqrt();

        SAVE_Z1[idx_jac_std][time_count] = std_jac.mean();
    }else{
        SAVE_Z1[idx_jac_std][time_count] = 0;
    }


    SAVE_Z1[idx_cost][time_count] = cost_array(0);
    SAVE_Z1[idx_cost+1][time_count] = cost_array(1);
    SAVE_Z1[idx_cost+2][time_count] = cost_array(2);
    SAVE_Z1[idx_cost+3][time_count] = cost_array(3);

    /// Schur Complement Method for Marginalization
    if(frame_count == WINDOW_SIZE)
    {
        //cout<<"cost at final is "<<cost<<endl<<endl;

        Gradient_Hessian_Update_For_Marginalization(Estimation_X,Estimation_X_init,Estimation_dX,Estimation_Z,Marginalized_H,Marginalized_b,hessian,gradient,cost);

        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> HMM;
        HMM.resize(para_size[0], para_size[0]);
        HMM.setZero();
        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> HMR;
        HMR.resize(para_size[0], para_size[1]);
        HMR.setZero();
        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> HRR;
        HRR.resize(para_size[1], para_size[1]);
        HRR.setZero();


        HMM = hessian.block(0,0,para_size[0], para_size[0]);
        HMR = hessian.block(0,para_size[0], para_size[0], para_size[1]);
        HRR = hessian.block(para_size[0], para_size[0], para_size[1], para_size[1]);

        Marginalized_H.resize(para_size[1], para_size[1]);
//        Marginalized_H = HRR - HMR.transpose()*HMM.householderQr().solve(HMR);
        Marginalized_H = HRR - HMR.transpose()*HMM.llt().solve(HMR);

        Marginalized_b.resize(para_size[1], 1);
        Marginalized_b = gradient.block(para_size[0], 0, para_size[1],1)
//                - HMR.transpose()*HMM.householderQr().solve(gradient.block(0,0, para_size[0],1));
        - HMR.transpose()*HMM.llt().solve(gradient.block(0,0, para_size[0],1));

        double eps = 1e-10;
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



void Traditional_estimator::Estimation_Gradient_Hessian_Cost(Eigen::Matrix<double, Eigen::Dynamic,1> &X,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &X_init,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &dX,
                                               const Eigen::Matrix<double, num_z * (WINDOW_SIZE + 1), 1> &Z,
                                               const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &marginalized_H,
                                               const Eigen::Matrix<double, Eigen::Dynamic,1> &marginalized_b,
                                               Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &gradient,
                                               double &cost)
{


    gradient.setZero();
    Hessian.setZero();

    cost = 0;


    if (marginalization_flag) {

        Eigen::Matrix3d R_init, R0;
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                R_init(3*i+j) = X_init(6 +3*i+j);
                R0(3*i+j) = X(state_idx[0]+6+3*i+j);
            }
        }

        Eigen::MatrixXd Partial_marginalization;
        Partial_marginalization.resize(para_size[0], para_size[0]);
        Partial_marginalization = Marginalized_H;

//        Eigen::Matrix<double, 15,15> big_jacobian;
//        for(int k=0; k<5; k++){
//            big_jacobian.block(3*k,3*k, 3,3) = R_init;
//        }

//        big_jacobian.block(6,6,3,3) = BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(R_init.transpose() * R0)).inverse();

//        Partial_marginalization = Marginalized_H * big_jacobian.transpose();



        Eigen::VectorXd Residual_marginalization;
        Residual_marginalization.resize(para_size[0],1);
        Residual_marginalization.setZero();

        Eigen::Vector3d phi;
        phi = dX.block(6,0,3,1);

        Residual_marginalization.block(0,0,6,1) += X.block(state_idx[0],0,6,1) - X_init.block(state_idx[0],0,6,1) + dX.block(0,0,6,1);
        Residual_marginalization.block(6,0,3,1) += BasicFunctions_Estimator::Logm_Vec(R_init.transpose() * R0*BasicFunctions_Estimator::Expm_Vec(phi));
        Residual_marginalization.block(9,0,6,1) += X.block(state_idx[0]+15,0,6,1) - X_init.block(state_idx[0]+15,0,6,1) + dX.block(9,0,6,1);


        //Residual_marginalization = Partial_marginalization * Residual_marginalization;
        Residual_marginalization = Marginalized_b + Marginalized_H * Residual_marginalization;





        Hessian.block(0,0, para_size[0], para_size[0]) += Partial_marginalization.transpose() * Partial_marginalization;
        gradient.block(0,0, para_size[0],1) += Partial_marginalization.transpose() * Residual_marginalization;
        cost += Residual_marginalization.transpose() * Residual_marginalization;

//std::cout<<"now marg residual is "<<cost<<endl;




        for(int i=0;i<frame_count;i++)
        {
            int j=i+1;

            std::vector<double> contact_cov_array = robot.Variable_Contact_Cov(HARD_CONTACT_t[i], dv_s[i]);

            for (int k=0; k<robot.leg_no; k++)
            {

                if (HARD_CONTACT_t[i](k) && HARD_CONTACT_t[j](k)){

                    Eigen::Matrix<double,3,1> Residual_Measurement;
                    Eigen::Matrix<double,3,15> Partial_Measurement_i;
                    Eigen::Matrix<double,3,15> Partial_Measurement_j;


                    factors.Measurement_Factor(Residual_Measurement, Partial_Measurement_i, Partial_Measurement_j,
                                                  X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                                  dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                                  Z.block(num_z*i +num_z_imu +3*k,0, 3,1),
                                                  Z.block(num_z*j +num_z_imu +3*k,0, 3,1),
                                                  k+1,
                                                  contact_cov_array[k],
                                                  robot);

                    gradient.block(para_idx[i],0, para_size[i],1)
                            += Partial_Measurement_i.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_i;
                    Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_j;

                    gradient.block(para_idx[j],0, para_size[j],1)
                            += Partial_Measurement_j.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_j;
                    Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_i;

                    cost += Residual_Measurement.transpose() * Residual_Measurement;

                }
            }

            //cout<<"Hessian is"<<endl;
    //BasicFunctions_Estimator::Show_mtrx(Hessian);



            Eigen::Matrix<double, 9,1> Residual_Propagation;
            Eigen::Matrix<double, 9,15> Partial_Propagation_i;
            Eigen::Matrix<double, 9,15> Partial_Propagation_j;

            factors.Propagation_Factor(Residual_Propagation, Partial_Propagation_i, Partial_Propagation_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       Z.middleRows(num_z*i, 6),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_i.transpose() * Residual_Propagation;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_j.transpose() * Residual_Propagation;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_i;

            cost += Residual_Propagation.transpose() * Residual_Propagation;

    //cout<<"Wow!!!"<<endl;


            Eigen::Matrix<double, 6,1> Residual_Propagation_Bias;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_i;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_j;

            factors.Bias_Factor(Residual_Propagation_Bias, Partial_Propagation_Bias_i, Partial_Propagation_Bias_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_Bias_i.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_Bias_j.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_i;

            cost += Residual_Propagation_Bias.transpose() * Residual_Propagation_Bias;

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
                        dv_mag_i = fabs(dv_s[i].block(3*k,0, 2,1).norm() );
                        da_mag_i = fabs( (dv_s[i].block(3*k,0, 2,1) - dv_s[i-1].block(3*k,0, 2,1) ).norm()/robot.dt );
                    }else if(xy_z == 2){
                        dv_mag_i = fabs(dv_s[i](3 * k + xy_z));
                        da_mag_i = fabs( (dv_s[i](3 * k + xy_z) - dv_s[i-1](3 * k + xy_z))/robot.dt );
                    }

                    //cout<<i<<endl;
                    if (contact_count == 0 && HARD_CONTACT_t[i](k)
                        && (dv_mag_i < v_threshold)
                        && ( da_mag_i < a_threshold)
                        ) {

                        end = i;
                        contact_count++;

                    } else if (contact_count > 0 && HARD_CONTACT_t[i](k)
                               && (dv_mag_i < v_threshold)
                               && ( da_mag_i < a_threshold)
                               ) {

                        contact_count++;

                    } else if (contact_count > 0 && (
                            !HARD_CONTACT_t[i](k)
                            || (dv_mag_i > v_threshold)
                            || (da_mag_i > a_threshold)
                            ) ) {

                        contact_count = 0;

                        start = i+1;
                    }

                    if ((start > 0) && (end - start > 1) ) {

//                        Eigen::Matrix<double,3,1> Residual_LT;
//                        Eigen::Matrix<double,3,15> Partial_LT_s;
//                        Eigen::Matrix<double,3,15> Partial_LT_e;
//
//
//                        factors.Long_Term_Stationary_Foot_Factor(start, end, xy_z, k,
//                                                                 Residual_LT, Partial_LT_s, Partial_LT_e,
//                                                                 X.block(state_idx[start],0, state_size[start],1), X.block(state_idx[end],0, state_size[end],1),
//                                                                 Z.block(num_z*start +num_z_imu +3*k,0, 3,1), Z.block(num_z*end +num_z_imu +3*k,0, 3,1),
//                                                                 robot);
//
//                        gradient.block(para_idx[start],0, para_size[start],1)
//                                += Partial_LT_s.transpose() * Residual_LT;
//                        Hessian.block(para_idx[start],para_idx[start], para_size[start],para_size[start])
//                                += Partial_LT_s.transpose() * Partial_LT_s;
//                        Hessian.block(para_idx[start],para_idx[end], para_size[start],para_size[end])
//                                += Partial_LT_s.transpose() * Partial_LT_e;
//
//                        gradient.block(para_idx[end],0, para_size[end],1)
//                                += Partial_LT_e.transpose() * Residual_LT;
//                        Hessian.block(para_idx[end],para_idx[end], para_size[end],para_size[end])
//                                += Partial_LT_e.transpose() * Partial_LT_e;
//                        Hessian.block(para_idx[end],para_idx[i], para_size[end],para_size[i])
//                                += Partial_LT_e.transpose() * Partial_LT_s;
//
//                        cost += Residual_LT.transpose() * Residual_LT;


                        start = 0;
                        end = -1;

                    }

                }
            }

        }




    }else{

        Eigen::Matrix<double,15,1> Residual_Prior;
        Eigen::Matrix<double,15,15> Partial_Prior;

        factors.Prior_Factor(Residual_Prior, Partial_Prior,
                             X.block(state_idx[0],0, state_size[0],1),
                             X_init.block(state_idx[0],0, state_size[0],1),
                             dX.block(para_idx[0],0, para_size[0],1),
                             robot);

        gradient.block(0,0, 15,1) += Partial_Prior.transpose() * Residual_Prior;
        Hessian.block(0,0, 15, 15) += Partial_Prior.transpose() * Partial_Prior;

        cost += Residual_Prior.transpose() * Residual_Prior;




        for(int i=0;i<frame_count;i++)
        {
            int j=i+1;

            std::vector<double> contact_cov_array = robot.Variable_Contact_Cov(HARD_CONTACT_t[i], dv_s[i]);

            for (int k=0; k<robot.leg_no; k++)
            {
                if (HARD_CONTACT_t[i](k) && HARD_CONTACT_t[j](k)){

                    Eigen::Matrix<double,3,1> Residual_Measurement;
                    Eigen::Matrix<double,3,15> Partial_Measurement_i;
                    Eigen::Matrix<double,3,15> Partial_Measurement_j;

                    factors.Measurement_Factor(Residual_Measurement, Partial_Measurement_i, Partial_Measurement_j,
                                                  X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                                  dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                                  Z.block(num_z*i +num_z_imu +3*k,0, 3,1),
                                                  Z.block(num_z*j +num_z_imu +3*k,0, 3,1),
                                                  k+1,
                                                  contact_cov_array[k],
                                                  robot);

                    gradient.block(para_idx[i],0, para_size[i],1)
                            += Partial_Measurement_i.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_i;
                    Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_j;

                    gradient.block(para_idx[j],0, para_size[j],1)
                            += Partial_Measurement_j.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_j;
                    Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_i;

                    cost += Residual_Measurement.transpose() * Residual_Measurement;
    //cout<<"Wow!!"<<endl;
                }
            }

            //cout<<"Hessian is"<<endl;
    //BasicFunctions_Estimator::Show_mtrx(Hessian);



            Eigen::Matrix<double, 9,1> Residual_Propagation;
            Eigen::Matrix<double, 9,15> Partial_Propagation_i;
            Eigen::Matrix<double, 9,15> Partial_Propagation_j;

            factors.Propagation_Factor(Residual_Propagation, Partial_Propagation_i, Partial_Propagation_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       Z.middleRows(num_z*i, 6),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_i.transpose() * Residual_Propagation;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_j.transpose() * Residual_Propagation;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_i;

            cost += Residual_Propagation.transpose() * Residual_Propagation;

    //cout<<"Wow!!!"<<endl;


            Eigen::Matrix<double, 6,1> Residual_Propagation_Bias;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_i;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_j;

            factors.Bias_Factor(Residual_Propagation_Bias, Partial_Propagation_Bias_i, Partial_Propagation_Bias_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_Bias_i.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_Bias_j.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_i;

            cost += Residual_Propagation_Bias.transpose() * Residual_Propagation_Bias;

        }

    }
}



void Traditional_estimator::Estimation_Gradient_Hessian_Cost(Eigen::Matrix<double, Eigen::Dynamic,1> &X,
                                                             Eigen::Matrix<double, Eigen::Dynamic,1> &X_init,
                                                             Eigen::Matrix<double, Eigen::Dynamic,1> &dX,
                                                             const Eigen::Matrix<double, num_z * (WINDOW_SIZE + 1), 1> &Z,
                                                             const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &marginalized_H,
                                                             const Eigen::Matrix<double, Eigen::Dynamic,1> &marginalized_b,
                                                             Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &Hessian,
                                                             Eigen::Matrix<double, Eigen::Dynamic,1> &gradient,
                                                             Eigen::Matrix<double,4,1> &cost_log, Eigen::MatrixXd &Jacobian_Vector)
{


    Eigen::Matrix<double,Eigen::Dynamic, 1> big_jacobian_by_vector;
    big_jacobian_by_vector.resize(0,1);
    big_jacobian_by_vector.setZero();

    gradient.setZero();
    Hessian.setZero();

    cost_log.setZero();


    if (marginalization_flag) {

        Eigen::Matrix3d R_init, R0;
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                R_init(3*i+j) = X_init(6 +3*i+j);
                R0(3*i+j) = X(state_idx[0]+6+3*i+j);
            }
        }

        Eigen::MatrixXd Partial_marginalization;
        Partial_marginalization.resize(para_size[0], para_size[0]);
        Partial_marginalization = Marginalized_H;

//        Eigen::Matrix<double, 15,15> big_jacobian;
//        for(int k=0; k<5; k++){
//            big_jacobian.block(3*k,3*k, 3,3) = R_init;
//        }

//        big_jacobian.block(6,6,3,3) = BasicFunctions_Estimator::Right_Jacobian_SO3(BasicFunctions_Estimator::Logm_Vec(R_init.transpose() * R0)).inverse();

//        Partial_marginalization = Marginalized_H * big_jacobian.transpose();



        Eigen::VectorXd Residual_marginalization;
        Residual_marginalization.resize(para_size[0],1);
        Residual_marginalization.setZero();

        Eigen::Vector3d phi;
        phi = dX.block(6,0,3,1);

        Residual_marginalization.block(0,0,6,1) += X.block(state_idx[0],0,6,1) - X_init.block(state_idx[0],0,6,1) + dX.block(0,0,6,1);
        Residual_marginalization.block(6,0,3,1) += BasicFunctions_Estimator::Logm_Vec(R_init.transpose() * R0*BasicFunctions_Estimator::Expm_Vec(phi));
        Residual_marginalization.block(9,0,6,1) += X.block(state_idx[0]+15,0,6,1) - X_init.block(state_idx[0]+15,0,6,1) + dX.block(9,0,6,1);


        //Residual_marginalization = Partial_marginalization * Residual_marginalization;
        Residual_marginalization = Marginalized_b + Marginalized_H * Residual_marginalization;





        Hessian.block(0,0, para_size[0], para_size[0]) += Partial_marginalization.transpose() * Partial_marginalization;
        gradient.block(0,0, para_size[0],1) += Partial_marginalization.transpose() * Residual_marginalization;
        cost_log(1) += Residual_marginalization.transpose() * Residual_marginalization;

//std::cout<<"now marg residual is "<<cost<<endl;




        for(int i=0;i<frame_count;i++)
        {
            int j=i+1;

            std::vector<double> contact_cov_array = robot.Variable_Contact_Cov(HARD_CONTACT_t[i], dv_s[i]);

            for (int k=0; k<robot.leg_no; k++)
            {

                if (HARD_CONTACT_t[i](k) && HARD_CONTACT_t[j](k)){

                    Eigen::Matrix<double,3,1> Residual_Measurement;
                    Eigen::Matrix<double,3,15> Partial_Measurement_i;
                    Eigen::Matrix<double,3,15> Partial_Measurement_j;


                    factors.Measurement_Factor(Residual_Measurement, Partial_Measurement_i, Partial_Measurement_j,
                                               X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                               dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                               Z.block(num_z*i +num_z_imu +3*k,0, 3,1),
                                               Z.block(num_z*j +num_z_imu +3*k,0, 3,1),
                                               k+1,
                                               contact_cov_array[k],
                                               robot);

                    gradient.block(para_idx[i],0, para_size[i],1)
                            += Partial_Measurement_i.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_i;
                    Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_j;

                    gradient.block(para_idx[j],0, para_size[j],1)
                            += Partial_Measurement_j.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_j;
                    Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_i;

                    cost_log(3) += Residual_Measurement.transpose() * Residual_Measurement;

                }
            }

            //cout<<"Hessian is"<<endl;
            //BasicFunctions_Estimator::Show_mtrx(Hessian);



            Eigen::Matrix<double, 9,1> Residual_Propagation;
            Eigen::Matrix<double, 9,15> Partial_Propagation_i;
            Eigen::Matrix<double, 9,15> Partial_Propagation_j;

            factors.Propagation_Factor(Residual_Propagation, Partial_Propagation_i, Partial_Propagation_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       Z.middleRows(num_z*i, 6),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_i.transpose() * Residual_Propagation;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_j.transpose() * Residual_Propagation;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_i;

            cost_log(2) += Residual_Propagation.transpose() * Residual_Propagation;

            //cout<<"Wow!!!"<<endl;


            Eigen::Matrix<double, 6,1> Residual_Propagation_Bias;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_i;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_j;

            factors.Bias_Factor(Residual_Propagation_Bias, Partial_Propagation_Bias_i, Partial_Propagation_Bias_j,
                                X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_Bias_i.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_Bias_j.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_i;

            cost_log(2) += Residual_Propagation_Bias.transpose() * Residual_Propagation_Bias;

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
                        dv_mag_i = fabs(dv_s[i].block(3*k,0, 2,1).norm() );
                        da_mag_i = fabs( (dv_s[i].block(3*k,0, 2,1) - dv_s[i-1].block(3*k,0, 2,1) ).norm()/robot.dt );
                    }else if(xy_z == 2){
                        dv_mag_i = fabs(dv_s[i](3 * k + xy_z));
                        da_mag_i = fabs( (dv_s[i](3 * k + xy_z) - dv_s[i-1](3 * k + xy_z))/robot.dt );
                    }

                    //cout<<i<<endl;
                    if (contact_count == 0 && HARD_CONTACT_t[i](k)
                        && (dv_mag_i < v_threshold)
                        && ( da_mag_i < a_threshold)
                            ) {

                        end = i;
                        contact_count++;

                    } else if (contact_count > 0 && HARD_CONTACT_t[i](k)
                               && (dv_mag_i < v_threshold)
                               && ( da_mag_i < a_threshold)
                            ) {

                        contact_count++;

                    } else if (contact_count > 0 && (
                            !HARD_CONTACT_t[i](k)
                            || (dv_mag_i > v_threshold)
                            || (da_mag_i > a_threshold)
                    ) ) {

                        contact_count = 0;

                        start = i+1;
                    }

                    if ((start > 0) && (end - start > 1) ) {

//                        Eigen::Matrix<double,3,1> Residual_LT;
//                        Eigen::Matrix<double,3,15> Partial_LT_s;
//                        Eigen::Matrix<double,3,15> Partial_LT_e;
//
//
//                        factors.Long_Term_Stationary_Foot_Factor(start, end, xy_z, k,
//                                                                 Residual_LT, Partial_LT_s, Partial_LT_e,
//                                                                 X.block(state_idx[start],0, state_size[start],1), X.block(state_idx[end],0, state_size[end],1),
//                                                                 Z.block(num_z*start +num_z_imu +3*k,0, 3,1), Z.block(num_z*end +num_z_imu +3*k,0, 3,1),
//                                                                 robot);
//
//                        gradient.block(para_idx[start],0, para_size[start],1)
//                                += Partial_LT_s.transpose() * Residual_LT;
//                        Hessian.block(para_idx[start],para_idx[start], para_size[start],para_size[start])
//                                += Partial_LT_s.transpose() * Partial_LT_s;
//                        Hessian.block(para_idx[start],para_idx[end], para_size[start],para_size[end])
//                                += Partial_LT_s.transpose() * Partial_LT_e;
//
//                        gradient.block(para_idx[end],0, para_size[end],1)
//                                += Partial_LT_e.transpose() * Residual_LT;
//                        Hessian.block(para_idx[end],para_idx[end], para_size[end],para_size[end])
//                                += Partial_LT_e.transpose() * Partial_LT_e;
//                        Hessian.block(para_idx[end],para_idx[i], para_size[end],para_size[i])
//                                += Partial_LT_e.transpose() * Partial_LT_s;
//
//                        cost_log(0) += Residual_LT.transpose() * Residual_LT;


                        start = 0;
                        end = -1;

                    }

                }
            }

        }




    }else{


        Eigen::Matrix<double,15,1> Residual_Prior;
        Eigen::Matrix<double,15,15> Partial_Prior;

        factors.Prior_Factor(Residual_Prior, Partial_Prior,
                             X.block(state_idx[0],0, state_size[0],1),
                             X_init.block(state_idx[0],0, state_size[0],1),
                             dX.block(para_idx[0],0, para_size[0],1),
                             robot);

        gradient.block(0,0, 15,1) += Partial_Prior.transpose() * Residual_Prior;
        Hessian.block(0,0, 15, 15) += Partial_Prior.transpose() * Partial_Prior;

        cost_log(1) += Residual_Prior.transpose() * Residual_Prior;

        double new_rows_no;

        new_rows_no = Partial_Prior.cols()*Partial_Prior.rows();
        VectorXd temp(Eigen::Map<VectorXd>(Partial_Prior.data(),new_rows_no));
        big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
        big_jacobian_by_vector.bottomRows(new_rows_no) = temp;


        for(int i=0;i<frame_count;i++)
        {
            int j=i+1;

            std::vector<double> contact_cov_array = robot.Variable_Contact_Cov(HARD_CONTACT_t[i], dv_s[i]);

            for (int k=0; k<robot.leg_no; k++)
            {
                if (HARD_CONTACT_t[i](k) && HARD_CONTACT_t[j](k)){

                    Eigen::Matrix<double,3,1> Residual_Measurement;
                    Eigen::Matrix<double,3,15> Partial_Measurement_i;
                    Eigen::Matrix<double,3,15> Partial_Measurement_j;

                    factors.Measurement_Factor(Residual_Measurement, Partial_Measurement_i, Partial_Measurement_j,
                                               X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                               dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                               Z.block(num_z*i +num_z_imu +3*k,0, 3,1),
                                               Z.block(num_z*j +num_z_imu +3*k,0, 3,1),
                                               k+1,
                                               contact_cov_array[k],
                                               robot);

                    gradient.block(para_idx[i],0, para_size[i],1)
                            += Partial_Measurement_i.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_i;
                    Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                            += Partial_Measurement_i.transpose() * Partial_Measurement_j;

                    gradient.block(para_idx[j],0, para_size[j],1)
                            += Partial_Measurement_j.transpose() * Residual_Measurement;
                    Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_j;
                    Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                            += Partial_Measurement_j.transpose() * Partial_Measurement_i;

                    cost_log(3) += Residual_Measurement.transpose() * Residual_Measurement;

                    double new_rows_no;

                    new_rows_no = Partial_Measurement_i.cols()*Partial_Measurement_i.rows();
                    VectorXd temp(Eigen::Map<VectorXd>(Partial_Measurement_i.data(),new_rows_no));
                    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
                    big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

                    new_rows_no = Partial_Measurement_j.cols()*Partial_Measurement_j.rows();
                    VectorXd temp2(Eigen::Map<VectorXd>(Partial_Measurement_j.data(),new_rows_no));
                    big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
                    big_jacobian_by_vector.bottomRows(new_rows_no) = temp2;
                    //cout<<"Wow!!"<<endl;
                }
            }

            //cout<<"Hessian is"<<endl;
            //BasicFunctions_Estimator::Show_mtrx(Hessian);



            Eigen::Matrix<double, 9,1> Residual_Propagation;
            Eigen::Matrix<double, 9,15> Partial_Propagation_i;
            Eigen::Matrix<double, 9,15> Partial_Propagation_j;

            factors.Propagation_Factor(Residual_Propagation, Partial_Propagation_i, Partial_Propagation_j,
                                       X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                       dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                       Z.middleRows(num_z*i, 6),
                                       robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_i.transpose() * Residual_Propagation;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_i.transpose() * Partial_Propagation_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_j.transpose() * Residual_Propagation;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_j.transpose() * Partial_Propagation_i;

            cost_log(2) += Residual_Propagation.transpose() * Residual_Propagation;

            double new_rows_no;

            new_rows_no = Partial_Propagation_i.cols()*Partial_Propagation_i.rows();
            VectorXd temp(Eigen::Map<VectorXd>(Partial_Propagation_i.data(),new_rows_no));
            big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
            big_jacobian_by_vector.bottomRows(new_rows_no) = temp;

            new_rows_no = Partial_Propagation_j.cols()*Partial_Propagation_j.rows();
            VectorXd temp2(Eigen::Map<VectorXd>(Partial_Propagation_j.data(),new_rows_no));
            big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
            big_jacobian_by_vector.bottomRows(new_rows_no) = temp2;


            //cout<<"Wow!!!"<<endl;


            Eigen::Matrix<double, 6,1> Residual_Propagation_Bias;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_i;
            Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_j;

            factors.Bias_Factor(Residual_Propagation_Bias, Partial_Propagation_Bias_i, Partial_Propagation_Bias_j,
                                X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                robot);

            gradient.block(para_idx[i],0, para_size[i],1)
                    += Partial_Propagation_Bias_i.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_i;
            Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                    += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_j;

            gradient.block(para_idx[j],0, para_size[j],1)
                    += Partial_Propagation_Bias_j.transpose() * Residual_Propagation_Bias;
            Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_j;
            Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                    += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_i;

            cost_log(2) += Residual_Propagation_Bias.transpose() * Residual_Propagation_Bias;

            new_rows_no = Partial_Propagation_Bias_i.cols()*Partial_Propagation_Bias_i.rows();
            VectorXd temp3(Eigen::Map<VectorXd>(Partial_Propagation_Bias_i.data(),new_rows_no));
            big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
            big_jacobian_by_vector.bottomRows(new_rows_no) = temp3;

            new_rows_no = Partial_Propagation_Bias_j.cols()*Partial_Propagation_Bias_j.rows();
            VectorXd temp4(Eigen::Map<VectorXd>(Partial_Propagation_Bias_j.data(),new_rows_no));
            big_jacobian_by_vector.conservativeResize(big_jacobian_by_vector.rows() + new_rows_no, 1);
            big_jacobian_by_vector.bottomRows(new_rows_no) = temp4;

        }

    }

    cost_log(0) += cost_log(1) + cost_log(2) + cost_log(3);


    //big_jacobian_by_vector = big_jacobian_by_vector/big_jacobian_by_vector.norm();

    Jacobian_Vector.conservativeResize(big_jacobian_by_vector.rows(), Jacobian_Vector.cols()+1);
    Jacobian_Vector.rightCols(1) = big_jacobian_by_vector;

}


void Traditional_estimator::retract_manifold(int start_frame) {





    for (int i=start_frame ;i<=frame_count; i++)
    {
        Bias_Gyro_s[i] = Bias_Gyro_s[i] + Estimation_dX.block(15*i+ 0,0,3,1);
        Bias_Acc_s[i] = Bias_Acc_s[i] + Estimation_dX.block(15*i+ 3,0,3,1);

        Velocity_s[i] = Velocity_s[i] + Estimation_dX.block(15*i+ 9,0,3,1);
        Position_s[i] = Position_s[i] + Estimation_dX.block(15*i+ 12,0,3,1);


        Eigen::Vector3d del_phi = Estimation_dX.block(15*i+ 6,0,3,1);
        Rotation_s[i] = Rotation_s[i] * BasicFunctions_Estimator::Expm_Vec(del_phi);
        JacobiSVD<Matrix3d> svd(Rotation_s[i], ComputeFullU|ComputeFullV);
        Rotation_s[i] = svd.matrixU() * svd.matrixV().transpose();


//cout<<"state at frame "<<i<<endl<<Rotation_s[i]<<" "<<Velocity_s[i].transpose()<<" "<<Position_s[i].transpose()
//   <<endl<<endl<<Bias_Gyro_s[i].transpose()<<" "<<Bias_Acc_s[i].transpose()<<endl;

        Estimation_X.block(21*i,0, 6,1) << Bias_Gyro_s[i], Bias_Acc_s[i];
        for(int j=0;j<3;j++)
        {
            for(int k=0;k<3;k++)
            {
                Estimation_X(21*i+ 6 +3*j+k) = Rotation_s[i](3*j+k);
                //Rowmajor
            }
        }
        Estimation_X.block(21*i +6+9,0, 3,1) = Velocity_s[i];
        Estimation_X.block(21*i +6+9+3,0, 3,1) = Position_s[i];


        Estimation_dX.block(15*i,0, 15,1).setZero();




    }


}


void Traditional_estimator::update_dv(int start_frame) {


    for (int i=start_frame ;i<=frame_count; i++)
    {

        for (int k=0; k<robot.leg_no; k++){

            //Estimating foot velocity
            dv_s[i].block(3*k,0,3,1) = Velocity_s[i]
                    + Rotation_s[i]*robot.Jacobian_Leg(Estimation_Z.block<3,1>(num_z*i+6+3*k,0), k+1)*Estimation_Z.block<3,1>(num_z*i+num_z_imu+num_z_encoder+3*k,0)
                    + Rotation_s[i]*BasicFunctions_Estimator::Hat_so3(Estimation_Z.block<3,1>(num_z*i,0) - Bias_Gyro_s[i])*robot.Forward_Kinematics_Leg(Estimation_Z.block<3,1>(num_z*i+6+3*k,0), k+1);


            if (slip_rejection_mode == true && CONTACT_t[i](k) == true &&
                dv_s[i].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[i](k) = true;
            }
            else
            {
                SLIP_t[i](k) = false;
            }

        }

    }

}




void Traditional_estimator::Gradient_Hessian_Update_For_Marginalization(Eigen::Matrix<double, Eigen::Dynamic,1> &X,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &X_init,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &dX,
                                               const Eigen::Matrix<double, num_z * (WINDOW_SIZE + 1), 1> &Z,
                                               const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &marginalized_H,
                                               const Eigen::Matrix<double, Eigen::Dynamic,1> &marginalized_b,
                                               Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &Hessian,
                                               Eigen::Matrix<double, Eigen::Dynamic,1> &gradient,
                                               double &cost)
{

    Hessian = -Hessian;
    gradient = -gradient;



    if(frame_count>1){


        int i=1;
        int j=i+1;

        std::vector<double> contact_cov_array = robot.Variable_Contact_Cov(HARD_CONTACT_t[i], dv_s[i]);

        for (int k=0; k<robot.leg_no; k++)
        {
            if (HARD_CONTACT_t[i](k) && HARD_CONTACT_t[j](k)){

                Eigen::Matrix<double,3,1> Residual_Measurement;
                Eigen::Matrix<double,3,15> Partial_Measurement_i;
                Eigen::Matrix<double,3,15> Partial_Measurement_j;

                factors.Measurement_Factor(Residual_Measurement, Partial_Measurement_i, Partial_Measurement_j,
                                           X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                           dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                           Z.block(num_z*i +num_z_imu +3*k,0, 3,1),
                                           Z.block(num_z*j +num_z_imu +3*k,0, 3,1),
                                           k+1,
                                           contact_cov_array[k],
                                           robot);

                gradient.block(para_idx[i],0, para_size[i],1)
                        += Partial_Measurement_i.transpose() * Residual_Measurement;
                Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                        += Partial_Measurement_i.transpose() * Partial_Measurement_i;
                Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                        += Partial_Measurement_i.transpose() * Partial_Measurement_j;

                gradient.block(para_idx[j],0, para_size[j],1)
                        += Partial_Measurement_j.transpose() * Residual_Measurement;
                Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                        += Partial_Measurement_j.transpose() * Partial_Measurement_j;
                Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                        += Partial_Measurement_j.transpose() * Partial_Measurement_i;

                cost += Residual_Measurement.transpose() * Residual_Measurement;

            }
        }

        //cout<<"Hessian is"<<endl;
        //BasicFunctions_Estimator::Show_mtrx(Hessian);



        Eigen::Matrix<double, 9,1> Residual_Propagation;
        Eigen::Matrix<double, 9,15> Partial_Propagation_i;
        Eigen::Matrix<double, 9,15> Partial_Propagation_j;

        factors.Propagation_Factor(Residual_Propagation, Partial_Propagation_i, Partial_Propagation_j,
                                   X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                                   dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                                   Z.middleRows(num_z*i, 6),
                                   robot);

        gradient.block(para_idx[i],0, para_size[i],1)
                += Partial_Propagation_i.transpose() * Residual_Propagation;
        Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                += Partial_Propagation_i.transpose() * Partial_Propagation_i;
        Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                += Partial_Propagation_i.transpose() * Partial_Propagation_j;

        gradient.block(para_idx[j],0, para_size[j],1)
                += Partial_Propagation_j.transpose() * Residual_Propagation;
        Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                += Partial_Propagation_j.transpose() * Partial_Propagation_j;
        Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                += Partial_Propagation_j.transpose() * Partial_Propagation_i;

        cost += Residual_Propagation.transpose() * Residual_Propagation;

        //cout<<"Wow!!!"<<endl;


        Eigen::Matrix<double, 6,1> Residual_Propagation_Bias;
        Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_i;
        Eigen::Matrix<double, 6,15> Partial_Propagation_Bias_j;

        factors.Bias_Factor(Residual_Propagation_Bias, Partial_Propagation_Bias_i, Partial_Propagation_Bias_j,
                            X.block(state_idx[i],0, state_size[i]+state_size[j],1),
                            dX.block(para_idx[i],0, para_size[i]+para_size[j],1),
                            robot);

        gradient.block(para_idx[i],0, para_size[i],1)
                += Partial_Propagation_Bias_i.transpose() * Residual_Propagation_Bias;
        Hessian.block(para_idx[i],para_idx[i], para_size[i],para_size[i])
                += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_i;
        Hessian.block(para_idx[i],para_idx[j], para_size[i],para_size[j])
                += Partial_Propagation_Bias_i.transpose() * Partial_Propagation_Bias_j;

        gradient.block(para_idx[j],0, para_size[j],1)
                += Partial_Propagation_Bias_j.transpose() * Residual_Propagation_Bias;
        Hessian.block(para_idx[j],para_idx[j], para_size[j],para_size[j])
                += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_j;
        Hessian.block(para_idx[j],para_idx[i], para_size[j],para_size[i])
                += Partial_Propagation_Bias_j.transpose() * Partial_Propagation_Bias_i;

        cost += Residual_Propagation_Bias.transpose() * Residual_Propagation_Bias;

    }



    Hessian = -Hessian;
    gradient = -gradient;



}










void Traditional_estimator::send_states(ROBOT_STATES &state_)
{




    //Sending Estimated States----------------------------------------------------------------------------

    Eigen::Vector3d W_pos_imu2bd = Rotation_s[frame_count] * robot.IMU2BD;

    state_.Position = Position_s[frame_count] + W_pos_imu2bd;

    Eigen::Vector3d w_gyro;
    w_gyro = Rotation_s[frame_count]*(Estimation_Z.block(num_z*(frame_count), 0, 3,1)-Bias_Gyro_s[frame_count]);

    state_.Velocity = Velocity_s[frame_count] + w_gyro.cross(W_pos_imu2bd);
    state_.Bias_Gyro = Bias_Gyro_s[frame_count];
    state_.Bias_Acc = Bias_Acc_s[frame_count];
    state_.Rotation= Rotation_s[frame_count];
    state_.Hard_Contact = HARD_CONTACT_t[frame_count];
    state_.Contact = CONTACT_t[frame_count];
    state_.Slip = SLIP_t[frame_count];
    state_.d = d_s[frame_count];

    if(frame_count==0)
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count];
        state_.Contact = CONTACT_t[frame_count];
        state_.Slip = SLIP_t[frame_count];
        state_.d_v = dv_s[frame_count];
    }
    else
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count-1];
        state_.Contact = CONTACT_t[frame_count-1];
        state_.Slip = SLIP_t[frame_count-1];
        state_.d_v = dv_s[frame_count-1];
    }

}








void Traditional_estimator::sliding_window()
{

    Estimation_X.block(0,0, state_idx[frame_count]+state_size[frame_count] - state_size[0],1)
            = Estimation_X.block(state_idx[1],0, state_idx[frame_count]+state_size[frame_count] - state_size[0],1);

    Estimation_X_init.resize(state_size[1],1);
    Estimation_X_init = Estimation_X.block(state_idx[0],0, state_size[0],1);

    Estimation_dX.setZero();

//    Estimation_dX.block(0,0, para_idx[frame_count]+para_size[frame_count] - para_size[0],1)
//            = Estimation_dX.block(para_idx[1],0, para_idx[frame_count]+para_size[frame_count] - para_size[0],1);


    Estimation_Z.block(0,0,num_z*WINDOW_SIZE,1) = Estimation_Z.block(num_z,0,num_z*WINDOW_SIZE,1);


    for(int i=0; i<WINDOW_SIZE; i++)
    {
        Position_s[i].swap(Position_s[i+1]);
        Velocity_s[i].swap(Velocity_s[i+1]);
        Rotation_s[i].swap(Rotation_s[i+1]);
        d_s[i].swap(d_s[i+1]);
        dv_s[i].swap(dv_s[i+1]);
        Bias_Acc_s[i].swap(Bias_Acc_s[i+1]);
        Bias_Gyro_s[i].swap(Bias_Gyro_s[i+1]);



        std::swap(IMU_Gyro[i],IMU_Gyro[i+1]);
        std::swap(IMU_Acc[i],IMU_Acc[i+1]);
        std::swap(ENCODER[i],ENCODER[i+1]);
        std::swap(ENCODERDOT[i],ENCODERDOT[i+1]);
        std::swap(HARD_CONTACT_t[i],HARD_CONTACT_t[i+1]);
        std::swap(CONTACT_t[i],CONTACT_t[i+1]);
        std::swap(SLIP_t[i],SLIP_t[i+1]);
        //cout<<"???"<<endl;
        //std::swap(para_Xi[i],para_Xi[i+1]);
        //std::iter_swap(para_Xi.begin() + i, para_Xi.begin() + i+1);
        //cout<<"Ok!"<<endl;
        //std::swap(para_delta_bias[i],para_delta_bias[i+1]);

        std::swap(contact_leg_num_array[i],contact_leg_num_array[i+1]);



        if(i>0){
            state_idx[i] = state_idx[i-1]+state_size[i];
        }else{
            state_idx[0]=0;
        }

        state_size[i]=state_size[i+1];

        if(i>0){
            para_idx[i] = para_idx[i-1]+para_size[i];
        }else{
            para_idx[0]=0;
        }

        para_size[i]=para_size[i+1];


    }

    //cout<<para_Xi.size()<<endl;
    //free(para_Xi[0]);
    //delete[] para_Xi[0];
    //cout<<para_Xi.size()<<endl;
    //para_Xi.erase(para_Xi.begin());
    //cout<<para_Xi.size()<<endl;
}



void Traditional_estimator::SAVE_onestep_Z1(int cnt)
{
    if(cnt<SAVEMAXCNT){
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
                SAVE_Z1[idx_TRUE_Slip+i][cnt] = SLIP_t[frame_count](i);
                SAVE_Z1[idx_TRUE_Hard_Contact+i][cnt] = HARD_CONTACT_t[frame_count](i);
            }

        }else{

            ///True state saving
        }


        //Sending Estimated States----------------------------------------------------------------------------

        for(int i=0;i<robot.leg_no;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Contact+i][cnt] = CONTACT_t[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Slip+i][cnt] = SLIP_t[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Hard_Contact+i][cnt] = HARD_CONTACT_t[frame_count](i);

        }



        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(Rotation_s[frame_count]);
        for(int i=0;i<3;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Position+i][cnt] = Position_s[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Velocity+i][cnt] = Velocity_s[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Bias_Gyro+i][cnt] = Bias_Gyro_s[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Bias_Acc+i][cnt] = Bias_Acc_s[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_rpy+i][cnt] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_Z1[idx_ESTIMATED_Rotation+i*3+j][cnt] = Rotation_s[frame_count](i,j);
            }
        }


        for(int i=0;i<robot.leg_no;i++){

            for(int k=0; k<3; k++){

                SAVE_Z1[idx_ESTIMATED_dv+ 3*i+k][cnt] = d_s[frame_count](3*i+k);
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







void Traditional_estimator::do_SAVE_Z1_all(std::string cov_info)
{



    cout<<"DO SAVE_Z1 ALL"<<endl;




    FILE* ffp = NULL;
    std::string str("../src/raisin_hound/houndlib/Invariant_Smoother.git/result/");



    str = str + estimator_info +"_"+ cov_info + "_" + initial_info + "_" + file_info + "x" + std::to_string(time_count) + "_" + std::to_string(WINDOW_SIZE) + ".txt";


    ffp= fopen(str.c_str(),"w");
    //        ffp= fopen("Z1_tesstt.txt","w");

    //        cout<<"FILE_OPEN: " << ffp<<endl;


    for(int i=0;i<time_count;i++)
    {
        //        cout << "i = " << i << endl;
        for(int j=0;j<SAVEMAX;j++)
        {

            fprintf(ffp,"%f\t",SAVE_Z1[j][i]);
        }
        fprintf(ffp,"\n");
        if(i/10000==0)
        {
            //printf("saving... %d / %d\n",i+1,time_count);
        }
    }

    fclose(ffp);

    SAVE_cnt=0;

    printf("%s\n",str.c_str());
    printf("*** SAVE_Z1 DONE ***\n");
    //    cout<<"!!"<<endl;

}




void Traditional_estimator::Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i, ROBOT_STATES &state_){

    //std::cout<<endl<<"Now step "<<time_count<<" starts!"<<endl;
    clock_t start, finish;
    start = clock();

    new_measurement(Sensor_i,Contact_i);

    if(frame_count>0){
    Optimization_Solve();
    }
    //test_estimator.retract_manifold();
    //clock_t ret = clock();
    //cout<< "ret time is "<<(double)(ret - opt) / CLOCKS_PER_SEC<<endl<<endl;

    send_states(state_);
    SAVE_onestep_Z1(time_count);

    if( Velocity_s[frame_count].norm() > 100 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
       dt=0;
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

    SAVE_Z1[idx_time_per_step][time_count] = time_per_step;
    //cout<< "Total operating time per step is "<<duration<<endl;


}

