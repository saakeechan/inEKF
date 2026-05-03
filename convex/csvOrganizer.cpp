/**
 * CSV Organizer for Robot Data
 * 
 * This program reads parsed data and reorganizes it into three separate CSV files:
 * 1. imu.csv - IMU measurements (gyroscope and accelerometer)
 * 2. metadata.csv - Metadata about the data sequence
 * 3. contacts.csv - Contact events and foot positions
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include "../src/utility/csvParser.hpp"

using namespace std;

/**
 * Structure to hold ground truth state at a given timestep
 */
struct GroundTruthState {
    double timestamp;
    
    // Position (world frame)
    double pos_x, pos_y, pos_z;
    
    // Orientation (roll, pitch, yaw in radians)
    double roll, pitch, yaw;
    
    // Linear velocity (world frame)
    double vel_x, vel_y, vel_z;
    
    // Quaternion (for completeness)
    double quat_w, quat_i, quat_j, quat_k;
    
    // Helper function to print ground truth
    void print() const {
        cout << "Ground Truth State at t = " << timestamp << " s:" << endl;
        cout << "  Position: [" << pos_x << ", " << pos_y << ", " << pos_z << "]" << endl;
        cout << "  Roll/Pitch/Yaw: [" << roll << ", " << pitch << ", " << yaw << "] rad" << endl;
        cout << "  Velocity: [" << vel_x << ", " << vel_y << ", " << vel_z << "]" << endl;
        cout << "  Quaternion: [" << quat_w << ", " << quat_i << ", " << quat_j << ", " << quat_k << "]" << endl;
    }
};

/**
 * Write IMU data to imu.csv
 * Format: sample_index, timestamp_s, omega_x, omega_y, omega_z, acc_x, acc_y, acc_z
 */
void writeIMUData(const DataStruct& data, const string& output_path) {
    ofstream file(output_path + "/imu.csv");
    
    if (!file.is_open()) {
        cerr << "Error: Could not create imu.csv at " << output_path << endl;
        return;
    }
    
    // Write header
    file << "sample_index,timestamp_s,omega_x,omega_y,omega_z,acc_x,acc_y,acc_z" << endl;
    
    // Write data rows
    file << fixed << setprecision(12);
    for (size_t i = 0; i < data.num_rows; i++) {
        file << i << ","
             << data.timestamp[i] << ","
             << data.gyro_body_x[i] << ","
             << data.gyro_body_y[i] << ","
             << data.gyro_body_z[i] << ","
             << data.acc_body_x[i] << ","
             << data.acc_body_y[i] << ","
             << data.acc_body_z[i] << endl;
    }
    
    file.close();
    cout << "Successfully created imu.csv with " << data.num_rows << " samples" << endl;
}

/**
 * Write metadata to metadata.csv
 * Format: key, value_0, value_1, ...
 */
void writeMetadata(const DataStruct& data, const string& output_path, const string& sequence_name) {
    ofstream file(output_path + "/metadata.csv");
    
    if (!file.is_open()) {
        cerr << "Error: Could not create metadata.csv at " << output_path << endl;
        return;
    }
    
    // Write header
    file << "key,value_0,value_1" << endl;
    
    // Write metadata rows
    file << "format_version,2" << endl;
    file << "sequence_name," << sequence_name << endl;
    file << "foot_names,left_foot,right_foot" << endl;
    file << "dense_contact_stream,1" << endl;
    file << "contact_state_value,1" << endl;
    file << "timestamp_source,record" << endl;
    
    file.close();
    cout << "Successfully created metadata.csv" << endl;
}

/**
 * Write contact data to contacts.csv
 * Format: event_index, timestamp_s, foot_index, foot_name, is_new_contact, body_x, body_y, body_z
 */
void writeContactData(const DataStruct& data, const string& output_path) {
    ofstream file(output_path + "/contacts.csv");
    
    if (!file.is_open()) {
        cerr << "Error: Could not create contacts.csv at " << output_path << endl;
        return;
    }
    
    // Write header
    file << "event_index,timestamp_s,foot_index,foot_name,is_new_contact,body_x,body_y,body_z" << endl;
    
    // Track previous contact states to detect new contacts
    bool prev_lfoot_contact = false;
    bool prev_rfoot_contact = false;
    int event_index = 0;
    
    file << fixed << setprecision(12);
    
    // Write contact events for each timestep where contact is active
    for (size_t i = 0; i < data.num_rows; i++) {
        bool lfoot_in_contact = data.lfoot_contact[i];
        bool rfoot_in_contact = data.rfoot_contact[i];
        
        // Check if we need to write contact events for this timestep
        bool write_event = lfoot_in_contact || rfoot_in_contact;
        
        if (write_event) {
            // Detect new contacts
            bool lfoot_new_contact = lfoot_in_contact && !prev_lfoot_contact;
            bool rfoot_new_contact = rfoot_in_contact && !prev_rfoot_contact;
            
            // Write left foot contact if active
            if (lfoot_in_contact) {
                file << event_index << ","
                     << data.timestamp[i] << ","
                     << 0 << ","  // foot_index: 0 for left foot
                     << "left_foot" << ","
                     << (lfoot_new_contact ? 1 : 0) << ","
                     << data.lfoot_pos_x[i] << ","
                     << data.lfoot_pos_y[i] << ","
                     << data.lfoot_pos_z[i] << endl;
            }
            
            // Write right foot contact if active
            if (rfoot_in_contact) {
                file << event_index << ","
                     << data.timestamp[i] << ","
                     << 1 << ","  // foot_index: 1 for right foot
                     << "right_foot" << ","
                     << (rfoot_new_contact ? 1 : 0) << ","
                     << data.rfoot_pos_x[i] << ","
                     << data.rfoot_pos_y[i] << ","
                     << data.rfoot_pos_z[i] << endl;
            }
            
            event_index++;
        }
        
        // Update previous contact states
        prev_lfoot_contact = lfoot_in_contact;
        prev_rfoot_contact = rfoot_in_contact;
    }
    
    file.close();
    cout << "Successfully created contacts.csv with " << event_index << " contact events" << endl;
}

/**
 * Write ground truth trajectory to groundtruth.csv
 * Format: timestamp_s, pos_x, pos_y, pos_z, roll, pitch, yaw, vel_x, vel_y, vel_z, quat_w, quat_i, quat_j, quat_k
 */
void writeGroundTruthTrajectory(const DataStruct& data, const string& output_path) {
    ofstream file(output_path + "/groundtruth.csv");
    
    if (!file.is_open()) {
        cerr << "Error: Could not create groundtruth.csv at " << output_path << endl;
        return;
    }
    
    // Write header
    file << "timestamp_s,pos_x,pos_y,pos_z,roll,pitch,yaw,vel_x,vel_y,vel_z,quat_w,quat_i,quat_j,quat_k" << endl;
    
    file << fixed << setprecision(12);
    
    // Write ground truth for all timesteps
    for (size_t i = 0; i < data.num_rows; i++) {
        file << data.timestamp[i] << ","
             << data.pos_x[i] << ","
             << data.pos_y[i] << ","
             << data.pos_z[i] << ","
             << data.roll[i] << ","
             << data.pitch[i] << ","
             << data.yaw[i] << ","
             << data.vel_x[i] << ","
             << data.vel_y[i] << ","
             << data.vel_z[i] << ","
             << data.quat_w[i] << ","
             << data.quat_i[i] << ","
             << data.quat_j[i] << ","
             << data.quat_k[i] << endl;
    }
    
    file.close();
    cout << "Successfully created groundtruth.csv with " << data.num_rows << " timesteps" << endl;
}

int main(int argc, char* argv[]) {
    // Default paths
    string input_csv = "../convex/Data/G1LoggingData.csv";
    string output_path = "../convex/Data";
    string sequence_name = "g1_walking";
    
    // Parse command line arguments
    if (argc >= 2) {
        input_csv = argv[1];
    }
    if (argc >= 3) {
        output_path = argv[2];
    }
    if (argc >= 4) {
        sequence_name = argv[3];
    }

    
    // Parse the input CSV file
    cout << "Parsing CSV file..." << endl;
    DataStruct data = ParseCSV(input_csv);
    
    if (data.num_rows == 0) {
        cerr << "Error: No data loaded from CSV file" << endl;
        return 1;
    }
    
    cout << "Successfully parsed " << data.num_rows << " rows of data" << endl;
    cout << endl;
    
    // Generate output CSV files
    cout << "Generating output files..." << endl;
    writeIMUData(data, output_path);
    writeMetadata(data, output_path, sequence_name);
    writeContactData(data, output_path);
    writeGroundTruthTrajectory(data, output_path);
    
    cout << endl;
    cout << "CSV organization complete!" << endl;
    cout << "Output files created in: " << output_path << endl;
    
    return 0;
}
