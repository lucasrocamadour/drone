#pragma once

#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct Vec3{
    double x;
    double y;
    double z;
};

struct Quaternion{
    double w;
    double x;
    double y;
    double z;
};

struct RigidBody{
    double mass;
    Vec3 size;
    Vec3 inertia_diag;
    Vec3 inertia_diag_inv;

    Vec3 position_world;
    Vec3 velocity_world;

    Quaternion attitude_world_from_body;
    Vec3 angular_velocity_body;
};

struct Rotor{
    Vec3 position_body;
    Vec3 thrust_axis_body;
    int spin_dir;

    double omega;
    double omega_cmd;

    double k_thrust;
    double k_torque;
};

class Drone{
    public:
        Drone();
        void step(double dt);
        void command_rotors(double omega_cmd_1, double omega_cmd_2, double omega_cmd_3, double omega_cmd_4);
        Vec3 calc_thrust();
        Vec3 calc_total_torque();

        RigidBody body;

        Rotor rotor1;
        Rotor rotor2;
        Rotor rotor3;
        Rotor rotor4;
};

class gyro{
    public:
        gyro();
        Vec3 sense(Drone& D1);
    private:
        Vec3 sensed_angular_velocity_body;
        Vec3 gyro_bias_body = {0.003, -0.002, 0.0015};
};

class accelerometer{
    public:
        accelerometer();
        Vec3 sense(Drone& D1);
    private:
        Vec3 accel_body;
        Vec3 accel_bias_body = {0.03, -0.02, 0.015};
        double sigma_accel = 0.006;
};

class barometer{
    public:
        barometer();
        double sense(Drone& D1);
    private:
        double baro_altitude;
        double baro_bias = 0.15;
        double sigma_baro = 0.11;
};


struct gpsdata{
    Vec3 sensed_position;
    Vec3 sensed_velocity;
    double horizontal_accuracy = 1.5;
    double vertical_accuracy = 2.5;
    double speed_accuracy = 0.05;
    bool fresh;
};

class gps{
    public:
        gps();
        gpsdata sense(Drone& D1, double dt);
    private:
        double t;
        double next_gps_time;
        double gps_update_period;
        Vec3 sensed_position;
        Vec3 sensed_velocity;
        gpsdata data;
        bool fresh;
        Vec3 position_bias;
        Vec3 velocity_bias;
        double horizontal_bias_sigma = 1.10;
        double vertical_bias_sigma = 1.80;
        double horizontal_jitter_sigma = 0.25;
        double vertical_jitter_sigma = 0.50;
        double velocity_bias_sigma = 0.005;
        double velocity_jitter_sigma = 0.020;
};


class magnometer{
    public:
        magnometer();
        Vec3 sense(Drone& D1);
    private:
        Vec3 mag_field_world;
        Vec3 mag_bias_body = {0.004, -0.003, 0.002};
        double sigma_mag = 0.0008;
};

struct pos_vel{
    Vec3 est_pos;
    Vec3 est_vel;
    Vec3 accel_bias_body;
    double baro_bias;
    double covariance[10][10];
    bool initialized;
};

struct propellerCMD{
    double omega1_cmd;
    double omega2_cmd;
    double omega3_cmd;
    double omega4_cmd;
};


Vec3 linear_drag(const Vec3& velocity_world);
Vec3 v_world(Quaternion& q, Vec3& v_body);
Vec3 v_world_transposed(const Quaternion& q, const Vec3& v_world);
Quaternion update_quat(Quaternion& q, Vec3 omega_body, double dt);

pos_vel estimate_pos_vel(pos_vel previous, gpsdata gps_data, Vec3 sensed_accel, double baro_altitude, Quaternion estimated_attitude, double dt);
void print_and_write(Drone& D1, gpsdata gps_data, gpsdata filtered_gps_data, double gps_xy_trust, double gps_time_since_fresh, Vec3 sensed_angular_vel, Vec3 filtered_sensed_angular_vel, Vec3 sensed_accel, Vec3 filtered_sensed_accel, double baro_altitude, double filtered_baro_altitude, Quaternion estimated_attitude, pos_vel estimated_pos_vel, const char* telemetry_path, double seconds);
void run_sim_batch(Drone& D1, gyro& G1, accelerometer& A1, barometer& B1, gps&GPS1, magnometer&M1, const char* telemetry_path, const char* conditions_path);
