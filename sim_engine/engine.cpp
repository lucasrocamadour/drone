#include "engine.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

using namespace std;

namespace {
const double dt = 0.01;
const Vec3 gravity = {0.0, 0.0, -9.81};
Vec3 estimated_gyro_bias = {0.0, 0.0, 0.0};
double yaw_spin_trim_integral = 0.0;
double position_hold_ix = 0.0;
double position_hold_iy = 0.0;
double position_hold_iz = 0.0;
int position_hold_x_confirm = 0;
int position_hold_y_confirm = 0;
int position_hold_z_confirm = 0;
bool position_hold_x_active = false;
bool position_hold_y_active = false;
bool position_hold_z_active = false;
double hold_inside_time = 0.0;
double hold_outside_time = 0.0;
double hold_overshoot_time = 0.0;
bool nav_baro_home_initialized = false;
double nav_baro_home = 0.0;
double nav_baro_home_sum = 0.0;
int nav_baro_home_count = 0;
bool nav_gps_home_initialized = false;
Vec3 nav_gps_home = {0.0, 0.0, 0.0};
Vec3 nav_gps_home_sum = {0.0, 0.0, 0.0};
int nav_gps_home_count = 0;
int nav_gps_x_innovation_sign = 0;
int nav_gps_y_innovation_sign = 0;
int nav_gps_x_innovation_confirm = 0;
int nav_gps_y_innovation_confirm = 0;
Vec3 trajectory_velocity_integral = {0.0, 0.0, 0.0};
Vec3 trajectory_accel_command = {0.0, 0.0, 0.0};
Vec3 stationary_hold_position = {0.0, 0.0, 0.0};
bool stationary_hold_position_captured = false;
}

Drone::Drone(){
    double width;
    double depth;
    double height;
    double Jx;
    double Jy;
    double Jz;
    double mass = 1.0;
    double arm_length = 0.20;

    width = 0.2;
    depth = 0.2;
    height = 0.05;
    Jx = (mass / 12.0) * ((depth * depth) + (height * height));
    Jy = (mass / 12.0) * ((width * width) + (height * height));
    Jz = (mass / 12.0) * ((width * width) + (depth * depth));
    body.mass = mass;
    body.size = {width, depth, height};
    body.inertia_diag = {Jx, Jy, Jz};
    body.inertia_diag_inv = {1.0 / Jx, 1.0 / Jy, 1.0 / Jz};
    body.position_world = {0.0, 0.0, 0.0};
    body.velocity_world = {0.0, 0.0, 0.0};
    body.attitude_world_from_body = {1.0, 0.0, 0.0, 0.0};
    body.angular_velocity_body = {0.0, 0.0, 0.0};

    double a = arm_length / sqrt(2.0);
    rotor1.position_body = {a, a, 0.0};
    rotor2.position_body = {-a, a, 0.0};
    rotor3.position_body = {-a, -a, 0.0};
    rotor4.position_body = {a, -a, 0.0};

    rotor1.thrust_axis_body = {0.0, 0.0, 1.0};
    rotor2.thrust_axis_body = {0.0, 0.0, 1.0};
    rotor3.thrust_axis_body = {0.0, 0.0, 1.0};
    rotor4.thrust_axis_body = {0.0, 0.0, 1.0};

    rotor1.spin_dir = 1;
    rotor2.spin_dir = -1;
    rotor3.spin_dir = 1;
    rotor4.spin_dir = -1;

    rotor1.omega = 0.0;
    rotor2.omega = 0.0;
    rotor3.omega = 0.0;
    rotor4.omega = 0.0;

    rotor1.omega_cmd = 0.0;
    rotor2.omega_cmd = 0.0;
    rotor3.omega_cmd = 0.0;
    rotor4.omega_cmd = 0.0;

    rotor1.k_thrust = 1.02e-5 * 0.99;
    rotor2.k_thrust = 1.02e-5 * 1.01;
    rotor3.k_thrust = 1.02e-5 * 1.003;
    rotor4.k_thrust = 1.02e-5 * 0.9993;

    rotor1.k_torque = 1e-6 * 1.09;
    rotor2.k_torque = 1e-6 * 0.9;
    rotor3.k_torque = 1e-6 * 1.02;
    rotor4.k_torque = 1e-6 * 1.01;
}

// ==================
// Sensor Helpers
// ==================

double noise(double sigma){
      static std::mt19937 rng(7);
      static std::normal_distribution<double>dist(0.0, 1.0);
      return sigma * dist(rng);
}


Vec3 v_world_transposed(const Quaternion& q, const Vec3& v_world_input){
    Vec3 out;

    double R11 = 1 - 2 * (q.y * q.y + q.z * q.z);
    double R12 = 2 * (q.x * q.y - q.z * q.w);
    double R13 = 2 * (q.x * q.z + q.y * q.w);
    double R21 = 2 * (q.x * q.y + q.z * q.w);
    double R22 = 1 - 2 * (q.x * q.x + q.z * q.z);
    double R23 = 2 * (q.y * q.z - q.x * q.w);
    double R31 = 2 * (q.x * q.z - q.y * q.w);
    double R32 = 2 * (q.y * q.z + q.x * q.w);
    double R33 = 1 - 2 * (q.x * q.x + q.y * q.y);

    out.x = R11 * v_world_input.x + R21 * v_world_input.y + R31 * v_world_input.z;
    out.y = R12 * v_world_input.x + R22 * v_world_input.y + R32 * v_world_input.z;
    out.z = R13 * v_world_input.x + R23 * v_world_input.y + R33 * v_world_input.z;

    return out;
}


// ==================
// Sensor Helpers
// ==================

// ==================
// Sensors
// ==================

gyro::gyro(){
    sensed_angular_velocity_body = {0.0, 0.0, 0.0};
}

Vec3 gyro::sense(Drone& D1){

    double sigma_gyro = 4.8e-4;
    Vec3 gyro_noise = {
        noise(sigma_gyro),
        noise(sigma_gyro),
        noise(sigma_gyro)
    };
    sensed_angular_velocity_body = {
        D1.body.angular_velocity_body.x + gyro_bias_body.x + gyro_noise.x,
        D1.body.angular_velocity_body.y + gyro_bias_body.y + gyro_noise.y,
        D1.body.angular_velocity_body.z + gyro_bias_body.z + gyro_noise.z
    };

    return sensed_angular_velocity_body;
}

barometer::barometer(){
    baro_altitude = 0.0;
}

double barometer::sense(Drone& D1){
    baro_altitude = D1.body.position_world.z + baro_bias + noise(sigma_baro);
    return baro_altitude;
}

accelerometer::accelerometer(){
    accel_body = {0.0, 0.0, 0.0};
}

Vec3 accelerometer::sense(Drone& D1){
    Vec3 total_force_body = D1.calc_thrust();
    Vec3 total_force_world = v_world(D1.body.attitude_world_from_body, total_force_body);
    Vec3 F_drag = linear_drag(D1.body.velocity_world);
    Vec3 specific_force_world = {
        (total_force_world.x + F_drag.x) / D1.body.mass,
        (total_force_world.y + F_drag.y) / D1.body.mass,
        (total_force_world.z + F_drag.z) / D1.body.mass
    };
    Quaternion sensed_quaternion = D1.body.attitude_world_from_body;
    accel_body = v_world_transposed(sensed_quaternion, specific_force_world);
    Vec3 accel_noise = {
        noise(sigma_accel),
        noise(sigma_accel),
        noise(sigma_accel)
    };
    accel_body.x += accel_bias_body.x + accel_noise.x;
    accel_body.y += accel_bias_body.y + accel_noise.y;
    accel_body.z += accel_bias_body.z + accel_noise.z;
    return accel_body;
}

gps::gps(){
    t = 0;
    next_gps_time = 0.1;
    gps_update_period = 0.1;
    data.sensed_position = {0,0,0};
    data.sensed_velocity = {0,0,0};

    data.fresh = false;
    sensed_position = {0,0,0};
    sensed_velocity = {0,0,0};
    position_bias = {0,0,0};
    velocity_bias = {0,0,0};
    fresh = false;
}

gpsdata gps::sense(Drone&D1, double dt){
    t += dt;
    if(t>=next_gps_time){
    next_gps_time += gps_update_period;

    auto update_bias = [](double current, double sigma, double correlation_time){
        double correlation = std::exp(-0.1 / correlation_time);
        return correlation * current +
            std::sqrt(1.0 - correlation * correlation) * noise(sigma);
    };

    position_bias.x = update_bias(position_bias.x, horizontal_bias_sigma, 45.0);
    position_bias.y = update_bias(position_bias.y, horizontal_bias_sigma, 45.0);
    position_bias.z = update_bias(position_bias.z, vertical_bias_sigma, 45.0);
    velocity_bias.x = update_bias(velocity_bias.x, velocity_bias_sigma, 8.0);
    velocity_bias.y = update_bias(velocity_bias.y, velocity_bias_sigma, 8.0);
    velocity_bias.z = update_bias(velocity_bias.z, velocity_bias_sigma, 8.0);

    data.sensed_position = {
        D1.body.position_world.x + position_bias.x + noise(horizontal_jitter_sigma),
        D1.body.position_world.y + position_bias.y + noise(horizontal_jitter_sigma),
        D1.body.position_world.z + position_bias.z + noise(vertical_jitter_sigma)
        };

    data.sensed_velocity = {
        D1.body.velocity_world.x + velocity_bias.x + noise(velocity_jitter_sigma),
        D1.body.velocity_world.y + velocity_bias.y + noise(velocity_jitter_sigma),
        D1.body.velocity_world.z + velocity_bias.z + noise(velocity_jitter_sigma)
        };
    data.fresh = true;
    } else{
        data.fresh = false;
    }
    return data;
}

magnometer::magnometer(){
    // Approximate mid-latitude Earth field in gauss, with a strong downward dip.
    mag_field_world = {0.22, 0.0, -0.43};
}

Vec3 magnometer::sense(Drone&D1){
    Vec3 mag_body = v_world_transposed(D1.body.attitude_world_from_body, mag_field_world);
    mag_body.x += mag_bias_body.x + noise(sigma_mag);
    mag_body.y += mag_bias_body.y + noise(sigma_mag);
    mag_body.z += mag_bias_body.z + noise(sigma_mag);

    return mag_body;
}

// ==================
//  Sensors End
// ==================

// ==================
//  Physics
// ==================

Quaternion update_quat(Quaternion& q, Vec3 omega_body, double dt_local){
    Quaternion p = {0.0, omega_body.x, omega_body.y, omega_body.z};
    Quaternion prod = {
        (q.w * p.w) - (q.x * p.x) - (q.y * p.y) - (q.z * p.z),
        (q.w * p.x) + (q.x * p.w) + (q.y * p.z) - (q.z * p.y),
        (q.w * p.y) - (q.x * p.z) + (q.y * p.w) + (q.z * p.x),
        (q.w * p.z) + (q.x * p.y) - (q.y * p.x) + (q.z * p.w),
    };
    Quaternion q_dot = {prod.w * 0.5, prod.x * 0.5, prod.y * 0.5, prod.z * 0.5};
    Quaternion n_quaternion = {
        q.w + (q_dot.w * dt_local),
        q.x + (q_dot.x * dt_local),
        q.y + (q_dot.y * dt_local),
        q.z + (q_dot.z * dt_local)
    };
    double magnitude = sqrt(
        n_quaternion.w * n_quaternion.w +
        n_quaternion.x * n_quaternion.x +
        n_quaternion.y * n_quaternion.y +
        n_quaternion.z * n_quaternion.z
    );
    n_quaternion = {
        n_quaternion.w / magnitude,
        n_quaternion.x / magnitude,
        n_quaternion.y / magnitude,
        n_quaternion.z / magnitude
    };
    return n_quaternion;
}

Vec3 v_world(Quaternion& q, Vec3& v_body){
    Vec3 v_world_result;
    double R11 = 1 - 2 * (q.y * q.y + q.z * q.z);
    double R12 = 2 * (q.x * q.y - q.z * q.w);
    double R13 = 2 * (q.x * q.z + q.y * q.w);
    double R21 = 2 * (q.x * q.y + q.z * q.w);
    double R22 = 1 - 2 * (q.x * q.x + q.z * q.z);
    double R23 = 2 * (q.y * q.z - q.x * q.w);
    double R31 = 2 * (q.x * q.z - q.y * q.w);
    double R32 = 2 * (q.y * q.z + q.x * q.w);
    double R33 = 1 - 2 * (q.x * q.x + q.y * q.y);

    v_world_result.x = R11 * v_body.x + R12 * v_body.y + R13 * v_body.z;
    v_world_result.y = R21 * v_body.x + R22 * v_body.y + R23 * v_body.z;
    v_world_result.z = R31 * v_body.x + R32 * v_body.y + R33 * v_body.z;

    return v_world_result;
}

void match_omega(Rotor& rotor, double dt){
    double motor_tau = 0.03;
    rotor.omega += (dt/motor_tau)* (rotor.omega_cmd-rotor.omega);

}

Vec3 damping_torque(const Vec3& angular_velocity_body){
    double k_ang = 0.02;
    return {
        -k_ang * angular_velocity_body.x,
        -k_ang * angular_velocity_body.y,
        -k_ang * angular_velocity_body.z
    };
}

Vec3 linear_drag(const Vec3& velocity_world){
    double k_lin = 0.1;
    return {
        -k_lin * velocity_world.x,
        -k_lin * velocity_world.y,
        -k_lin * velocity_world.z
    };
}

void Drone::step(double dt_local){
    match_omega(rotor1, dt_local);
    match_omega(rotor2, dt_local);
    match_omega(rotor3, dt_local);
    match_omega(rotor4, dt_local);
    Vec3 total_force_body = calc_thrust();
    Vec3 total_force_world = v_world(body.attitude_world_from_body, total_force_body);
    Vec3 F_drag = linear_drag(body.velocity_world);
    total_force_world = {
        total_force_world.x + F_drag.x,
        total_force_world.y + F_drag.y,
        total_force_world.z + F_drag.z
    };
    Vec3 acceleration = {
        gravity.x + (total_force_world.x / body.mass),
        gravity.y + (total_force_world.y / body.mass),
        gravity.z + (total_force_world.z / body.mass)
    };
    Vec3 total_torque = calc_total_torque();
    Vec3 tau_damping = damping_torque(body.angular_velocity_body);
    total_torque = {
        total_torque.x + tau_damping.x,
        total_torque.y + tau_damping.y,
        total_torque.z + tau_damping.z
    };
    Vec3 w = body.angular_velocity_body;
    Vec3 Jw = {
        body.inertia_diag.x * w.x,
        body.inertia_diag.y * w.y,
        body.inertia_diag.z * w.z
    };
    Vec3 coupling = {
        w.y * Jw.z - w.z * Jw.y,
        w.z * Jw.x - w.x * Jw.z,
        w.x * Jw.y - w.y * Jw.x
    };
    Vec3 tau_eff = {
        total_torque.x - coupling.x,
        total_torque.y - coupling.y,
        total_torque.z - coupling.z
    };

    Vec3 alpha = {
        tau_eff.x * body.inertia_diag_inv.x,
        tau_eff.y * body.inertia_diag_inv.y,
        tau_eff.z * body.inertia_diag_inv.z
    };

    body.velocity_world = {
        body.velocity_world.x + acceleration.x * dt_local,
        body.velocity_world.y + acceleration.y * dt_local,
        body.velocity_world.z + acceleration.z * dt_local
    };

    body.angular_velocity_body = {
        body.angular_velocity_body.x + alpha.x * dt_local,
        body.angular_velocity_body.y + alpha.y * dt_local,
        body.angular_velocity_body.z + alpha.z * dt_local
    };

    body.attitude_world_from_body = update_quat(
        body.attitude_world_from_body,
        body.angular_velocity_body,
        dt_local
    );
    body.position_world = {
        body.position_world.x + body.velocity_world.x * dt_local,
        body.position_world.y + body.velocity_world.y * dt_local,
        body.position_world.z + body.velocity_world.z * dt_local
    };
}

void Drone::command_rotors(double omega_cmd_1, double omega_cmd_2, double omega_cmd_3, double omega_cmd_4){
    rotor1.omega_cmd = omega_cmd_1;
    rotor2.omega_cmd = omega_cmd_2;
    rotor3.omega_cmd = omega_cmd_3;
    rotor4.omega_cmd = omega_cmd_4;
}

double thrust_rotor(Rotor& rotor){
    return rotor.k_thrust * rotor.omega * rotor.omega;
}

Vec3 reaction_torque_rotor(Rotor& rotor){
    double Q_i = rotor.k_torque * (rotor.omega * rotor.omega);
    double yaw_torque_i = rotor.spin_dir * Q_i;
    return {0.0, 0.0, yaw_torque_i};
}

Vec3 force_rotor(Rotor& rotor, double T_i){
    return {
        rotor.thrust_axis_body.x * T_i,
        rotor.thrust_axis_body.y * T_i,
        rotor.thrust_axis_body.z * T_i
    };
}

Vec3 Drone::calc_thrust(){
    double T1 = thrust_rotor(rotor1);
    double T2 = thrust_rotor(rotor2);
    double T3 = thrust_rotor(rotor3);
    double T4 = thrust_rotor(rotor4);

    double T = T1 + T2 + T3 + T4;
    return {0.0, 0.0, T};
}

Vec3 Drone::calc_total_torque(){
    double T1 = thrust_rotor(rotor1);
    double T2 = thrust_rotor(rotor2);
    double T3 = thrust_rotor(rotor3);
    double T4 = thrust_rotor(rotor4);
    Vec3 F1 = force_rotor(rotor1, T1);
    Vec3 F2 = force_rotor(rotor2, T2);
    Vec3 F3 = force_rotor(rotor3, T3);
    Vec3 F4 = force_rotor(rotor4, T4);
    Vec3 r1 = reaction_torque_rotor(rotor1);
    Vec3 r2 = reaction_torque_rotor(rotor2);
    Vec3 r3 = reaction_torque_rotor(rotor3);
    Vec3 r4 = reaction_torque_rotor(rotor4);
    Vec3 tau_1 = {
        (rotor1.position_body.y * F1.z) - (rotor1.position_body.z * F1.y),
        (rotor1.position_body.z * F1.x) - (rotor1.position_body.x * F1.z),
        (rotor1.position_body.x * F1.y) - (rotor1.position_body.y * F1.x),
    };
    Vec3 tau_2 = {
        (rotor2.position_body.y * F2.z) - (rotor2.position_body.z * F2.y),
        (rotor2.position_body.z * F2.x) - (rotor2.position_body.x * F2.z),
        (rotor2.position_body.x * F2.y) - (rotor2.position_body.y * F2.x),
    };
    Vec3 tau_3 = {
        (rotor3.position_body.y * F3.z) - (rotor3.position_body.z * F3.y),
        (rotor3.position_body.z * F3.x) - (rotor3.position_body.x * F3.z),
        (rotor3.position_body.x * F3.y) - (rotor3.position_body.y * F3.x),
    };
    Vec3 tau_4 = {
        (rotor4.position_body.y * F4.z) - (rotor4.position_body.z * F4.y),
        (rotor4.position_body.z * F4.x) - (rotor4.position_body.x * F4.z),
        (rotor4.position_body.x * F4.y) - (rotor4.position_body.y * F4.x),
    };
    tau_1 = {tau_1.x + r1.x, tau_1.y + r1.y, tau_1.z + r1.z};
    tau_2 = {tau_2.x + r2.x, tau_2.y + r2.y, tau_2.z + r2.z};
    tau_3 = {tau_3.x + r3.x, tau_3.y + r3.y, tau_3.z + r3.z};
    tau_4 = {tau_4.x + r4.x, tau_4.y + r4.y, tau_4.z + r4.z};

    return {
        tau_1.x + tau_2.x + tau_3.x + tau_4.x,
        tau_1.y + tau_2.y + tau_3.y + tau_4.y,
        tau_1.z + tau_2.z + tau_3.z + tau_4.z,
    };
}


Quaternion estimate_attitude(Vec3 sensed_angular_vel, Quaternion&prev_quat, Vec3 sensed_accel, Vec3 sensed_mag, double dt){
    auto normalize_vec = [](Vec3 value){
        double magnitude = sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        if(magnitude < 1e-9){
            return Vec3{0.0, 0.0, 0.0};
        }
        return Vec3{value.x / magnitude, value.y / magnitude, value.z / magnitude};
    };

    auto cross = [](const Vec3& a, const Vec3& b){
        return Vec3{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    };

    auto dot = [](const Vec3& a, const Vec3& b){
        return a.x * b.x + a.y * b.y + a.z * b.z;
    };

    auto normalize_quat = [](Quaternion value){
        double magnitude = sqrt(
            value.w * value.w +
            value.x * value.x +
            value.y * value.y +
            value.z * value.z
        );
        if(magnitude < 1e-9){
            return Quaternion{1.0, 0.0, 0.0, 0.0};
        }
        return Quaternion{
            value.w / magnitude,
            value.x / magnitude,
            value.y / magnitude,
            value.z / magnitude
        };
    };

    Quaternion q_est = normalize_quat(prev_quat);

    const Vec3 gravity_world = {0.0, 0.0, 1.0};
    const Vec3 magnetic_world_raw = {0.22, 0.0, -0.43};
    const Vec3 magnetic_world = normalize_vec(magnetic_world_raw);

    Vec3 accel_body = normalize_vec(sensed_accel);
    Vec3 mag_body = normalize_vec(sensed_mag);
    Vec3 gravity_pred_body = v_world_transposed(q_est, gravity_world);
    Vec3 mag_pred_body = v_world_transposed(q_est, magnetic_world);

    double accel_magnitude = sqrt(
        sensed_accel.x * sensed_accel.x +
        sensed_accel.y * sensed_accel.y +
        sensed_accel.z * sensed_accel.z
    );
    double accel_magnitude_trust = 1.0 - abs(accel_magnitude - 9.81) / 1.5;
    if(accel_magnitude_trust < 0.0){
        accel_magnitude_trust = 0.0;
    } else if(accel_magnitude_trust > 1.0){
        accel_magnitude_trust = 1.0;
    }

    double accel_alignment = dot(accel_body, gravity_pred_body);
    double accel_direction_trust = (accel_alignment - 0.90) / 0.10;
    if(accel_direction_trust < 0.0){
        accel_direction_trust = 0.0;
    } else if(accel_direction_trust > 1.0){
        accel_direction_trust = 1.0;
    }
    double accel_trust = accel_magnitude_trust * accel_direction_trust;

    Vec3 accel_error = cross(accel_body, gravity_pred_body);

    Vec3 mag_body_horizontal = {
        mag_body.x - dot(mag_body, gravity_pred_body) * gravity_pred_body.x,
        mag_body.y - dot(mag_body, gravity_pred_body) * gravity_pred_body.y,
        mag_body.z - dot(mag_body, gravity_pred_body) * gravity_pred_body.z
    };
    Vec3 mag_pred_horizontal = {
        mag_pred_body.x - dot(mag_pred_body, gravity_pred_body) * gravity_pred_body.x,
        mag_pred_body.y - dot(mag_pred_body, gravity_pred_body) * gravity_pred_body.y,
        mag_pred_body.z - dot(mag_pred_body, gravity_pred_body) * gravity_pred_body.z
    };
    mag_body_horizontal = normalize_vec(mag_body_horizontal);
    mag_pred_horizontal = normalize_vec(mag_pred_horizontal);
    Vec3 mag_error = cross(mag_body_horizontal, mag_pred_horizontal);

    double mag_magnitude = sqrt(
        sensed_mag.x * sensed_mag.x +
        sensed_mag.y * sensed_mag.y +
        sensed_mag.z * sensed_mag.z
    );
    double expected_mag_magnitude = sqrt(
        magnetic_world_raw.x * magnetic_world_raw.x +
        magnetic_world_raw.y * magnetic_world_raw.y +
        magnetic_world_raw.z * magnetic_world_raw.z
    );
    double mag_trust = 1.0 - abs(mag_magnitude - expected_mag_magnitude) / 0.03;
    if(mag_trust < 0.0){
        mag_trust = 0.0;
    } else if(mag_trust > 1.0){
        mag_trust = 1.0;
    }

    const double kp_accel = 0.20;
    const double kp_mag = 0.25;
    const double ki_accel = 0.03;
    const double ki_mag = 0.005;
    estimated_gyro_bias.x -= (
        ki_accel * accel_trust * accel_error.x +
        ki_mag * mag_trust * mag_error.x
    ) * dt;
    estimated_gyro_bias.y -= (
        ki_accel * accel_trust * accel_error.y +
        ki_mag * mag_trust * mag_error.y
    ) * dt;
    estimated_gyro_bias.z -= (
        ki_accel * accel_trust * accel_error.z +
        ki_mag * mag_trust * mag_error.z
    ) * dt;
    estimated_gyro_bias.x = clamp(estimated_gyro_bias.x, -0.05, 0.05);
    estimated_gyro_bias.y = clamp(estimated_gyro_bias.y, -0.05, 0.05);
    estimated_gyro_bias.z = clamp(estimated_gyro_bias.z, -0.05, 0.05);

    Vec3 corrected_rates = {
        sensed_angular_vel.x - estimated_gyro_bias.x + kp_accel * accel_trust * accel_error.x + kp_mag * mag_trust * mag_error.x,
        sensed_angular_vel.y - estimated_gyro_bias.y + kp_accel * accel_trust * accel_error.y + kp_mag * mag_trust * mag_error.y,
        sensed_angular_vel.z - estimated_gyro_bias.z + kp_accel * accel_trust * accel_error.z + kp_mag * mag_trust * mag_error.z
    };

    Quaternion omega_quat = {0.0, corrected_rates.x, corrected_rates.y, corrected_rates.z};
    Quaternion q_dot = {
        0.5 * (-q_est.x * omega_quat.x - q_est.y * omega_quat.y - q_est.z * omega_quat.z),
        0.5 * ( q_est.w * omega_quat.x + q_est.y * omega_quat.z - q_est.z * omega_quat.y),
        0.5 * ( q_est.w * omega_quat.y - q_est.x * omega_quat.z + q_est.z * omega_quat.x),
        0.5 * ( q_est.w * omega_quat.z + q_est.x * omega_quat.y - q_est.y * omega_quat.x)
    };

    q_est = {
        q_est.w + q_dot.w * dt,
        q_est.x + q_dot.x * dt,
        q_est.y + q_dot.y * dt,
        q_est.z + q_dot.z * dt
    };

    return normalize_quat(q_est);
}

#if 0
pos_vel estimate_pos_vel_legacy(pos_vel previous, gpsdata gps_data, Vec3 sensed_accel, double baro_altitude, Quaternion estimated_attitude, double dt){
    constexpr int state_count = 10;
    constexpr int pos_offset = 0;
    constexpr int vel_offset = 3;
    constexpr int accel_bias_offset = 6;
    constexpr int baro_bias_index = 9;

    if(!previous.initialized){
        previous.est_pos = {0.0, 0.0, 0.0};
        previous.est_vel = {0.0, 0.0, 0.0};
        previous.accel_bias_body = {0.0, 0.0, 0.0};
        previous.baro_bias = 0.0;

        for(int row = 0; row < state_count; row++){
            for(int col = 0; col < state_count; col++){
                previous.covariance[row][col] = 0.0;
            }
        }
        previous.covariance[0][0] = 1.0;
        previous.covariance[1][1] = 1.0;
        previous.covariance[2][2] = 0.15;
        previous.covariance[3][3] = 0.25;
        previous.covariance[4][4] = 0.25;
        previous.covariance[5][5] = 0.15;
        previous.covariance[6][6] = 0.05;
        previous.covariance[7][7] = 0.05;
        previous.covariance[8][8] = 0.05;
        previous.covariance[9][9] = 0.25;
        previous.initialized = true;
    }

    double state[state_count] = {
        previous.est_pos.x,
        previous.est_pos.y,
        previous.est_pos.z,
        previous.est_vel.x,
        previous.est_vel.y,
        previous.est_vel.z,
        previous.accel_bias_body.x,
        previous.accel_bias_body.y,
        previous.accel_bias_body.z,
        previous.baro_bias
    };

    Vec3 body_x = {1.0, 0.0, 0.0};
    Vec3 body_y = {0.0, 1.0, 0.0};
    Vec3 body_z = {0.0, 0.0, 1.0};
    Vec3 world_x = v_world(estimated_attitude, body_x);
    Vec3 world_y = v_world(estimated_attitude, body_y);
    Vec3 world_z = v_world(estimated_attitude, body_z);
    double rotation[3][3] = {
        {world_x.x, world_y.x, world_z.x},
        {world_x.y, world_y.y, world_z.y},
        {world_x.z, world_y.z, world_z.z}
    };

    Vec3 corrected_accel_body = {
        sensed_accel.x - previous.accel_bias_body.x,
        sensed_accel.y - previous.accel_bias_body.y,
        sensed_accel.z - previous.accel_bias_body.z
    };
    Vec3 accel_world = v_world(estimated_attitude, corrected_accel_body);
    accel_world.z -= 9.81;

    double accelerations[3] = {accel_world.x, accel_world.y, accel_world.z};
    double dt2 = dt * dt;

    for(int axis = 0; axis < 3; axis++){
        state[pos_offset + axis] += state[vel_offset + axis] * dt + 0.5 * accelerations[axis] * dt2;
        state[vel_offset + axis] += accelerations[axis] * dt;
    }

    double transition[state_count][state_count] = {};
    for(int index = 0; index < state_count; index++){
        transition[index][index] = 1.0;
    }
    for(int axis = 0; axis < 3; axis++){
        transition[pos_offset + axis][vel_offset + axis] = dt;
        for(int body_axis = 0; body_axis < 3; body_axis++){
            transition[pos_offset + axis][accel_bias_offset + body_axis] =
                -0.5 * rotation[axis][body_axis] * dt2;
            transition[vel_offset + axis][accel_bias_offset + body_axis] =
                -rotation[axis][body_axis] * dt;
        }
    }

    double predicted_covariance[state_count][state_count] = {};
    for(int row = 0; row < state_count; row++){
        for(int col = 0; col < state_count; col++){
            for(int k = 0; k < state_count; k++){
                for(int m = 0; m < state_count; m++){
                    predicted_covariance[row][col] +=
                        transition[row][k] * previous.covariance[k][m] * transition[col][m];
                }
            }
        }
    }

    const double accel_noise_variance = 0.15;
    const double accel_bias_walk_variance = 0.00001;
    const double baro_bias_walk_variance = 0.00002;
    for(int axis = 0; axis < 3; axis++){
        predicted_covariance[pos_offset + axis][pos_offset + axis] += 0.25 * dt2 * dt2 * accel_noise_variance;
        predicted_covariance[vel_offset + axis][vel_offset + axis] += dt2 * accel_noise_variance;
        predicted_covariance[accel_bias_offset + axis][accel_bias_offset + axis] += dt * accel_bias_walk_variance;
    }
    predicted_covariance[baro_bias_index][baro_bias_index] += dt * baro_bias_walk_variance;

    for(int row = 0; row < state_count; row++){
        for(int col = 0; col < state_count; col++){
            previous.covariance[row][col] = predicted_covariance[row][col];
        }
    }

    auto correct = [&](const double observation[state_count], double measurement, double measurement_variance){
        double predicted_measurement = 0.0;
        for(int index = 0; index < state_count; index++){
            predicted_measurement += observation[index] * state[index];
        }

        double innovation = measurement - predicted_measurement;
        double covariance_times_h[state_count] = {};
        double innovation_variance = measurement_variance;
        for(int row = 0; row < state_count; row++){
            for(int col = 0; col < state_count; col++){
                covariance_times_h[row] += previous.covariance[row][col] * observation[col];
            }
            innovation_variance += observation[row] * covariance_times_h[row];
        }
        if(innovation_variance <= 1e-9 || innovation * innovation > 25.0 * innovation_variance){
            return;
        }

        double gain[state_count] = {};
        for(int index = 0; index < state_count; index++){
            gain[index] = covariance_times_h[index] / innovation_variance;
            state[index] += gain[index] * innovation;
        }

        double identity_minus_kh[state_count][state_count] = {};
        for(int row = 0; row < state_count; row++){
            for(int col = 0; col < state_count; col++){
                identity_minus_kh[row][col] = (row == col ? 1.0 : 0.0) - gain[row] * observation[col];
            }
        }

        double joseph[state_count][state_count] = {};
        for(int row = 0; row < state_count; row++){
            for(int col = 0; col < state_count; col++){
                for(int k = 0; k < state_count; k++){
                    for(int m = 0; m < state_count; m++){
                        joseph[row][col] +=
                            identity_minus_kh[row][k] * previous.covariance[k][m] * identity_minus_kh[col][m];
                    }
                }
                joseph[row][col] += gain[row] * measurement_variance * gain[col];
            }
        }
        for(int row = 0; row < state_count; row++){
            for(int col = 0; col < state_count; col++){
                previous.covariance[row][col] = joseph[row][col];
            }
        }
    };

    auto gps_measurement_is_sane = [&](){
        double pos_error_x = gps_data.sensed_position.x - state[pos_offset + 0];
        double pos_error_y = gps_data.sensed_position.y - state[pos_offset + 1];
        double vel_error_x = gps_data.sensed_velocity.x - state[vel_offset + 0];
        double vel_error_y = gps_data.sensed_velocity.y - state[vel_offset + 1];

        double xy_pos_error = std::sqrt(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
        double xy_vel_error = std::sqrt(vel_error_x * vel_error_x + vel_error_y * vel_error_y);

        double xy_pos_gate = std::max(1.0, 3.0 * gps_data.horizontal_accuracy);
        double xy_vel_gate = std::max(0.60, 4.0 * gps_data.speed_accuracy);

        return xy_pos_error <= xy_pos_gate && xy_vel_error <= xy_vel_gate;
    };

    if(gps_data.fresh && gps_measurement_is_sane()){
        double horizontal_variance = gps_data.horizontal_accuracy * gps_data.horizontal_accuracy;
        double speed_variance = gps_data.speed_accuracy * gps_data.speed_accuracy;

        for(int axis = 0; axis < 2; axis++){
            double position_observation[state_count] = {};
            position_observation[pos_offset + axis] = 1.0;
            correct(position_observation, axis == 0 ? gps_data.sensed_position.x : gps_data.sensed_position.y, horizontal_variance);

            double velocity_observation[state_count] = {};
            velocity_observation[vel_offset + axis] = 1.0;
            correct(velocity_observation, axis == 0 ? gps_data.sensed_velocity.x : gps_data.sensed_velocity.y, speed_variance);
        }

        auto apply_persistent_position_evidence = [](
            double measurement,
            double& estimated_position,
            int& previous_sign,
            int& confirm_count
        ){
            constexpr double evidence_threshold = 0.25;
            constexpr int required_confirmations = 5;
            constexpr double correction_gain = 0.02;
            constexpr double max_correction = 0.015;

            double innovation = measurement - estimated_position;
            if(std::abs(innovation) < evidence_threshold){
                previous_sign = 0;
                confirm_count = 0;
                return;
            }

            int innovation_sign = innovation > 0.0 ? 1 : -1;
            if(innovation_sign == previous_sign){
                confirm_count += 1;
            } else {
                previous_sign = innovation_sign;
                confirm_count = 1;
            }

            if(confirm_count >= required_confirmations){
                estimated_position += clamp(
                    correction_gain * innovation,
                    -max_correction,
                    max_correction
                );
            }
        };

        apply_persistent_position_evidence(
            gps_data.sensed_position.x,
            state[pos_offset + 0],
            nav_gps_x_innovation_sign,
            nav_gps_x_innovation_confirm
        );
        apply_persistent_position_evidence(
            gps_data.sensed_position.y,
            state[pos_offset + 1],
            nav_gps_y_innovation_sign,
            nav_gps_y_innovation_confirm
        );
    }

    double accel_magnitude = std::sqrt(
        sensed_accel.x * sensed_accel.x +
        sensed_accel.y * sensed_accel.y +
        sensed_accel.z * sensed_accel.z
    );
    double horizontal_state_speed = std::sqrt(
        state[vel_offset + 0] * state[vel_offset + 0] +
        state[vel_offset + 1] * state[vel_offset + 1]
    );
    double horizontal_gps_speed = std::sqrt(
        gps_data.sensed_velocity.x * gps_data.sensed_velocity.x +
        gps_data.sensed_velocity.y * gps_data.sensed_velocity.y
    );
    bool likely_hovering =
        gps_data.fresh &&
        std::abs(accel_magnitude - 9.81) < 0.20 &&
        horizontal_state_speed < 0.25 &&
        horizontal_gps_speed < 0.12;
    if(likely_hovering){
        double zero_velocity_variance = 0.12;
        for(int axis = 0; axis < 2; axis++){
            double velocity_observation[state_count] = {};
            velocity_observation[vel_offset + axis] = 1.0;
            correct(velocity_observation, 0.0, zero_velocity_variance);
        }
    }

    double baro_observation[state_count] = {};
    baro_observation[pos_offset + 2] = 1.0;
    baro_observation[baro_bias_index] = 1.0;
    correct(baro_observation, baro_altitude, 0.08);

    state[6] = clamp(state[6], -0.08, 0.08);
    state[7] = clamp(state[7], -0.08, 0.08);
    state[8] = clamp(state[8], -0.08, 0.08);
    state[9] = clamp(state[9], -0.50, 0.50);

    previous.est_pos = {state[0], state[1], state[2]};
    previous.est_vel = {state[3], state[4], state[5]};
    previous.accel_bias_body = {state[6], state[7], state[8]};
    previous.baro_bias = state[9];
    return previous;
}
#endif

pos_vel estimate_pos_vel(
    pos_vel previous,
    gpsdata gps_data,
    Vec3 sensed_accel,
    double baro_altitude,
    Quaternion estimated_attitude,
    double dt
){
    if(!previous.initialized){
        previous.est_pos = {0.0, 0.0, 0.0};
        previous.est_vel = {0.0, 0.0, 0.0};
        previous.accel_bias_body = {0.0, 0.0, 0.0};
        previous.baro_bias = 0.0;
        previous.initialized = true;
    }

    Vec3 corrected_accel_body = {
        sensed_accel.x - previous.accel_bias_body.x,
        sensed_accel.y - previous.accel_bias_body.y,
        sensed_accel.z - previous.accel_bias_body.z
    };
    Vec3 accel_world = v_world(estimated_attitude, corrected_accel_body);
    accel_world.z -= 9.81;

    previous.est_vel.z += accel_world.z * dt;
    previous.est_pos.x += previous.est_vel.x * dt;
    previous.est_pos.y += previous.est_vel.y * dt;
    previous.est_pos.z += previous.est_vel.z * dt;

    if(gps_data.fresh){
        constexpr double horizontal_velocity_gain = 0.85;
        constexpr double vertical_velocity_gain = 0.35;
        constexpr double slow_position_gain = 0.0005;
        constexpr double max_horizontal_position_innovation = 2.0;

        previous.est_vel.x += horizontal_velocity_gain *
            (gps_data.sensed_velocity.x - previous.est_vel.x);
        previous.est_vel.y += horizontal_velocity_gain *
            (gps_data.sensed_velocity.y - previous.est_vel.y);
        previous.est_vel.z += vertical_velocity_gain *
            (gps_data.sensed_velocity.z - previous.est_vel.z);

        double innovation_x = gps_data.sensed_position.x - previous.est_pos.x;
        double innovation_y = gps_data.sensed_position.y - previous.est_pos.y;
        if(std::abs(innovation_x) < max_horizontal_position_innovation){
            previous.est_pos.x += slow_position_gain * innovation_x;
        }
        if(std::abs(innovation_y) < max_horizontal_position_innovation){
            previous.est_pos.y += slow_position_gain * innovation_y;
        }
    }

    constexpr double barometer_position_gain = 0.08;
    previous.est_pos.z += barometer_position_gain *
        (baro_altitude - previous.est_pos.z);

    return previous;
}

void print_and_write(Drone& D1, gpsdata gps_data, gpsdata filtered_gps_data, double gps_xy_trust, double gps_time_since_fresh, Vec3 sensed_angular_vel, Vec3 filtered_sensed_angular_vel, Vec3 sensed_accel, Vec3 filtered_sensed_accel, double baro_altitude, double filtered_baro_altitude, Quaternion estimated_attitude, pos_vel estimated_pos_vel, const char* telemetry_path, double seconds);


// Filtered
Vec3 low_pass_vec3(Vec3 sensed, Vec3 previous_filtered, double dt, double tau){
    double alpha = dt / (tau + dt);
    Vec3 filtered = {
    previous_filtered.x + alpha * (sensed.x - previous_filtered.x),
    previous_filtered.y + alpha * (sensed.y - previous_filtered.y),
    previous_filtered.z + alpha * (sensed.z - previous_filtered.z)
        };
    return filtered;
}

Vec3 filter_vec3(Vec3 sensed, Vec3 previous_filtered, double dt){
    return low_pass_vec3(sensed, previous_filtered, dt, 0.03);
}
  gpsdata filter_gpsdata(gpsdata sensed, gpsdata previous_filtered, double /*dt*/){
      constexpr double gps_position_tau = 0.25;
      constexpr double gps_velocity_tau = 0.12;
      constexpr double gps_sample_period = 0.10;
      bool previous_uninitialized =
          !previous_filtered.fresh &&
          previous_filtered.sensed_position.x == 0.0 &&
          previous_filtered.sensed_position.y == 0.0 &&
          previous_filtered.sensed_position.z == 0.0 &&
          previous_filtered.sensed_velocity.x == 0.0 &&
          previous_filtered.sensed_velocity.y == 0.0 &&
          previous_filtered.sensed_velocity.z == 0.0;

      if(!sensed.fresh){
          previous_filtered.fresh = false;
          return previous_filtered;
      }

      if(previous_uninitialized){
          return sensed;
      }
      gpsdata filtered = previous_filtered;
      double position_alpha = gps_sample_period / (gps_position_tau + gps_sample_period);
      double velocity_alpha = gps_sample_period / (gps_velocity_tau + gps_sample_period);

      filtered.sensed_position = {
          previous_filtered.sensed_position.x + position_alpha * (sensed.sensed_position.x - previous_filtered.sensed_position.x),
          previous_filtered.sensed_position.y + position_alpha * (sensed.sensed_position.y - previous_filtered.sensed_position.y),
          sensed.sensed_position.z
      };
      filtered.sensed_velocity = {
          previous_filtered.sensed_velocity.x + velocity_alpha * (sensed.sensed_velocity.x - previous_filtered.sensed_velocity.x),
          previous_filtered.sensed_velocity.y + velocity_alpha * (sensed.sensed_velocity.y - previous_filtered.sensed_velocity.y),
          sensed.sensed_velocity.z
      };

      filtered.horizontal_accuracy = sensed.horizontal_accuracy * 0.40;
      filtered.vertical_accuracy = sensed.vertical_accuracy;
      filtered.speed_accuracy = sensed.speed_accuracy * 0.80;
      filtered.fresh = true;

      return filtered;
  }



double filter_baro_altitude(double sensed_baro_altitude, double prev_filtered, double dt){
    double tau = 0.03;
    double alpha = dt / (tau+dt);
    double filtered = prev_filtered + alpha*(sensed_baro_altitude-prev_filtered);
    return filtered;
}

// Filtered

// =============
// Hover & Helpers
// =============

propellerCMD collective_adjusted(
    const pos_vel& est,
    const Quaternion& q_est,
    const Vec3& gyro_body,
    double xy_trust,
    const Vec3& target_position,
    double target_yaw,
    double dt
){
    (void)est;
    (void)xy_trust;
    (void)target_position;
    (void)dt;

    constexpr double gravity_mag = 9.80665;
    constexpr double mass_kg = 1.0;
    constexpr double nominal_k_thrust = 1.02e-5;

    constexpr double max_thrust_per_motor = 12.0;
    constexpr double max_omega = 850.0;
    constexpr double min_omega = 250.0;

    auto wrap_pi = [](double angle){
        while(angle > M_PI){
            angle -= 2.0 * M_PI;
        }
        while(angle < -M_PI){
            angle += 2.0 * M_PI;
        }
        return angle;
    };
auto thrust_to_omega = [&](double thrust_N){
    thrust_N = clamp(thrust_N, 0.0, max_thrust_per_motor);
    return std::sqrt(thrust_N / nominal_k_thrust);
};

    /*
        Quaternion to Euler.
    */
    double sinr_cosp = 2.0 * (q_est.w * q_est.x + q_est.y * q_est.z);
    double cosr_cosp = 1.0 - 2.0 * (q_est.x * q_est.x + q_est.y * q_est.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q_est.w * q_est.y - q_est.z * q_est.x);
    sinp = clamp(sinp, -1.0, 1.0);
    double pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q_est.w * q_est.z + q_est.x * q_est.y);
    double cosy_cosp = 1.0 - 2.0 * (q_est.y * q_est.y + q_est.z * q_est.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    /*
        Collective hover thrust.
    */
    double vertical_alignment = std::cos(roll) * std::cos(pitch);
    vertical_alignment = clamp(vertical_alignment, 0.50, 1.0);

    double total_thrust_N = mass_kg * gravity_mag / vertical_alignment;
    double collective = total_thrust_N / 4.0;

    /*
        Stay upright.

        Target:
            roll  = 0
            pitch = 0
            yaw   = target_yaw
    */
    double roll_error = wrap_pi(0.0 - roll);
    double pitch_error = wrap_pi(0.0 - pitch);
    double yaw_error = wrap_pi(target_yaw - yaw);

    constexpr double kp_angle_roll = 2.5;
    constexpr double kp_angle_pitch = 2.5;
    constexpr double kp_angle_yaw = 0.8;

    double roll_rate_cmd = clamp(kp_angle_roll * roll_error, -1.5, 1.5);
    double pitch_rate_cmd = clamp(kp_angle_pitch * pitch_error, -1.5, 1.5);
    double yaw_rate_cmd = clamp(kp_angle_yaw * yaw_error, -0.6, 0.6);

    constexpr double kp_rate_roll = 0.16;
    constexpr double kp_rate_pitch = 0.16;
    constexpr double kp_rate_yaw = 0.045;

    double roll_delta = clamp(
        kp_rate_roll * (roll_rate_cmd - gyro_body.x),
        -0.50,
        0.50
    );

    double pitch_delta = clamp(
        kp_rate_pitch * (pitch_rate_cmd - gyro_body.y),
        -0.50,
        0.50
    );

    double yaw_delta = clamp(
        kp_rate_yaw * (yaw_rate_cmd - gyro_body.z),
        -0.18,
        0.18
    );

    /*
        Rotor layout:
            rotor1 = {+x, +y}
            rotor2 = {-x, +y}
            rotor3 = {-x, -y}
            rotor4 = {+x, -y}
    */
    double f1 = collective + roll_delta - pitch_delta + yaw_delta;
    double f2 = collective + roll_delta + pitch_delta - yaw_delta;
    double f3 = collective - roll_delta + pitch_delta + yaw_delta;
    double f4 = collective - roll_delta - pitch_delta - yaw_delta;

    f1 = clamp(f1, 0.0, max_thrust_per_motor);
    f2 = clamp(f2, 0.0, max_thrust_per_motor);
    f3 = clamp(f3, 0.0, max_thrust_per_motor);
    f4 = clamp(f4, 0.0, max_thrust_per_motor);

    double w1 = thrust_to_omega(f1);
    double w2 = thrust_to_omega(f2);
    double w3 = thrust_to_omega(f3);
    double w4 = thrust_to_omega(f4);

    w1 = clamp(w1, min_omega, max_omega);
    w2 = clamp(w2, min_omega, max_omega);
    w3 = clamp(w3, min_omega, max_omega);
    w4 = clamp(w4, min_omega, max_omega);

    return {w1, w2, w3, w4};
}

static propellerCMD counter_spin_commands(
    propellerCMD commands,
    const Vec3& gyro_body
){
    constexpr double yaw_rate_target = 0.0;
    constexpr double yaw_brake_gain_omega = 60.0;
    constexpr double yaw_brake_gain_integral = 18.0;
    constexpr double max_yaw_brake_omega = 120.0;
    constexpr double max_yaw_trim_integral = 2.5;
    constexpr double min_omega = 250.0;
    constexpr double max_omega = 850.0;

    double yaw_rate_error = yaw_rate_target - gyro_body.z;
    yaw_spin_trim_integral += yaw_rate_error * dt;
    yaw_spin_trim_integral = clamp(
        yaw_spin_trim_integral,
        -max_yaw_trim_integral,
        max_yaw_trim_integral
    );

    double yaw_brake = clamp(
        yaw_brake_gain_omega * yaw_rate_error +
        yaw_brake_gain_integral * yaw_spin_trim_integral,
        -max_yaw_brake_omega,
        max_yaw_brake_omega
    );

    // rotors 1 and 3 produce positive reaction torque, 2 and 4 negative.
    commands.omega1_cmd = clamp(commands.omega1_cmd + yaw_brake, min_omega, max_omega);
    commands.omega2_cmd = clamp(commands.omega2_cmd - yaw_brake, min_omega, max_omega);
    commands.omega3_cmd = clamp(commands.omega3_cmd + yaw_brake, min_omega, max_omega);
    commands.omega4_cmd = clamp(commands.omega4_cmd - yaw_brake, min_omega, max_omega);

    return commands;
}

static double position_axis_accel_command(
    double target_position,
    double estimated_position,
    double estimated_velocity,
    double& integral,
    int& confirm_counter,
    bool& correction_active,
    double dt_local,
    double engage_band,
    double release_band,
    int engage_samples,
    int release_samples,
    double max_accel,
    double max_speed,
    double velocity_gain,
    double integral_gain
){
    constexpr double near_target_speed = 0.12;
    constexpr double overshoot_brake_gain = 1.35;
    constexpr double moving_away_boost = 1.05;
    constexpr double actuator_response_time = 0.25;

    double error = target_position - estimated_position;
    double abs_error = std::abs(error);
    double abs_velocity = std::abs(estimated_velocity);

    if(!correction_active){
        if(abs_error > engage_band){
            confirm_counter += 1;
        } else {
            confirm_counter = 0;
            integral *= 0.98;
        }
        if(confirm_counter >= engage_samples){
            correction_active = true;
            confirm_counter = 0;
        }
    } else {
        if(abs_error < release_band && abs_velocity < near_target_speed){
            confirm_counter += 1;
        } else {
            confirm_counter = 0;
        }
        if(confirm_counter >= release_samples){
            correction_active = false;
            confirm_counter = 0;
            integral = 0.0;
            return 0.0;
        }
    }

    if(!correction_active){
        double micro_accel = clamp(
            -1.2 * estimated_velocity,
            -0.18 * max_accel,
            0.18 * max_accel
        );
        return micro_accel;
    }

    double direction_to_target = (error >= 0.0) ? 1.0 : -1.0;
    double velocity_direction = (estimated_velocity >= 0.0) ? 1.0 : -1.0;
    bool moving_toward_target = (error * estimated_velocity) > 0.0;
    bool moving_away_from_target = (error * estimated_velocity) < 0.0;
    double stopping_distance =
        (abs_velocity * abs_velocity) / std::max(2.0 * max_accel, 1e-6) +
        abs_velocity * actuator_response_time;

    if(abs_error < 2.0){
        integral += error * dt_local;
    }
    integral = clamp(integral, -0.8, 0.8);

    if(std::abs(target_position - estimated_position) < engage_band){
        double accel = -velocity_gain * 2.2 * estimated_velocity + 1.2 * error;
        return clamp(accel, -max_accel, max_accel);
    }

    if(moving_toward_target && stopping_distance >= abs_error){
        double accel = -velocity_direction * max_accel * overshoot_brake_gain;
        return clamp(accel, -max_accel, max_accel);
    }

    double no_overshoot_speed = std::sqrt(std::max(0.0, 2.0 * max_accel * abs_error));
    double desired_speed = direction_to_target * std::min(max_speed, no_overshoot_speed);

    if(moving_away_from_target && abs_error > release_band){
        desired_speed = direction_to_target * max_speed * moving_away_boost;
    }

    if(abs_error < 0.35){
        desired_speed = clamp(desired_speed, -near_target_speed, near_target_speed);
    }

    double accel = velocity_gain * (desired_speed - estimated_velocity) + integral_gain * integral;
    return clamp(accel, -max_accel, max_accel);
}

[[maybe_unused]] static propellerCMD maintain_position_commands(
    propellerCMD commands,
    const pos_vel& est,
    const Quaternion& q_est,
    const Vec3& gyro_body,
    const Vec3& target_position,
    double xy_trust,
    double dt_local
){
    constexpr double gravity_mag = 9.80665;
    constexpr double min_omega = 250.0;
    constexpr double max_omega = 850.0;

    constexpr double lookahead_xy = 0.30;
    constexpr double lookahead_z = 0.22;

    constexpr double max_xy_accel = 1.65;
    constexpr double max_tilt = 12.0 * M_PI / 180.0;

    constexpr double kp_tilt = 180.0;
    constexpr double kd_tilt_rate = 35.0;
    constexpr double max_tilt_trim = 95.0;

    constexpr double max_collective_trim = 130.0;

    double sinr_cosp = 2.0 * (q_est.w * q_est.x + q_est.y * q_est.z);
    double cosr_cosp = 1.0 - 2.0 * (q_est.x * q_est.x + q_est.y * q_est.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q_est.w * q_est.y - q_est.z * q_est.x);
    sinp = clamp(sinp, -1.0, 1.0);
    double pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q_est.w * q_est.z + q_est.x * q_est.y);
    double cosy_cosp = 1.0 - 2.0 * (q_est.y * q_est.y + q_est.z * q_est.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double predicted_x = est.est_pos.x + est.est_vel.x * lookahead_xy;
    double predicted_y = est.est_pos.y + est.est_vel.y * lookahead_xy;
    double predicted_z = est.est_pos.z + est.est_vel.z * lookahead_z;

    double xy_authority = 0.35 + 0.65 * clamp((xy_trust - 0.15) / 0.55, 0.0, 1.0);

    double accel_cmd_x = position_axis_accel_command(
        target_position.x,
        predicted_x,
        est.est_vel.x,
        position_hold_ix,
        position_hold_x_confirm,
        position_hold_x_active,
        dt_local,
        std::max(0.10, std::abs(target_position.x) * 0.10),
        std::max(0.06, std::abs(target_position.x) * 0.06),
        18,
        24,
        max_xy_accel * xy_authority,
        0.25,
        2.8,
        0.06
    );
    double accel_cmd_y = position_axis_accel_command(
        target_position.y,
        predicted_y,
        est.est_vel.y,
        position_hold_iy,
        position_hold_y_confirm,
        position_hold_y_active,
        dt_local,
        std::max(0.10, std::abs(target_position.y) * 0.10),
        std::max(0.06, std::abs(target_position.y) * 0.06),
        18,
        24,
        max_xy_accel * xy_authority,
        0.25,
        2.8,
        0.06
    );

    double desired_roll = clamp(
        -(accel_cmd_y * std::cos(yaw) - accel_cmd_x * std::sin(yaw)) / gravity_mag,
        -max_tilt,
        max_tilt
    );
    double desired_pitch = clamp(
        (accel_cmd_x * std::cos(yaw) + accel_cmd_y * std::sin(yaw)) / gravity_mag,
        -max_tilt,
        max_tilt
    );

    double roll_trim = clamp(
        kp_tilt * (desired_roll - roll) - kd_tilt_rate * gyro_body.x,
        -max_tilt_trim,
        max_tilt_trim
    );
    double pitch_trim = clamp(
        kp_tilt * (desired_pitch - pitch) - kd_tilt_rate * gyro_body.y,
        -max_tilt_trim,
        max_tilt_trim
    );

    double accel_cmd_z = position_axis_accel_command(
        target_position.z,
        predicted_z,
        est.est_vel.z,
        position_hold_iz,
        position_hold_z_confirm,
        position_hold_z_active,
        dt_local,
        std::max(0.10, std::abs(target_position.z) * 0.10),
        std::max(0.06, std::abs(target_position.z) * 0.06),
        12,
        18,
        1.6,
        0.25,
        2.8,
        0.10
    );

    double collective_trim = clamp(accel_cmd_z * 42.0, -max_collective_trim, max_collective_trim);

    commands.omega1_cmd += collective_trim + roll_trim - pitch_trim;
    commands.omega2_cmd += collective_trim + roll_trim + pitch_trim;
    commands.omega3_cmd += collective_trim - roll_trim + pitch_trim;
    commands.omega4_cmd += collective_trim - roll_trim - pitch_trim;

    commands.omega1_cmd = clamp(commands.omega1_cmd, min_omega, max_omega);
    commands.omega2_cmd = clamp(commands.omega2_cmd, min_omega, max_omega);
    commands.omega3_cmd = clamp(commands.omega3_cmd, min_omega, max_omega);
    commands.omega4_cmd = clamp(commands.omega4_cmd, min_omega, max_omega);

    return commands;
}

static propellerCMD climb_one_meter_and_stop(
    propellerCMD commands,
    const Quaternion& q_est,
    const Vec3& gyro_body,
    double target_altitude,
    const Vec3& estimated_position,
    const Vec3& estimated_velocity,
    double dt_local
){
    constexpr double fast_approach_band = 2.0;
    constexpr double max_fast_climb_speed = 2.5;
    constexpr double max_fast_vertical_accel = 3.2;
    constexpr double fast_velocity_gain = 3.6;
    constexpr double fast_velocity_integral_gain = 0.28;
    constexpr double max_fast_velocity_integral = 1.4;
    constexpr double max_climb_speed = 0.35;
    constexpr double max_vertical_accel = 1.5;
    constexpr double braking_accel = 1.8;
    constexpr double response_delay = 0.20;
    constexpr double position_gain = 1.5;
    constexpr double velocity_gain = 3.2;
    constexpr double velocity_integral_gain = 0.12;
    constexpr double max_velocity_integral = 0.5;
    constexpr double max_jerk = 12.0;
    constexpr double altitude_hold_band = 0.03;
    constexpr double stopped_velocity = 0.05;
    constexpr double max_collective_trim = 130.0;
    constexpr double gravity_mag = 9.80665;
    constexpr double horizontal_velocity_gain = 3.0;
    constexpr double horizontal_position_gain = 0.8;
    constexpr double horizontal_velocity_integral_gain = 0.40;
    constexpr double max_horizontal_velocity_integral = 0.8;
    constexpr double max_horizontal_accel = 1.5;
    constexpr double max_tilt = 8.0 * M_PI / 180.0;
    constexpr double kp_tilt = 180.0;
    constexpr double kd_tilt_rate = 35.0;
    constexpr double max_tilt_trim = 95.0;
    constexpr double min_omega = 250.0;
    constexpr double max_omega = 850.0;

    double estimated_altitude = estimated_position.z;
    double estimated_vertical_velocity = estimated_velocity.z;
    double altitude_error = target_altitude - estimated_altitude;
    double abs_altitude_error = std::abs(altitude_error);
    double direction = altitude_error >= 0.0 ? 1.0 : -1.0;
    bool fast_approach = abs_altitude_error > fast_approach_band;
    double desired_vertical_velocity;
    if(fast_approach){
        desired_vertical_velocity = direction * max_fast_climb_speed;
    } else {
        double safe_speed = std::sqrt(
            std::max(0.0, 2.0 * braking_accel * abs_altitude_error)
        );
        desired_vertical_velocity = direction * std::min(
            max_climb_speed,
            std::min(safe_speed, position_gain * abs_altitude_error)
        );
    }

    double velocity_direction = estimated_vertical_velocity >= 0.0 ? 1.0 : -1.0;
    double predicted_stop_altitude =
        estimated_altitude +
        estimated_vertical_velocity * response_delay +
        velocity_direction * estimated_vertical_velocity * estimated_vertical_velocity /
            (2.0 * braking_accel);
    bool predicted_to_cross_target =
        altitude_error * (target_altitude - predicted_stop_altitude) <= 0.0 &&
        std::abs(estimated_vertical_velocity) > stopped_velocity;
    if(!fast_approach && predicted_to_cross_target){
        desired_vertical_velocity = 0.0;
    }

    bool holding =
        std::abs(altitude_error) < altitude_hold_band &&
        std::abs(estimated_vertical_velocity) < stopped_velocity;
    if(holding && !stationary_hold_position_captured){
        stationary_hold_position = estimated_position;
        stationary_hold_position_captured = true;
        trajectory_velocity_integral.x = 0.0;
        trajectory_velocity_integral.y = 0.0;
    }
    if(holding){
        desired_vertical_velocity = 0.0;
        trajectory_velocity_integral.z *= 0.98;
    } else {
        double max_vertical_velocity_integral = fast_approach
            ? max_fast_velocity_integral
            : max_velocity_integral;
        trajectory_velocity_integral.z = clamp(
            trajectory_velocity_integral.z +
                (desired_vertical_velocity - estimated_vertical_velocity) * dt_local,
            -max_vertical_velocity_integral,
            max_vertical_velocity_integral
        );
    }

    double vertical_kp = fast_approach ? fast_velocity_gain : velocity_gain;
    double vertical_ki = fast_approach ? fast_velocity_integral_gain : velocity_integral_gain;
    double vertical_accel_limit = fast_approach ? max_fast_vertical_accel : max_vertical_accel;

    double desired_vertical_accel = clamp(
        vertical_kp * (desired_vertical_velocity - estimated_vertical_velocity) +
            vertical_ki * trajectory_velocity_integral.z,
        -vertical_accel_limit,
        vertical_accel_limit
    );
    trajectory_accel_command.z += clamp(
        desired_vertical_accel - trajectory_accel_command.z,
        -max_jerk * dt_local,
        max_jerk * dt_local
    );
    double collective_trim = clamp(
        trajectory_accel_command.z * 42.0,
        -max_collective_trim,
        max_collective_trim
    );

    double roll = std::atan2(
        2.0 * (q_est.w * q_est.x + q_est.y * q_est.z),
        1.0 - 2.0 * (q_est.x * q_est.x + q_est.y * q_est.y)
    );
    double pitch_sine = clamp(
        2.0 * (q_est.w * q_est.y - q_est.z * q_est.x),
        -1.0,
        1.0
    );
    double pitch = std::asin(pitch_sine);
    double yaw = std::atan2(
        2.0 * (q_est.w * q_est.z + q_est.x * q_est.y),
        1.0 - 2.0 * (q_est.y * q_est.y + q_est.z * q_est.z)
    );
    trajectory_velocity_integral.x = clamp(
        trajectory_velocity_integral.x - estimated_velocity.x * dt_local,
        -max_horizontal_velocity_integral,
        max_horizontal_velocity_integral
    );
    trajectory_velocity_integral.y = clamp(
        trajectory_velocity_integral.y - estimated_velocity.y * dt_local,
        -max_horizontal_velocity_integral,
        max_horizontal_velocity_integral
    );
    double position_error_x = stationary_hold_position_captured
        ? stationary_hold_position.x - estimated_position.x
        : 0.0;
    double position_error_y = stationary_hold_position_captured
        ? stationary_hold_position.y - estimated_position.y
        : 0.0;
    double accel_x = clamp(
        horizontal_position_gain * position_error_x -
            horizontal_velocity_gain * estimated_velocity.x +
            horizontal_velocity_integral_gain * trajectory_velocity_integral.x,
        -max_horizontal_accel,
        max_horizontal_accel
    );
    double accel_y = clamp(
        horizontal_position_gain * position_error_y -
            horizontal_velocity_gain * estimated_velocity.y +
            horizontal_velocity_integral_gain * trajectory_velocity_integral.y,
        -max_horizontal_accel,
        max_horizontal_accel
    );
    double desired_roll = clamp(
        -(accel_y * std::cos(yaw) - accel_x * std::sin(yaw)) / gravity_mag,
        -max_tilt,
        max_tilt
    );
    double desired_pitch = clamp(
        (accel_x * std::cos(yaw) + accel_y * std::sin(yaw)) / gravity_mag,
        -max_tilt,
        max_tilt
    );
    double roll_trim = clamp(
        kp_tilt * (desired_roll - roll) - kd_tilt_rate * gyro_body.x,
        -max_tilt_trim,
        max_tilt_trim
    );
    double pitch_trim = clamp(
        kp_tilt * (desired_pitch - pitch) - kd_tilt_rate * gyro_body.y,
        -max_tilt_trim,
        max_tilt_trim
    );

    commands.omega1_cmd += collective_trim + roll_trim - pitch_trim;
    commands.omega2_cmd += collective_trim + roll_trim + pitch_trim;
    commands.omega3_cmd += collective_trim - roll_trim + pitch_trim;
    commands.omega4_cmd += collective_trim - roll_trim - pitch_trim;

    commands.omega1_cmd = clamp(commands.omega1_cmd, min_omega, max_omega);
    commands.omega2_cmd = clamp(commands.omega2_cmd, min_omega, max_omega);
    commands.omega3_cmd = clamp(commands.omega3_cmd, min_omega, max_omega);
    commands.omega4_cmd = clamp(commands.omega4_cmd, min_omega, max_omega);
    return commands;
}

propellerCMD hover(
        const pos_vel& est,
        const Quaternion& q_est,
        const Vec3& gyro_body,
        double xy_trust,
        const Vec3& target_position,
        double target_yaw,
        const Vec3& gps_velocity,
        double dt
        ){
    (void)gps_velocity;
    propellerCMD commands = collective_adjusted(
        est,
        q_est,
        gyro_body,
        xy_trust,
        target_position,
        target_yaw,
        dt
    );
    // Previous sensor-fused predictive target controller, retained for comparison.
#if 0
    commands = maintain_position_commands(
        commands,
        est,
        q_est,
        gyro_body,
        target_position,
        xy_trust,
        dt
    );
#endif
    commands = climb_one_meter_and_stop(
        commands,
        q_est,
        gyro_body,
        target_position.z,
        est.est_pos,
        est.est_vel,
        dt
    );
    commands = counter_spin_commands(commands, gyro_body);
    return commands;
}

void run_sim_batch(Drone& D1, gyro& G1, accelerometer& A1, barometer& B1,  gps&GPS1, magnometer&M1, const char* telemetry_path, const char* conditions_path){
    ifstream conditions_file(conditions_path);
    json data = json::parse(conditions_file);

    constexpr double runtime = 60.0;

    estimated_gyro_bias = {0.0, 0.0, 0.0};
    yaw_spin_trim_integral = 0.0;
    position_hold_ix = 0.0;
    position_hold_iy = 0.0;
    position_hold_iz = 0.0;
    position_hold_x_confirm = 0;
    position_hold_y_confirm = 0;
    position_hold_z_confirm = 0;
    position_hold_x_active = false;
    position_hold_y_active = false;
    position_hold_z_active = false;
    hold_inside_time = 0.0;
    hold_outside_time = 0.0;
    hold_overshoot_time = 0.0;
    nav_baro_home_initialized = false;
    nav_baro_home = 0.0;
    nav_baro_home_sum = 0.0;
    nav_baro_home_count = 0;
    nav_gps_home_initialized = false;
    nav_gps_home = {0.0, 0.0, 0.0};
    nav_gps_home_sum = {0.0, 0.0, 0.0};
    nav_gps_home_count = 0;
    nav_gps_x_innovation_sign = 0;
    nav_gps_y_innovation_sign = 0;
    nav_gps_x_innovation_confirm = 0;
    nav_gps_y_innovation_confirm = 0;
    trajectory_velocity_integral = {0.0, 0.0, 0.0};
    trajectory_accel_command = {0.0, 0.0, 0.0};
    stationary_hold_position = {0.0, 0.0, 0.0};
    stationary_hold_position_captured = false;


    Vec3 sensed_angular_vel = {0.0, 0.0, 0.0};
    Vec3 filtered_sensed_angular_vel = {0,0,0};
    Vec3 prev_filtered_sensed_angular_vel = {0,0,0};
    Vec3 estimated_angular_vel = {0.0, 0.0, 0.0};
    bool gyro_filter_initialized = false;

    Vec3 sensed_accel = {0.0, 0.0, 0.0};
    Vec3 filtered_sensed_accel = {0.0, 0.0, 0.0};
    Vec3 prev_filtered_sensed_accel = {0.0, 0.0, 0.0};
    bool accel_filter_initialized = false;

    gpsdata gps_data{};
    gpsdata gps_data_filtered{};
    gpsdata prev_gps_data_filtered{};
    Quaternion estimated_attitude;
    Quaternion prev_quat = {1,0,0,0};
    Vec3 sensed_mag = {0,0,0};
    pos_vel est_pos_vel;
    Vec3 attitude_accel;
    est_pos_vel.est_pos = {0,0,0};
    est_pos_vel.est_vel = {0,0,0};
    est_pos_vel.initialized = false;

    double baro_altitude = 0.0;
    double filtered_baro_altitude = 0.0;
    double prev_filtered_baro_altitude = 0.0;
    bool baro_filter_initialized = false;
    double gps_time_since_fresh = 1e9;
    double gps_xy_trust = 0.15;


    double seconds = 0.0;
    propellerCMD commands{0.0, 0.0, 0.0, 0.0};
    Vec3 target_position = {0.0, 0.0, data.value("target_1_z", 10.0)};
    double target_yaw = data.value("target_yaw", 0.0);

    constexpr int preflight_baro_samples = 100;
    for(int sample = 0; sample < preflight_baro_samples; sample++){
        nav_baro_home_sum += B1.sense(D1);
    }
    nav_baro_home_count = preflight_baro_samples;
    nav_baro_home = nav_baro_home_sum / static_cast<double>(nav_baro_home_count);
    nav_baro_home_initialized = true;

    constexpr int preflight_gps_samples = 600;
    for(int sample = 0; sample < preflight_gps_samples; sample++){
        gpsdata home_sample = GPS1.sense(D1, 0.10);
        nav_gps_home_sum.x += home_sample.sensed_position.x;
        nav_gps_home_sum.y += home_sample.sensed_position.y;
        nav_gps_home_sum.z += home_sample.sensed_position.z;
    }
    nav_gps_home_count = preflight_gps_samples;
    nav_gps_home = {
        nav_gps_home_sum.x / static_cast<double>(nav_gps_home_count),
        nav_gps_home_sum.y / static_cast<double>(nav_gps_home_count),
        nav_gps_home_sum.z / static_cast<double>(nav_gps_home_count)
    };
    nav_gps_home_initialized = true;

    commands = hover(
        est_pos_vel,
        prev_quat,
        sensed_angular_vel,
        gps_xy_trust,
        target_position,
        target_yaw,
        {0.0, 0.0, 0.0},
        dt
    );

    while(seconds < runtime){
        seconds += dt;

        D1.command_rotors(commands.omega1_cmd, commands.omega2_cmd, commands.omega3_cmd, commands.omega4_cmd);
        D1.step(dt);
        sensed_angular_vel = G1.sense(D1);
        if(!gyro_filter_initialized){
            filtered_sensed_angular_vel = sensed_angular_vel;
            gyro_filter_initialized = true;
        } else {
            filtered_sensed_angular_vel = low_pass_vec3(sensed_angular_vel, prev_filtered_sensed_angular_vel, dt, 0.005);
        }
        prev_filtered_sensed_angular_vel = filtered_sensed_angular_vel;

        sensed_accel = A1.sense(D1);
        if(!accel_filter_initialized){
            filtered_sensed_accel = sensed_accel;
            accel_filter_initialized = true;
        } else {
            filtered_sensed_accel = low_pass_vec3(sensed_accel, prev_filtered_sensed_accel, dt, 0.02);
        }
        prev_filtered_sensed_accel = filtered_sensed_accel;

        sensed_mag = M1.sense(D1);

        baro_altitude = B1.sense(D1);
        if(!baro_filter_initialized){
            filtered_baro_altitude = baro_altitude;
            baro_filter_initialized = true;
        } else {
            filtered_baro_altitude = filter_baro_altitude(baro_altitude, prev_filtered_baro_altitude, dt);
        }
        prev_filtered_baro_altitude = filtered_baro_altitude;
        if(!nav_baro_home_initialized){
            nav_baro_home_sum += filtered_baro_altitude;
            nav_baro_home_count += 1;
            if(nav_baro_home_count >= 20){
                nav_baro_home = nav_baro_home_sum / static_cast<double>(nav_baro_home_count);
                nav_baro_home_initialized = true;
            }
        }
        double local_baro_altitude = nav_baro_home_initialized ? (filtered_baro_altitude - nav_baro_home) : 0.0;

        gps_data = GPS1.sense(D1, dt);
        gps_data_filtered = filter_gpsdata(gps_data, prev_gps_data_filtered, dt);
        prev_gps_data_filtered = gps_data_filtered;
        if(gps_data_filtered.fresh && !nav_gps_home_initialized){
            nav_gps_home_sum.x += gps_data_filtered.sensed_position.x;
            nav_gps_home_sum.y += gps_data_filtered.sensed_position.y;
            nav_gps_home_sum.z += gps_data_filtered.sensed_position.z;
            nav_gps_home_count += 1;
            if(nav_gps_home_count >= 15){
                nav_gps_home = {
                    nav_gps_home_sum.x / static_cast<double>(nav_gps_home_count),
                    nav_gps_home_sum.y / static_cast<double>(nav_gps_home_count),
                    nav_gps_home_sum.z / static_cast<double>(nav_gps_home_count)
                };
                nav_gps_home_initialized = true;
            }
        }
        if(nav_gps_home_initialized){
            gps_data.sensed_position = {
                gps_data.sensed_position.x - nav_gps_home.x,
                gps_data.sensed_position.y - nav_gps_home.y,
                gps_data.sensed_position.z - nav_gps_home.z
            };
            gps_data_filtered.sensed_position = {
                gps_data_filtered.sensed_position.x - nav_gps_home.x,
                gps_data_filtered.sensed_position.y - nav_gps_home.y,
                gps_data_filtered.sensed_position.z - nav_gps_home.z
            };
        } else {
            gps_data_filtered.fresh = false;
        }
        if(gps_data.fresh){
            gps_time_since_fresh = 0.0;
        } else {
            gps_time_since_fresh += dt;
        }

        attitude_accel = {
            filtered_sensed_accel.x - est_pos_vel.accel_bias_body.x,
            filtered_sensed_accel.y - est_pos_vel.accel_bias_body.y,
            filtered_sensed_accel.z - est_pos_vel.accel_bias_body.z
        };
        estimated_attitude = estimate_attitude(filtered_sensed_angular_vel, prev_quat, attitude_accel, sensed_mag, dt);
        prev_quat = estimated_attitude;
        estimated_angular_vel = {
            filtered_sensed_angular_vel.x - estimated_gyro_bias.x,
            filtered_sensed_angular_vel.y - estimated_gyro_bias.y,
            filtered_sensed_angular_vel.z - estimated_gyro_bias.z
        };
        est_pos_vel = estimate_pos_vel(est_pos_vel, gps_data_filtered, filtered_sensed_accel, local_baro_altitude, estimated_attitude, dt);

        double filtered_gps_xy_error = std::sqrt(
            (gps_data_filtered.sensed_position.x - est_pos_vel.est_pos.x) * (gps_data_filtered.sensed_position.x - est_pos_vel.est_pos.x) +
            (gps_data_filtered.sensed_position.y - est_pos_vel.est_pos.y) * (gps_data_filtered.sensed_position.y - est_pos_vel.est_pos.y)
        );
        double filtered_gps_xy_velocity_error = std::sqrt(
            (gps_data_filtered.sensed_velocity.x - est_pos_vel.est_vel.x) * (gps_data_filtered.sensed_velocity.x - est_pos_vel.est_vel.x) +
            (gps_data_filtered.sensed_velocity.y - est_pos_vel.est_vel.y) * (gps_data_filtered.sensed_velocity.y - est_pos_vel.est_vel.y)
        );
        double freshness_trust = 1.0 - gps_time_since_fresh / 0.35;
        freshness_trust = clamp(freshness_trust, 0.0, 1.0);
        double position_consistency_trust = 1.0 - filtered_gps_xy_error / 1.2;
        position_consistency_trust = clamp(position_consistency_trust, 0.0, 1.0);
        double velocity_consistency_trust = 1.0 - filtered_gps_xy_velocity_error / 0.6;
        velocity_consistency_trust = clamp(velocity_consistency_trust, 0.0, 1.0);
        gps_xy_trust = std::max(0.15, freshness_trust * position_consistency_trust * velocity_consistency_trust);

        double target_dx = D1.body.position_world.x - target_position.x;
        double target_dy = D1.body.position_world.y - target_position.y;
        double target_dz = D1.body.position_world.z - target_position.z;
        double hold_band_x = std::max(0.10, std::abs(target_position.x) * 0.10);
        double hold_band_y = std::max(0.10, std::abs(target_position.y) * 0.10);
        double hold_band_z = std::max(0.10, std::abs(target_position.z) * 0.10);
        bool inside_hold_envelope =
            std::abs(target_dx) <= hold_band_x &&
            std::abs(target_dy) <= hold_band_y &&
            std::abs(target_dz) <= hold_band_z;
        if(inside_hold_envelope){
            hold_inside_time += dt;
        } else {
            hold_outside_time += dt;
            double overshoot_amount =
                std::max(0.0, std::abs(target_dx) - hold_band_x) +
                std::max(0.0, std::abs(target_dy) - hold_band_y) +
                std::max(0.0, std::abs(target_dz) - hold_band_z);
            hold_overshoot_time += overshoot_amount * dt;
        }


        commands = hover(
            est_pos_vel,
            estimated_attitude,
            estimated_angular_vel,
            gps_xy_trust,
            target_position,
            target_yaw,
            gps_data_filtered.sensed_velocity,
            dt
        );
        print_and_write(D1, gps_data, gps_data_filtered, gps_xy_trust, gps_time_since_fresh, sensed_angular_vel, filtered_sensed_angular_vel, sensed_accel, filtered_sensed_accel, baro_altitude, filtered_baro_altitude, estimated_attitude, est_pos_vel, telemetry_path, seconds);
    }
}




void print_and_write(Drone& D1, gpsdata gps_data, gpsdata filtered_gps_data, double gps_xy_trust, double gps_time_since_fresh, Vec3 sensed_angular_vel, Vec3 filtered_sensed_angular_vel, Vec3 sensed_accel, Vec3 filtered_sensed_accel, double baro_altitude, double filtered_baro_altitude, Quaternion estimated_attitude, pos_vel estimated_pos_vel, const char* telemetry_path, double seconds){
    ofstream telemetry_file(telemetry_path, ios::app);

    cout << "Position: " << "(" << D1.body.position_world.x << "," << D1.body.position_world.y << "," << D1.body.position_world.z << ")" << endl;
    cout << "Angular velocity: " << "(" << D1.body.angular_velocity_body.x << "," << D1.body.angular_velocity_body.y << "," << D1.body.angular_velocity_body.z << ")" << endl;
    cout << "Quaternion: " << "(" << D1.body.attitude_world_from_body.w << "," << D1.body.attitude_world_from_body.x << "," << D1.body.attitude_world_from_body.y << "," << D1.body.attitude_world_from_body.z << ")" << endl;
    cout << "Time: " << seconds << "s" << endl;

    telemetry_file << "{";
    telemetry_file << "\"time\": " << seconds << ", ";

    telemetry_file << "\"true_pos_x\": " << D1.body.position_world.x << ",";
    telemetry_file << "\"true_pos_y\": " << D1.body.position_world.y << ",";
    telemetry_file << "\"true_pos_z\": " << D1.body.position_world.z << ",";

    telemetry_file << "\"est_pos_x\": " << estimated_pos_vel.est_pos.x << ",";
    telemetry_file << "\"est_pos_y\": " << estimated_pos_vel.est_pos.y << ",";
    telemetry_file << "\"est_pos_z\": " << estimated_pos_vel.est_pos.z << ",";
    telemetry_file << "\"stationary_hold_captured\": " << (stationary_hold_position_captured ? 1 : 0) << ",";
    telemetry_file << "\"stationary_hold_x\": " << stationary_hold_position.x << ",";
    telemetry_file << "\"stationary_hold_y\": " << stationary_hold_position.y << ",";

    telemetry_file << "\"vel_x\": " << D1.body.velocity_world.x << ",";
    telemetry_file << "\"vel_y\": " << D1.body.velocity_world.y << ",";
    telemetry_file << "\"vel_z\": " << D1.body.velocity_world.z << ",";

    telemetry_file << "\"est_vel_x\": " << estimated_pos_vel.est_vel.x << ",";
    telemetry_file << "\"est_vel_y\": " << estimated_pos_vel.est_vel.y << ",";
    telemetry_file << "\"est_vel_z\": " << estimated_pos_vel.est_vel.z << ",";

    telemetry_file << "\"est_accel_bias_x\": " << estimated_pos_vel.accel_bias_body.x << ",";
    telemetry_file << "\"est_accel_bias_y\": " << estimated_pos_vel.accel_bias_body.y << ",";
    telemetry_file << "\"est_accel_bias_z\": " << estimated_pos_vel.accel_bias_body.z << ",";
    telemetry_file << "\"est_baro_bias\": " << estimated_pos_vel.baro_bias << ",";

    telemetry_file << "\"true_quat_w\": " << D1.body.attitude_world_from_body.w << ",";
    telemetry_file << "\"true_quat_x\": " << D1.body.attitude_world_from_body.x << ",";
    telemetry_file << "\"true_quat_y\": " << D1.body.attitude_world_from_body.y << ",";
    telemetry_file << "\"true_quat_z\": " << D1.body.attitude_world_from_body.z << ",";

    telemetry_file << "\"est_quat_w\": " << estimated_attitude.w << ",";
    telemetry_file << "\"est_quat_x\": " << estimated_attitude.x << ",";
    telemetry_file << "\"est_quat_y\": " << estimated_attitude.y << ",";
    telemetry_file << "\"est_quat_z\": " << estimated_attitude.z << ",";

    telemetry_file << "\"omega_x\": " << D1.body.angular_velocity_body.x << ",";
    telemetry_file << "\"omega_y\": " << D1.body.angular_velocity_body.y << ",";
    telemetry_file << "\"omega_z\": " << D1.body.angular_velocity_body.z << ",";

    telemetry_file << "\"sensed_omega_x\": " << sensed_angular_vel.x << ",";
    telemetry_file << "\"sensed_omega_y\": " << sensed_angular_vel.y << ",";
    telemetry_file << "\"sensed_omega_z\": " << sensed_angular_vel.z << ",";

    telemetry_file << "\"filtered_omega_x\": " << filtered_sensed_angular_vel.x << ",";
    telemetry_file << "\"filtered_omega_y\": " << filtered_sensed_angular_vel.y << ",";
    telemetry_file << "\"filtered_omega_z\": " << filtered_sensed_angular_vel.z << ",";
    telemetry_file << "\"est_omega_x\": " << filtered_sensed_angular_vel.x - estimated_gyro_bias.x << ",";
    telemetry_file << "\"est_omega_y\": " << filtered_sensed_angular_vel.y - estimated_gyro_bias.y << ",";
    telemetry_file << "\"est_omega_z\": " << filtered_sensed_angular_vel.z - estimated_gyro_bias.z << ",";
    telemetry_file << "\"est_gyro_bias_x\": " << estimated_gyro_bias.x << ",";
    telemetry_file << "\"est_gyro_bias_y\": " << estimated_gyro_bias.y << ",";
    telemetry_file << "\"est_gyro_bias_z\": " << estimated_gyro_bias.z << ",";

    telemetry_file << "\"omega_1\": " << D1.rotor1.omega << ",";
    telemetry_file << "\"omega_2\": " << D1.rotor2.omega << ",";
    telemetry_file << "\"omega_3\": " << D1.rotor3.omega << ",";
    telemetry_file << "\"omega_4\": " << D1.rotor4.omega << ",";

    telemetry_file << "\"sensed_accel_x\": " << sensed_accel.x << ",";
    telemetry_file << "\"sensed_accel_y\": " << sensed_accel.y << ",";
    telemetry_file << "\"sensed_accel_z\": " << sensed_accel.z << ",";

    telemetry_file << "\"filtered_accel_x\": " << filtered_sensed_accel.x << ",";
    telemetry_file << "\"filtered_accel_y\": " << filtered_sensed_accel.y << ",";
    telemetry_file << "\"filtered_accel_z\": " << filtered_sensed_accel.z << ",";

    telemetry_file << "\"gps_pos_x\": " << gps_data.sensed_position.x << ",";
    telemetry_file << "\"gps_pos_y\": " << gps_data.sensed_position.y << ",";
    telemetry_file << "\"gps_pos_z\": " << gps_data.sensed_position.z << ",";

    telemetry_file << "\"gps_vel_x\": " << gps_data.sensed_velocity.x << ",";
    telemetry_file << "\"gps_vel_y\": " << gps_data.sensed_velocity.y << ",";
    telemetry_file << "\"gps_vel_z\": " << gps_data.sensed_velocity.z << ",";

    telemetry_file << "\"filtered_gps_pos_x\": " << filtered_gps_data.sensed_position.x << ",";
    telemetry_file << "\"filtered_gps_pos_y\": " << filtered_gps_data.sensed_position.y << ",";
    telemetry_file << "\"filtered_gps_pos_z\": " << filtered_gps_data.sensed_position.z << ",";

    telemetry_file << "\"filtered_gps_vel_x\": " << filtered_gps_data.sensed_velocity.x << ",";
    telemetry_file << "\"filtered_gps_vel_y\": " << filtered_gps_data.sensed_velocity.y << ",";
    telemetry_file << "\"filtered_gps_vel_z\": " << filtered_gps_data.sensed_velocity.z << ",";
    telemetry_file << "\"gps_xy_trust\": " << gps_xy_trust << ",";
    telemetry_file << "\"gps_time_since_fresh\": " << gps_time_since_fresh << ",";

    telemetry_file << "\"baro_altitude\": " << baro_altitude << ",";
    telemetry_file << "\"filtered_baro_altitude\": " << filtered_baro_altitude << ",";

    telemetry_file << "\"x_active\": " << (position_hold_x_active ? 1 : 0) << ",";
    telemetry_file << "\"y_active\": " << (position_hold_y_active ? 1 : 0) << ",";
    telemetry_file << "\"z_active\": " << (position_hold_z_active ? 1 : 0) << ",";
    telemetry_file << "\"x_confirm\": " << position_hold_x_confirm << ",";
    telemetry_file << "\"y_confirm\": " << position_hold_y_confirm << ",";
    telemetry_file << "\"z_confirm\": " << position_hold_z_confirm << ",";
    telemetry_file << "\"hold_inside_time\": " << hold_inside_time << ",";
    telemetry_file << "\"hold_outside_time\": " << hold_outside_time << ",";
    telemetry_file << "\"hold_overshoot_time\": " << hold_overshoot_time;

    telemetry_file << "}\n";
    telemetry_file.flush();
}
