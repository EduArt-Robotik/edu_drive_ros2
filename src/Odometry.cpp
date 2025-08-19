#include "Odometry.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace edu
{

Odometry::Odometry(OdometryMode odometry_mode, edu::Matrix invKinematicModel) : _odometry_mode(odometry_mode), _invKinematics(invKinematicModel)
{
    reset();
}

Odometry::~Odometry()
{

}

void Odometry::reset()
{
    _is_pos_init = false;
    _is_vel_init = false;

    _prev_pos_vec = edu::Vec(_invKinematics.cols(), 0.0);
    _prev_time_ns = 0;

    _pose.x = 0;
    _pose.y = 0;
    _pose.theta = 0;
}

void Odometry::set_mode(OdometryMode odometry_mode)
{
    _odometry_mode = odometry_mode;
}

OdometryMode Odometry::get_mode(){
    return _odometry_mode;
}

bool Odometry::is_pos_init()
{
    return _is_pos_init;
}

bool Odometry::is_vel_init()
{
    return _is_vel_init;
}

Pose Odometry::get_pose()
{
    return _pose;
}

int Odometry::update(edu::Vec mot_pos_vec)
{
    int size = static_cast<int>(mot_pos_vec.size());
    if( size !=_invKinematics.cols())
    {
        std::cout   << "Odometry::update: Received velocity vector with invalid length. Received "
                    << mot_pos_vec.size() << "  elements, Expected " <<  _invKinematics.cols() << " elements" << std::endl;
        return -1;
    }

    int status = 1;

    if((_odometry_mode == ODOMETRY_ABSOLUTE_MODE) || (_is_pos_init)){
        
        edu::Vec vOmega = mot_pos_vec;

        // In case absolute values are give, calculate the difference to prev. position
        if(_odometry_mode == ODOMETRY_RELATIVE_MODE){
            for(int i = 0; i < size; i++){
                vOmega.at(i) -= _prev_pos_vec.at(i);
            }
        }

        // Calculate new position in robot ksys
        edu::Vec vTwist = _invKinematics * vOmega;

        if(std::none_of(vTwist.begin(), vTwist.end(), [](double i){ return std::isnan(i) || std::isinf(i); }))
        {
            // Calculate new position in odom ksys
            status = propagate_position(vTwist);
        }else{
            status = -1;
        }
    }
    
    if(status == 1){
        _is_pos_init = true;
        _prev_pos_vec = mot_pos_vec;
    }

    return status;
}

int Odometry::update(uint64_t time_ns, edu::Vec mot_vel_vec)
{
    int size = static_cast<int>(mot_vel_vec.size());
    if(size != _invKinematics.cols())
    {
        std::cout   << "Odometry::update: Received velocity vector with invalid length. Received "
                    << mot_vel_vec.size() << "  elements, Expected " <<  _invKinematics.cols() << " elements" << std::endl;
        return -1;
    }

    int status = 1;

    if((_odometry_mode == ODOMETRY_RELATIVE_MODE) || (_is_vel_init)){

        // In case absolute values are give, calculate the difference to prev. time
        double dt_s = (_odometry_mode == ODOMETRY_ABSOLUTE_MODE) ? (double)(time_ns - _prev_time_ns) / 10e9F : (double)time_ns / 10e9F;
        
        // TODO: Find out where the error is and fix this!
        // Magic number fix:
        dt_s *= 10;

        // Calculate change in motor position
        // Convert rpm to rad per sec
        edu::Vec vOmega = mot_vel_vec;
        
        double factor = dt_s * rpm_to_rad_per_sec_factor;
        for(auto& v : vOmega){
            v *= factor;
        }

        edu::Vec vTwist = _invKinematics * vOmega;

        if(std::none_of(vTwist.begin(), vTwist.end(), [](double i){ return std::isnan(i) || std::isinf(i); }))
        {
            // Calculate new position in odom ksys
            status = propagate_position(vTwist);
        }else{
            status = -1;
        }
    }

    if(status == 1){
        _is_vel_init = true;
        _prev_time_ns = time_ns;
    }

    return status;
}

int Odometry::propagate_position(edu::Vec twistVec)
{
    int status = 1;

    auto dx     = twistVec[0];
    auto dy     = twistVec[1];
    auto dtheta = twistVec[2];

    // Euclidean distance of last move
    double ds = sqrt(dx*dx+dy*dy);

    // Direction of last move in world ksys
    double alpha = _pose.theta + atan2(dy, dx);

    // Turn radius of last move
    double r = ds/dtheta;

    // Calculate movement in world ksys
    double dx_world = 0;
    double dy_world = 0;
    if(dtheta < _straigt_line_threshold){
        // Straigt line move
        dx_world = ds * cos(alpha);
        dy_world = ds * sin(alpha);
    }else{
        // Move on curved path
        dx_world = r * (- sin(alpha) + sin(alpha + dtheta));
        dy_world = r * (+ cos(alpha) - cos(alpha + dtheta));
    }

    // Check for errors
    if(std::isnan(dx) || std::isinf(dx)) status = -2;
    if(std::isnan(dy) || std::isinf(dy)) status = -2;
    if(std::isnan(dtheta) || std::isinf(dtheta)) status = -2;

    if(status == 1){    
        // Sum up increments
        _pose.x += dx_world;
        _pose.y += dy_world;
        _pose.theta += dtheta;
    }

    return status;
}

}
