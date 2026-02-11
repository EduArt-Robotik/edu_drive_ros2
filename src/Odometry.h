#pragma once

#include "Matrix.h"
#include <cstdint>
#include <vector>

namespace edu
{

    constexpr double edu_PI = 3.14159265358979323846;
    constexpr double rpm_to_rad_per_sec_factor = 2 * edu_PI / 60.0F;

    enum OdometryMode
    {
        ODOMETRY_RELATIVE_MODE = 0,
        ODOMETRY_ABSOLUTE_MODE = 1
    };

    struct Pose
    {
        double x;
        double y;
        double theta;
    };

    /**
    * @class Odometry
    * @brief Simple odometry estimation based on motor speeds or motor positions
    * @author Hannes Duske
    * @date 15.03.2024
    */
    class Odometry
    {
    public:

        /**
         * Constructor
         * @param[in] absolute_mode Set odometry to absolute or relative mode
         * @param[in] invKinematicModel Matrix of inverted kinematic vectors (Wheel spin to Twist conversion)
         */
        Odometry(OdometryMode odometry_mode, edu::Matrix invKinematicModel);

        /**
         * Destructor
         */
        ~Odometry();

        /**
         * Reset odometry estimation
         */
        void reset();

        /**
         * Set odometry to absolute or relative mode
         * @param[in] odometry_mode
         */
        void set_mode(OdometryMode odometry_mode);

        /**
         * Get current odometry mode
         */
        OdometryMode get_mode();

        /**
         * Check if odometry position-model is initialized
        */
       bool is_pos_init();

       /**
         * Check if odometry velocity-model is initialized
        */
       bool is_vel_init();

        /*
        * Get current pose estimate
        */
       Pose get_pose();

        /**
         * Update odometry estimation with new wheel positions
         * Absolute position or change in position depends on odometry_mode setting
         * @param[in] mot_pos_vec Vector of the motor position. Either absolute or relative position depending on the OdometryMode setting
         * @retval status 1: o.k., status -1: error in last step (no update)
         */
        int update(edu::Vec mot_pos_vec);

        /**
         * Update odometry estimation with new wheel speeds
         * Absolute time or change in time depends on odometry_mode setting
         * @param[in] time_ns Time in nanoseconds
         * @param[in] mot_vel_vec Vector of the motor velocities.
         * @retval status 1: o.k., status -1: error in last step (no update)
         */
        int update(uint64_t time_ns, edu::Vec mot_vel_vec);

    private:

        /**
         * Calculate pose-change in world coordinate system
         * @param[in] twistVec Twist vector describing change in x, y and theta in the robot coordinate system
         * @retval status 1: o.k., status -1: error in calculation
         */
        int propagate_position(edu::Vec twistVec);

        Pose _pose;
        OdometryMode _odometry_mode;

        const double _straight_line_threshold = 0.001F;
        bool _is_pos_init;
        bool _is_vel_init;

        edu::Vec _prev_pos_vec;
        uint64_t _prev_time_ns;

        edu::Matrix _invKinematics;
    };

}
