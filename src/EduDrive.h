#ifndef __EDU_DRIVE_H
#define __EDU_DRIVE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "MotorController.h"
#include "RPiAdapterBoard.h"
#include "RPiExtensionBoard.h"
#include "PowerManagementBoard.h"
#include <rclcpp/publisher.hpp>
#include "Odometry.h"
#include "tf2_ros/transform_broadcaster.h"

namespace edu
{

/**
 * @class EduDrive
 * @brief Drive interface to EduArt's stackable motor controllers
 * @author Stefan May
 * @date 27.04.2022
 */
class EduDrive : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     *
     */
    EduDrive();

    /**
     * @brief Destructor
     *
     */
    ~EduDrive();

    /**
     * @brief Initialize drive
     * @param[in] cp Parameters for the closed-loop controller of all motor channels
     * @param[in] can Instance of CAN communication socket. All connected devices share the communication bus.
     * @param[in] using_pwr_mgmt Flag indicating whether the power management module of EduArt is used. This enables additional measurement topics (voltage and current sensing).
     * @param[in] verbosity Make the instance of this class more chatty. Debug information will be printed to stdout.
     */
    void initDrive(std::vector<ControllerParams> cp, SocketCAN& can, bool using_pwr_mgmt=true, bool verbosity=false);

    /**
     * @brief Blocking ROS handler method. Call this method to enter the ROS message loop.
     */
    void run();

    /**
     * @brief Enable all drives
     */
    void enable();

    /**
     * @brief Disable all drives
     */
    void disable();

    /**
     * @brief Method called by ROS as joystick data is available
     * @param joy joystick data
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

    /**
     * @brief Method called by ROS as twist data is available
     */
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);

    /**
     * @brief 
     */
    void receiveCAN();
    
    void checkLaggyConnection();
    
private:

    int gpio_write(const char *dev_name, int offset, int value);

    int gpio_read(const char *dev_name, int offset, int &value);

    void controlMotors(float vFwd, float vLeft, float omega);

    bool enableCallback(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request, const std::shared_ptr<std_srvs::srv::SetBool_Response> response);

    // Input topics / services
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr           _subJoy;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       _subVel;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr               _srvEnable;

    // Data available from motor controllers
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr      _pubEnabled;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   _pubRPM;
    
    // Data available from adapter board
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubTemp;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubVoltageAdapter;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    _pubOrientation;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr   _pubAccel;
    
    // Data available from power management board
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubVoltagePwrMgmt;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubCurrentPwrMgmt;

    // Odometry
    std::unique_ptr<tf2_ros::TransformBroadcaster>                   _tf_broadcaster;

    rclcpp::Time                   _lastCmd;       // Time elapsed since last call

    SocketCAN*                     _can;           // Pointer to CAN instance

    std::vector<MotorController*>  _mc;            // Vector containing pointer to all motor controller instances
    RPiAdapterBoard*               _adapter;       // Adapter board
    RPiExtensionBoard*             _extension;     // Extension board
    PowerManagementBoard*          _pwr_mgmt;      // Power management board
    Odometry*                      _odometry;      // Odometry model of robot

    double _vMax;
    double _omegaMax;

    bool _enabled;
    bool _using_pwr_mgmt;
    bool _verbosity;
};

} // namespace

#endif //__EDU_DRIVE_H
