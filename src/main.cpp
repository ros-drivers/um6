

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "registers.h"
#include "comms.h"


// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = UM6_TEMPERATURE;

/**
 * Send configuration messages to the UM6, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(serial::Serial& ser)
{

}

/**
 * Uses the register::Accessor instances to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um6::Registers& r)
{
  static ros::NodeHandle n;
  static ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1, false);
  static ros::Publisher mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1, false);
  static ros::Publisher rpy_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 1, false);
  static ros::Publisher temp_pub = n.advertise<std_msgs::Float32>("imu/temperature", 1, false);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "imu";

  if (imu_pub.getNumSubscribers() > 0) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = header;

    // IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
    imu_msg.orientation.x = r.quat.get_scaled(2);
    imu_msg.orientation.x = r.quat.get_scaled(1);
    imu_msg.orientation.x = -r.quat.get_scaled(3);
    imu_msg.orientation.x = r.quat.get_scaled(0);

    // NED -> ENU conversion.
    imu_msg.angular_velocity.x = r.gyro.get_scaled(1);
    imu_msg.angular_velocity.y = r.gyro.get_scaled(0);
    imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);
    imu_msg.linear_acceleration.x = -r.accel.get_scaled(1);
    imu_msg.linear_acceleration.y = -r.accel.get_scaled(0);
    imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);

    imu_pub.publish(imu_msg);
  }

  if (mag_pub.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header = header;
    mag_msg.vector.x = r.mag.get_scaled(1);
    mag_msg.vector.y = r.mag.get_scaled(0);
    mag_msg.vector.z = -r.mag.get_scaled(2);
    mag_pub.publish(mag_msg);
  }

  if (rpy_pub.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = header;
    rpy_msg.vector.x = r.euler.get_scaled(0);
    rpy_msg.vector.y = r.euler.get_scaled(1);
    rpy_msg.vector.z = r.euler.get_scaled(2);
    rpy_pub.publish(rpy_msg);
  }

  if (temp_pub.getNumSubscribers() > 0) {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
}

/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um6_driver");

  // Load parameters from private node handle.
  std::string port;
  int32_t baud;
  ros::NodeHandle n_local("~");
  n_local.param<std::string>("port", port, "/dev/ttyUSB0");
  n_local.param("baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout::simpleTimeout(500);
  ser.setTimeout(to);

  bool first_failure = true;
  while (ros::ok()) {
    try {
      ser.open();
    } catch (serial::IOException& e) {
      ROS_DEBUG("Unable to connect to port.");
    }
    if (ser.isOpen()) {
      ROS_INFO("Successfully connected to serial port.");
      first_failure = true;
      try {
        um6::Comms sensor(ser);


        while(ros::ok()) {
          if (sensor.receive() == TRIGGER_PACKET) {
            // Triggered by arrival of final message in group.
            publishMsgs(sensor.registers);
          }
        }
      } catch (std::exception& e) {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    } else {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device " << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
