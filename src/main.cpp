

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
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in UM6 registers.
 */
template<typename RegT>
void configureVector3(um6::Comms& sensor, um6::Accessor<RegT>& reg,
    std::string param, std::string human_name) {
  if (reg.length != 3) {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (!sensor.sendWaitAck(reg)) {

      throw std::runtime_error("Unable to configure "); // << human_name);
    }
  }
}

/**
 * Send configuration messages to the UM6, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(um6::Comms& sensor)
{
  um6::Registers r;

  // Enable outputs we need.
  const uint8_t UM6_BAUD_115200 = 0x5;
  uint32_t comm_reg = UM6_BROADCAST_ENABLED |
      UM6_GYROS_PROC_ENABLED | UM6_ACCELS_PROC_ENABLED | UM6_MAG_PROC_ENABLED | 
      UM6_QUAT_ENABLED | UM6_EULER_ENABLED | UM6_COV_ENABLED | UM6_TEMPERATURE_ENABLED |
      UM6_BAUD_115200 << UM6_BAUD_START_BIT;
  r.communication.set(0, comm_reg);
  if (!sensor.sendWaitAck(r.communication)) {
    throw std::runtime_error("Unable to set configuration register.");
  }

  // Configurable vectors.
  configureVector3(sensor, r.mag_ref, "~mag_ref", "magnetic reference vector");
  configureVector3(sensor, r.accel_ref, "~accel_ref", "accelerometer reference vector");
  configureVector3(sensor, r.mag_bias, "~mag_bias", "magnetic bias vector");
  configureVector3(sensor, r.accel_bias, "~accel_bias", "accelerometer bias vector");
  configureVector3(sensor, r.gyro_bias, "~gyro_bias", "gyroscope bias vector");
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um6::Registers& r, ros::NodeHandle& n, std_msgs::Header& header)
{
  static ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1, false);
  static ros::Publisher mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1, false);
  static ros::Publisher rpy_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 1, false);
  static ros::Publisher temp_pub = n.advertise<std_msgs::Float32>("imu/temperature", 1, false);

  if (imu_pub.getNumSubscribers() > 0) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = header;

    // IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
    imu_msg.orientation.x = r.quat.get_scaled(2);
    imu_msg.orientation.y = r.quat.get_scaled(1);
    imu_msg.orientation.z = -r.quat.get_scaled(3);
    imu_msg.orientation.w = r.quat.get_scaled(0);

    // IMU reports a 4x4 wxyz covariance, ROS requires only 3x3 xyz.
    // NED -> ENU conversion req'd?
    imu_msg.orientation_covariance[0] = r.covariance.get_scaled(5);
    imu_msg.orientation_covariance[1] = r.covariance.get_scaled(6);
    imu_msg.orientation_covariance[2] = r.covariance.get_scaled(7);
    imu_msg.orientation_covariance[3] = r.covariance.get_scaled(9);
    imu_msg.orientation_covariance[4] = r.covariance.get_scaled(10);
    imu_msg.orientation_covariance[5] = r.covariance.get_scaled(11);
    imu_msg.orientation_covariance[6] = r.covariance.get_scaled(13);
    imu_msg.orientation_covariance[7] = r.covariance.get_scaled(14);
    imu_msg.orientation_covariance[8] = r.covariance.get_scaled(15);

    // NED -> ENU conversion.
    imu_msg.angular_velocity.x = r.gyro.get_scaled(1);
    imu_msg.angular_velocity.y = r.gyro.get_scaled(0);
    imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

    // NED -> ENU conversion.
    imu_msg.linear_acceleration.x = r.accel.get_scaled(1);
    imu_msg.linear_acceleration.y = r.accel.get_scaled(0);
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
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  ros::param::param<int32_t>("~baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  ser.setTimeout(to);

  ros::NodeHandle n;
  std_msgs::Header header;
  ros::param::param<std::string>("~frame_id", header.frame_id, "imu_link");

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
        configureSensor(sensor);

        um6::Registers registers;
        while(ros::ok()) {
          if (sensor.receive(&registers) == TRIGGER_PACKET) {
            // Triggered by arrival of final message in group.
            header.stamp = ros::Time::now();
            publishMsgs(registers, n, header);
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
