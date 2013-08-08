
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>

// Byte-order related stuff.
#include <endian.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "firmware_registers.h"


namespace registers {

static uint32_t raw[COMMAND_START_ADDRESS + COMMAND_COUNT];

template<typename RegT>
class Accessor {
  public:
    Accessor(uint8_t register_number, uint8_t array_length, double scale_factor=1.0)
      : ptr_((bytes_t*)(&raw[register_number])), length_(array_length), scale_(scale_factor)
    {}

    RegT get(uint8_t index)
    {
      union {
        uint8_t bytes[sizeof(RegT)];
        RegT value;
      };

      // Reverse bytes if necessary.
      for (uint8_t i = 0; i < sizeof(RegT); i++) {
      #if __BYTE_ORDER == __LITTLE_ENDIAN
        bytes[i] = ptr_[index][sizeof(RegT) - (i+1)]; 
      #else  
        #warning Big-endian implementation is untested.
        bytes[i] = ptr_[index][i]; 
      #endif
      }

      return value;
    }

    double get_scaled(uint16_t index) {
      return get(index) * scale_;
    }
    
  private:
    typedef uint8_t bytes_t[sizeof(RegT)];
    bytes_t * const ptr_;
    const double scale_; 
    const uint16_t length_;
};

#define PI 3.14159265359
#define TO_RADIANS (PI / 180.0)
#define TO_DEGREES (180.0 / PI)

Accessor<int16_t> gyro_raw(UM6_GYRO_RAW_XY, 3);
Accessor<int16_t> accel_raw(UM6_ACCEL_RAW_XY, 3);
Accessor<int16_t> mag_raw(UM6_MAG_RAW_XY, 3);

// Scale factors copied from Python driver.
Accessor<int16_t> gyro(UM6_GYRO_PROC_XY, 3, 0.0610352 * TO_RADIANS);
Accessor<int16_t> accel(UM6_ACCEL_PROC_XY, 3, 0.000183105);
Accessor<int16_t> mag(UM6_MAG_PROC_XY, 3, 0.000305176);
Accessor<int16_t> euler(UM6_EULER_PHI_THETA, 3, 0.0109863 * TO_RADIANS);
Accessor<int16_t> quat(UM6_QUAT_AB, 4, 0.0000335693);
Accessor<float> covariance(UM6_ERROR_COV_00, 16);

Accessor<float> temperature(UM6_TEMPERATURE, 1);
}

#define PACKET_HAS_DATA     (1 << 7)
#define PACKET_IS_BATCH     (1 << 6)
#define PACKET_BATCH_LENGTH_MASK  (0x0F)
#define PACKET_BATCH_LENGTH_OFFSET  2

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
#define TRIGGER_PACKET UM6_TEMPERATURE


void configureSensor(serial::Serial& ser)
{


}

void handlePublish()
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

    using namespace registers;

    // IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
    imu_msg.orientation.x = quat.get_scaled(2);
    imu_msg.orientation.x = quat.get_scaled(1);
    imu_msg.orientation.x = -quat.get_scaled(3);
    imu_msg.orientation.x = quat.get_scaled(0);

    // NED -> ENU conversion.
    imu_msg.angular_velocity.x = gyro.get_scaled(1);
    imu_msg.angular_velocity.y = gyro.get_scaled(0);
    imu_msg.angular_velocity.z = -gyro.get_scaled(2);
    imu_msg.linear_acceleration.x = -gyro.get_scaled(1);
    imu_msg.linear_acceleration.y = -gyro.get_scaled(0);
    imu_msg.linear_acceleration.z = -gyro.get_scaled(2);

    imu_pub.publish(imu_msg);
  }

  if (mag_pub.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header = header;

    using namespace registers;
    mag_msg.vector.x = mag.get_scaled(1);
    mag_msg.vector.y = mag.get_scaled(0);
    mag_msg.vector.z = -mag.get_scaled(2);

    mag_pub.publish(mag_msg);
  }

  if (rpy_pub.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = header;

    using namespace registers;
    rpy_msg.vector.x = euler.get_scaled(0);
    rpy_msg.vector.y = euler.get_scaled(1);
    rpy_msg.vector.z = euler.get_scaled(2);

    rpy_pub.publish(rpy_msg);
  }

  if (temp_pub.getNumSubscribers() > 0) {
    std_msgs::Float32 temp_msg;
    temp_msg.data = registers::temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
}

void readSensor(serial::Serial& ser)
{
  bool first = true;

  while(ros::ok()) {
    // Quick and dirty way to find a start-of-packet.
    std::string snp;
    ser.readline(snp, 96, "snp");
    if (boost::algorithm::ends_with(snp, "snp")) {
      uint16_t checksum_calculated = 's' + 'n' + 'p';
      if (!first) 
        ROS_WARN_COND(snp.length() > 3, "Discarded %ld junk byte(s) preceeding packet.", snp.length() - 3);
      uint8_t type_address[2];
      ser.read(type_address, 2);
      checksum_calculated += type_address[0] + type_address[1];
      if (type_address[0] & PACKET_HAS_DATA) {
        uint8_t data_length = 1;
        if (type_address[0] & PACKET_IS_BATCH) {
          data_length = (type_address[0] >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
          ROS_DEBUG("Received packet %02x with batched (%d) data.", type_address[1], data_length);
        } else {
          ROS_DEBUG("Received packet %02x with non-batched data.", type_address[1]);
        }

        // Read data bytes initially into a buffer so that we can verify checksum.
        std::string data;
        ser.read(data, data_length * 4);
        BOOST_FOREACH(uint8_t ch, data)
          checksum_calculated += ch;
     
        // Compute and compare the checksum.
        uint16_t checksum_transmitted; 
        ser.read((uint8_t*)&checksum_transmitted, 2);
        checksum_transmitted = ntohs(checksum_transmitted);
        if (checksum_transmitted == checksum_calculated) {
          // Copy data from checksum buffer into registers.
          // Byte-order correction (as necessary) happens at access-time.
          memcpy(&registers::raw[type_address[1]], data.c_str(), data.length());

          if (type_address[1] == TRIGGER_PACKET) {
            handlePublish();
            ros::spinOnce();
          }
        } else {
          ROS_WARN("Discarding packet due to bad checksum.");
          ROS_DEBUG("Computed checksum: %04x  Transmitted checksum: %04x", 
              checksum_calculated, checksum_transmitted);
        }
      } else {
        ROS_DEBUG("Received packet %02x without data.", type_address[1]);
      }

    }
    first = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "um6_driver");

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

  // Setup ROS publishers?

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
        configureSensor(ser);
        readSensor(ser);
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
