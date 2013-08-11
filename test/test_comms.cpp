#include "comms.h"
#include "registers.h"
#include "serial/serial.h"
#include <gtest/gtest.h>
#include <fcntl.h>

class FakeSerial : public ::testing::Test {
  protected:
    virtual void SetUp() {
      ASSERT_NE(-1, master_fd = posix_openpt( O_RDWR | O_NOCTTY | O_NDELAY ));
      ASSERT_NE(-1, grantpt(master_fd));
      ASSERT_NE(-1, unlockpt(master_fd));
      ASSERT_TRUE((ser_name = ptsname(master_fd)) != NULL);
      ser.setPort(ser_name);
      ser.open();
      ASSERT_TRUE(ser.isOpen()) << "Couldn't open Serial connection to pseudoterminal.";
    }

    virtual void TearDown() {
      ser.close();
      close(master_fd);
    }

    serial::Serial ser;
    int master_fd;

  private:
    char* ser_name;
};

TEST_F(FakeSerial, basic_message_rx) {
  // Send message from device which should write four bytes to the raw magnetometer's first register.
  std::string msg(um6::Comms::message(UM6_MAG_RAW_XY, std::string("\x1\x2\x3\x4")));
  write(master_fd, msg.c_str(), msg.length());

  um6::Comms sensor(ser);
  um6::Registers registers;
  ASSERT_EQ(UM6_MAG_RAW_XY, sensor.receive(&registers)) << "Didn't return ID of arriving message.";
  EXPECT_EQ(0x0102, registers.mag_raw.get(0));
}

TEST_F(FakeSerial, batch_message_rx) {
  // Send message from device which should write four bytes to the raw accelerometer's registers.
  std::string msg(um6::Comms::message(UM6_ACCEL_RAW_XY, std::string("\x5\x6\x7\x8\x9\xa\0\0", 8)));
  write(master_fd, msg.c_str(), msg.length());

  um6::Comms sensor(ser);
  um6::Registers registers;
  ASSERT_EQ(UM6_ACCEL_RAW_XY, sensor.receive(&registers)) << "Didn't return ID of arriving message.";
  EXPECT_EQ(0x0506, registers.accel_raw.get(0));
  EXPECT_EQ(0x0708, registers.accel_raw.get(1));
  EXPECT_EQ(0x090a, registers.accel_raw.get(2));
}


int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
