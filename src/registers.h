
#include "firmware_registers.h"

namespace registers {

uint32_t raw[COMMAND_START_ADDRESS + COMMAND_COUNT];

/**
 * This class provides an accessor of fields contained in one or more
 * consecutive UM6 registers. Each register is nominally a uint32_t, 
 * but XYZ vectors are stored as a pair of int16_t values in one 
 * register and one in the following register. Other values are 
 * stored as int32_t representation or float32s.
 *
 * This class takes care of the necessary transformations to simplify
 * the actual "business logic" of the driver.
 */
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
#if __BYTE_ORDER == __LITTLE_ENDIAN
      // Reverse bytes.
      for (uint8_t i = 0; i < sizeof(RegT); i++) {
        bytes[i] = ptr_[index][sizeof(RegT) - (i+1)]; 
      }
#else  
#warning Big-endian implementation is untested.
      // Copy bytes without reversing.
      memcpy(bytes, ptr_, sizeof(RegT));
#endif
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


