#pragma once

#include <stdint.h>
#include <string.h>
#include <endian.h>

#include "firmware_registers.h"

#define PI 3.14159265359
#define TO_RADIANS (PI / 180.0)
#define TO_DEGREES (180.0 / PI)
#define NUM_REGISTERS (COMMAND_START_ADDRESS + COMMAND_COUNT)


namespace um6 {

inline static void memcpy_network(void* dest, void* src, size_t count) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
  for (uint8_t i = 0; i < count; i++) {
    ((uint8_t*)dest)[i] = ((uint8_t*)src)[count - (i+1)]; 
  }
#else  
  // Copy bytes without reversing.
  #warning Big-endian implementation is untested.
  memcpy(dest, src, count);
#endif
}

class Registers;
 
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
class Accessor_ {
  public:
    Accessor_(Registers* registers, uint8_t register_index, uint8_t array_length)
      : index(register_index), length(array_length), registers_(registers)
    {}

    void* raw();
    const uint8_t index;
    const uint16_t length;
    
  private:
    Registers* registers_;
};

template<typename RegT>
class Accessor : public Accessor_ {
  public:
    Accessor(Registers* registers, uint8_t register_index, uint8_t array_length, double scale_factor=1.0)
      : Accessor_(registers, register_index, array_length), scale_(scale_factor)
    {}

    RegT get(uint8_t index)
    {
      RegT value;
      memcpy_network(&value, (RegT*)raw() + index, sizeof(value));
      return value;
    }

    double get_scaled(uint16_t index) {
      return get(index) * scale_;
    }

    void set(uint8_t index, RegT val)
    {

    }

  private:
    const double scale_; 
};

class Registers
{
  public:
    Registers() :
      gyro_raw(this, UM6_GYRO_RAW_XY, 3),
      accel_raw(this, UM6_ACCEL_RAW_XY, 3),
      mag_raw(this, UM6_MAG_RAW_XY, 3),
      gyro(this, UM6_GYRO_PROC_XY, 3, 0.0610352 * TO_RADIANS),
      accel(this, UM6_ACCEL_PROC_XY, 3, 0.000183105),
      mag(this, UM6_MAG_PROC_XY, 3, 0.000305176),
      euler(this, UM6_EULER_PHI_THETA, 3, 0.0109863 * TO_RADIANS),
      quat(this, UM6_QUAT_AB, 4, 0.0000335693),
      covariance(this, UM6_ERROR_COV_00, 16),
      temperature(this, UM6_TEMPERATURE, 1)
    {
      memset(raw, 0, sizeof(raw));
    }

    uint32_t raw[NUM_REGISTERS];

    Accessor<int16_t> gyro_raw, accel_raw, mag_raw;
    Accessor<int16_t> gyro, accel, mag, euler, quat;
    Accessor<float> covariance, temperature;
};

}
