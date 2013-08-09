#include "comms.h"
#include "registers.h"

#include "ros/console.h"
#include "serial/serial.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <arpa/inet.h>

namespace um6 {

const uint8_t PACKET_HAS_DATA = 1 << 7;
const uint8_t PACKET_IS_BATCH = 1 << 6;
const uint8_t PACKET_BATCH_LENGTH_MASK = 0x0F;
const uint8_t PACKET_BATCH_LENGTH_OFFSET = 2;


int16_t Comms::receive() {
  int16_t successful_packet = -1;

  // Search the serial stream for a start-of-packet sequence.
  std::string snp;
  serial_.readline(snp, 96, "snp");
  if (boost::algorithm::ends_with(snp, "snp")) {
    uint16_t checksum_calculated = 's' + 'n' + 'p';
    ROS_WARN_COND(!first_spin_ && snp.length() > 3, 
        "Discarded %ld junk byte(s) preceeding packet.", snp.length() - 3);
    uint8_t type_address[2];
    serial_.read(type_address, 2);
    checksum_calculated += type_address[0] + type_address[1];
    if (type_address[0] & PACKET_HAS_DATA) {
      uint8_t data_length = 1;
      if (type_address[0] & PACKET_IS_BATCH) {
        data_length = (type_address[0] >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        ROS_DEBUG("Received packet %02x with batched (%d) data.", type_address[1], data_length);
      } else {
        ROS_DEBUG("Received packet %02x with non-batched data.", type_address[1]);
      }

      // Read data bytes initially into a buffer so that we can compute the checksum.
      std::string data;
      serial_.read(data, data_length * 4);
      BOOST_FOREACH(uint8_t ch, data)
        checksum_calculated += ch;
   
      // Compare computed checksum with transmitted value.
      uint16_t checksum_transmitted; 
      serial_.read((uint8_t*)&checksum_transmitted, 2);
      checksum_transmitted = ntohs(checksum_transmitted);
      if (checksum_transmitted == checksum_calculated) {
        // Copy data from checksum buffer into registers.
        // Byte-order correction (as necessary) happens at access-time.
        memcpy(&registers.raw[type_address[1]], data.c_str(), data.length());
        successful_packet = type_address[1];
      } else {
        ROS_WARN("Discarding packet due to bad checksum.");
        ROS_DEBUG("Computed checksum: %04x  Transmitted checksum: %04x", 
            checksum_calculated, checksum_transmitted);
      }
    } else {
      ROS_DEBUG("Received packet %02x without data.", type_address[1]);
    }

  }
  first_spin_ = false;
  return successful_packet;
}

}
