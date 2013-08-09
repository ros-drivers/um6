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


int16_t Comms::receive(Registers& registers) {
  int16_t successful_packet = -1;

  // Search the serial stream for a start-of-packet sequence.
  std::string snp;
  serial_.readline(snp, 96, "snp");
  if (boost::algorithm::ends_with(snp, "snp")) {
    uint16_t checksum_calculated = 's' + 'n' + 'p';
    ROS_WARN_COND(!first_spin_ && snp.length() > 3, 
        "Discarded %ld junk byte(s) preceeding packet.", snp.length() - 3);
    uint8_t type, address;
    serial_.read(&type, 1);
    serial_.read(&address, 1);
    checksum_calculated += type + address;
    if (type & PACKET_HAS_DATA) {
      uint8_t data_length = 1;
      if (type & PACKET_IS_BATCH) {
        data_length = (type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        ROS_DEBUG("Received packet %02x with batched (%d) data.", address, data_length);
      } else {
        ROS_DEBUG("Received packet %02x with non-batched data.", address);
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
        registers.write_raw(address, data);
        successful_packet = address;
      } else {
        ROS_WARN("Discarding packet due to bad checksum.");
        ROS_DEBUG("Computed checksum: %04x  Transmitted checksum: %04x", 
            checksum_calculated, checksum_transmitted);
      }
    } else {
      ROS_DEBUG("Received packet %02x without data.", address);
    }

  }
  first_spin_ = false;
  return successful_packet;
}


void Comms::send(Accessor_& r) {
  
  std::stringstream ss("snp");
  uint8_t type;
  if (r.length > 0) type |= PACKET_HAS_DATA;
  if (r.length > 1) {
    type |= PACKET_IS_BATCH;
    type |= r.length << PACKET_BATCH_LENGTH_OFFSET;
  }
  ss << type << r.index;

  std::string data((char*)r.raw(), r.length * 4);
  ss << data;

  uint16_t checksum = 0;
  BOOST_FOREACH(uint8_t ch, ss.str())
    checksum += ch;

  ss << checksum;
  serial_.write(ss.str());
}

void Comms::sendWaitAck(Accessor_&) {

}

}
