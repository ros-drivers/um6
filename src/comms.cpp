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


int16_t Comms::receive(Registers* registers = NULL) {
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
    std::string data;
    if (type & PACKET_HAS_DATA) {
      uint8_t data_length = 1;
      if (type & PACKET_IS_BATCH) {
        data_length = (type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        ROS_DEBUG("Received packet %02x with batched (%d) data.", address, data_length);
      } else {
        ROS_DEBUG("Received packet %02x with non-batched data.", address);
      }

      // Read data bytes initially into a buffer so that we can compute the checksum.
      serial_.read(data, data_length * 4);
      BOOST_FOREACH(uint8_t ch, data)
        checksum_calculated += ch;
    } else {
      ROS_DEBUG("Received packet %02x without data.", address);
    }
   
    // Compare computed checksum with transmitted value.
    uint16_t checksum_transmitted; 
    serial_.read((uint8_t*)&checksum_transmitted, 2);
    checksum_transmitted = ntohs(checksum_transmitted);
    if (checksum_transmitted == checksum_calculated) {
      // Copy data from checksum buffer into registers, if specified.
      // Note that byte-order correction (as necessary) happens at access-time.
      if ((type & PACKET_HAS_DATA) and registers) {
        registers->write_raw(address, data);
      }
      successful_packet = address;
    } else {
      ROS_WARN("Discarding packet due to bad checksum.");
      ROS_DEBUG("Computed checksum: %04x  Transmitted checksum: %04x", 
                checksum_calculated, checksum_transmitted);
    }
  }
  first_spin_ = false;
  return successful_packet;
}

void Comms::send(Accessor_& r) {
  
  std::stringstream ss(std::stringstream::out | std::stringstream::binary);
  ss << "snp";

  uint8_t type;
  if (r.length > 0)
    type |= PACKET_HAS_DATA;
  if (r.length > 1) {
    type |= PACKET_IS_BATCH;
    type |= r.length << PACKET_BATCH_LENGTH_OFFSET;
  }
  std::string data((char*)r.raw(), r.length * 4);
  ss << type << r.index << data;

  uint16_t checksum = 0;
  std::string checksum_string = ss.str();
  BOOST_FOREACH(uint8_t ch, checksum_string)
    checksum += ch;
  checksum = htons(checksum);
  ss << std::string((char*)&checksum, sizeof(checksum));
  
  serial_.write(ss.str());
}

bool Comms::sendWaitAck(Accessor_& r) {
  const uint8_t tries = 5;
  for(uint8_t t = 0; t < tries; t++) {
    send(r);
    const uint8_t listens = 20;
    for(uint8_t i = 0; i < listens; i++) {
      int16_t received = receive();
      if (received == r.index) {
        ROS_DEBUG("Message %02x ack received.", received);
        return true;
      } else if (received == -1) {
        ROS_DEBUG("Serial read timed out waiting for ack. Attempting to retransmit.");
        break;
      }
    } 
  }
  return false;
}

}
