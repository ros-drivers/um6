
#include "registers.h"

namespace serial {
  class Serial;
}

namespace um6 {

class Comms
{
  private:
    bool first_spin_;
    serial::Serial& serial_;

  public:
    Registers registers;

    Comms(serial::Serial& s) : serial_(s), first_spin_(true) {
    }

    /**
     * Returns -1 if the serial port timed out before receiving a packet
     * successfully, or if there was a bad checksum or any other error.
     * Otherwise, returns the 8-bit register number of the successfully
     * returned packet.
     */
    int16_t receive();

    void send(Accessor_&);

    void sendAndAck(Accessor_&);
};

}
