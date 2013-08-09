
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

    bool spinOnce(const uint8_t trigger_packet=0xff);
};

}
