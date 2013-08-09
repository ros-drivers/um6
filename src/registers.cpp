
#include "registers.h"

namespace um6 {

void* Accessor_::raw() {
  // This is ridiculous to have a whole source file for this tiny implementation,
  // but it's necessary to resolve the otherwise circular dependency between the
  // Registers and Accessor classes, when Registers contains Accessor instances
  // and Accessor is a template class.
  return &registers_->raw_[index];
}

}
