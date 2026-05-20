#include "dsp/transducer_response.hpp"

namespace openCREST::dsp {

// Anchor the TransducerResponse vtable in this TU so the compiler does
// not emit a weak vtable in every translation unit that includes the
// header.
TransducerResponse::~TransducerResponse() = default;

} // namespace openCREST::dsp
