#include "RoleConfig.h"

#if defined(TRANSMITTER) == defined(RECEIVER)
#error "Define exactly one of TRANSMITTER or RECEIVER in RoleConfig.h"
#endif
