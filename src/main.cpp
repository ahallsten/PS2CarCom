#include "RoleConfig.h"

#if defined(TRANSMITTER) == defined(RECEIVER)
#error "Build with exactly one PlatformIO role environment: transmitter or receiver"
#endif
