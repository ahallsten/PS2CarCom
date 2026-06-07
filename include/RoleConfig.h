#ifndef ROLE_CONFIG_H
#define ROLE_CONFIG_H

#if defined(TRANSMITTER) == defined(RECEIVER)
#error "Build with exactly one firmware role: transmitter or receiver"
#endif

#endif
