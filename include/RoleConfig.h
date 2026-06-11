#ifndef ROLE_CONFIG_H
#define ROLE_CONFIG_H

#if ((defined(TRANSMITTER) ? 1 : 0) + \
     (defined(RECEIVER) ? 1 : 0) + \
     (defined(WHEEL_ENCODER_SLAVE) ? 1 : 0)) != 1
#error "Build with exactly one firmware role: transmitter, receiver, or wheel_encoder_slave"
#endif

#endif
