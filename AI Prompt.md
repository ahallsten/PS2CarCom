the serial output is still super slow. Then if we take a look at the leds that I have tied to the pwm pins, they are giving quick and periodic bursts like there is something in the loop that is taking a long time. I think there is definitely some blocking function or way too much computing is going on somewhere and the little atmel 32u4 is just getting a little overwhelmed we need to find out where it it taking so much time. One thing to note is the receiver doesn't seem to be going into enable. so we also need to find out what is causeing that as well. here is an example of the logs:

RX control seq 53 | present: YES | RSSI: -70 | ButtonWord: 0000000000000000 | LX: 93 | LY: 0 | RX: 130 | RY: 121
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 54 | present: YES | RSSI: -70 | ButtonWord: 0000000000000000 | LX: 95 | LY: 0 | RX: 130 | RY: 121
RX control seq 61 | present: YES | RSSI: -70 | ButtonWord: 0000000000000000 | LX: 97 | LY: 0 | RX: 128 | RY: 121
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 68 | present: YES | RSSI: -69 | ButtonWord: 0000000000000000 | LX: 121 | LY: 0 | RX: 128 | RY: 121
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 75 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 255 | LY: 133 | RX: 130 | RY: 121
 | pwmLY 0 | pwmLX -255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY 0 | pwmLX -255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 89 | present: YES | RSSI: -69 | ButtonWord: 0000000000000000 | LX: 255 | LY: 133 | RX: 130 | RY: 121
 | pwmLY 0 | pwmLX -255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 98 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 255 | LY: 133 | RX: 130 | RY: 121
 | pwmLY 0 | pwmLX -255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 105 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 142 | LY: 127 | RX: 130 | RY: 121
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 112 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 0 | LY: 127 | RX: 130 | RY: 121
 | pwmLY 0 | pwmLX 255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 119 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 121 | LY: 52 | RX: 130 | RY: 121
 | pwmLY 134 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 120 | present: YES | RSSI: -69 | ButtonWord: 0000000000000000 | LX: 121 | LY: 0 | RX: 130 | RY: 121
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 136 | present: YES | RSSI: -69 | ButtonWord: 0000000000000000 | LX: 121 | LY: 0 | RX: 130 | RY: 121
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 151 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 121 | LY: 0 | RX: 130 | RY: 121
RX control seq 158 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 124 | LY: 255 | RX: 130 | RY: 118
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 174 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 124 | LY: 255 | RX: 130 | RY: 118
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 181 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 126 | LY: 255 | RX: 130 | RY: 118
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 189 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 126 | LY: 130 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 196 | present: YES | RSSI: -68 | ButtonWord: 0000001000000000 | LX: 126 | LY: 118 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 204 | present: YES | RSSI: -67 | ButtonWord: 0000000000000000 | LX: 126 | LY: 118 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 219 | present: YES | RSSI: -67 | ButtonWord: 0000001000000000 | LX: 126 | LY: 118 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 227 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 135 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 234 | present: YES | RSSI: -67 | ButtonWord: 0000001000000000 | LX: 126 | LY: 132 | RX: 130 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 241 | present: YES | RSSI: -67 | ButtonWord: 0000000000000000 | LX: 126 | LY: 132 | RX: 130 | RY: 118
RX control seq 249 | present: YES | RSSI: -66 | ButtonWord: 0000001000000000 | LX: 126 | LY: 0 | RX: 130 | RY: 118
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 0 | present: YES | RSSI: -67 | ButtonWord: 0000000000000000 | LX: 126 | LY: 0 | RX: 130 | RY: 118
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 8 | present: YES | RSSI: -67 | ButtonWord: 0000001000000000 | LX: 126 | LY: 0 | RX: 130 | RY: 118
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 16 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 0 | RX: 130 | RY: 118
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 23 | present: YES | RSSI: -67 | ButtonWord: 0000001000000000 | LX: 126 | LY: 255 | RX: 130 | RY: 118
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 38 | present: YES | RSSI: -66 | ButtonWord: 0000001000000000 | LX: 128 | LY: 255 | RX: 130 | RY: 118
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 45 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 125 | RX: 255 | RY: 153
 | pwmLY 0 | pwmLX 0 | pwmRY -41 | pwmSTR -255 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 50 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 125 | RX: 0 | RY: 118
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 255 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 57 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 125 | RX: 0 | RY: 120
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 255 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 63 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 125 | RX: 93 | RY: 109
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 29 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 68 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 126 | LY: 124 | RX: 138 | RY: 0
 | pwmLY 0 | pwmLX 0 | pwmRY 255 | pwmSTR -6 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 74 | present: YES | RSSI: -72 | ButtonWord: 0000000000000000 | LX: 126 | LY: 124 | RX: 111 | RY: 0
 | pwmLY 0 | pwmLX 0 | pwmRY 255 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 81 | present: YES | RSSI: -71 | ButtonWord: 0000000000000000 | LX: 126 | LY: 124 | RX: 109 | RY: 255
 | pwmLY 0 | pwmLX 0 | pwmRY -255 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 87 | present: YES | RSSI: -70 | ButtonWord: 0000000000000000 | LX: 126 | LY: 124 | RX: 114 | RY: 255
 | pwmLY 0 | pwmLX 0 | pwmRY -255 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 93 | present: YES | RSSI: -66 | ButtonWord: 0000000000000000 | LX: 0 | LY: 153 | RX: 114 | RY: 113
 | pwmLY -18 | pwmLX 255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 101 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 145 | LY: 149 | RX: 114 | RY: 113
 | pwmLY -9 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 108 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 255 | LY: 150 | RX: 114 | RY: 113
 | pwmLY -11 | pwmLX -255 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 126 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 140 | LY: 255 | RX: 114 | RY: 113
 | pwmLY -255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 141 | present: YES | RSSI: -68 | ButtonWord: 0000000000000000 | LX: 104 | LY: 0 | RX: 114 | RY: 113
 | pwmLY 255 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
RX control seq 150 | present: YES | RSSI: -69 | ButtonWord: 0000000000000000 | LX: 111 | LY: 122 | RX: 114 | RY: 113
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF
 | pwmLY 0 | pwmLX 0 | pwmRY 0 | pwmSTR 0 | EN: OFF | T_MD: OFF | p_Brk: OFF

 Can we maybe do a better job of standardizeing the debug messages as well? Maybe we also need to work in time stamps and message ID on every message? Essentially a very minimal time stamp and message ID that looked at by in the transmitter and reciever logs to show which messages have been recieved. Maybe we print out ACK status's as well? 