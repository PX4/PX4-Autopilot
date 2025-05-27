#pragma once

#pragma once

// Define mask constants as macros before ctype_base.h sees them
#ifndef _U
#define _U 0x01 // upper
#endif
#ifndef _L
#define _L 0x02 // lower
#endif
#ifndef _N
#define _N 0x04 // digit
#endif
#ifndef _X
#define _X 0x08 // hex digit
#endif
#ifndef _S
#define _S 0x10 // space
#endif
#ifndef _P
#define _P 0x20 // printable
#endif
#ifndef _B
#define _B 0x40 // blank
#endif
#ifndef _C
#define _C 0x80 // control
#endif
