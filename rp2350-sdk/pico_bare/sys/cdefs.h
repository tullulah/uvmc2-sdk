/* sys/cdefs.h for the debug cart's bare build (see ../pico/config_autogen.h). With a toolchain
 * that has newlib this is only a step to newlib's own; with one that has not (Homebrew's
 * arm-none-eabi-gcc), these are all the pico-sdk's headers want from it. */
#if __has_include_next(<sys/cdefs.h>)
#include_next <sys/cdefs.h>
#else
#ifndef PICO_BARE_SYS_CDEFS_H
#define PICO_BARE_SYS_CDEFS_H
#define __CONCAT1(x, y) x ## y
#define __CONCAT(x, y)  __CONCAT1(x, y)
#define __STRING(x)     #x
#define __XSTRING(x)    __STRING(x)
#define __always_inline __inline__ __attribute__((__always_inline__))
#define __noinline      __attribute__((__noinline__))
#define __unused        __attribute__((__unused__))
#define __used          __attribute__((__used__))
#endif
#endif
/* the structs' size checks, whichever assert.h the game's include path brings (the AAE ports
   have a stub one, Homebrew's toolchain none) */
#ifndef static_assert
#define static_assert _Static_assert
#endif
