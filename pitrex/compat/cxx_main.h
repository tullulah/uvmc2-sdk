/* cxx_main.h — in front of every C++ source of a PiTrex build (pitrex.mk, PITREX_CXX_SRCS).
 *
 * The build renames the game's main to uvm2_game_main, which the SDK's start-up calls as a C
 * function. In a C++ source a function of that name would be a C++ one, and the link would
 * not find it: this says it is the C one.
 */
#ifndef UVM2_PITREX_CXX_MAIN_H
#define UVM2_PITREX_CXX_MAIN_H
#ifdef __cplusplus
extern "C" int uvm2_game_main(int argc, char *argv[]);
#endif
#endif
