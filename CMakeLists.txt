cmake_minimum_required(VERSION 3.16)
project(pdp_11 C)

set(CMAKE_C_STANDARD 11)

add_executable(pdp_11 main.c cpu.h cpu.c mem.c mem.h log.h)
