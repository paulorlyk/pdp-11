cmake_minimum_required(VERSION 3.16)
project(pdp_11 C)

set(CMAKE_C_STANDARD 11)

add_executable(pdp_11
        main.c
        cpu.h
        cpu.c
        mem.h
        mem.c
        log.h
        rk11.h
        rk11.c
        task_scheduler.h
        task_scheduler.c
        device.h
        device.c
        dl11.h
        dl11.c
        )
