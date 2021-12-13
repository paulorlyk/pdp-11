cmake_minimum_required(VERSION 3.16)
project(pdp_11 C)

set(CMAKE_C_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wformat-overflow=2 -Wformat=2 -pedantic")

add_executable(pdp_11
        main.c
        cpu.h
        cpu.c
        mem.h
        mem.c
        mmu.h
        mmu.c
        log.h
        rk11.h
        rk11.c
        task_scheduler.h
        task_scheduler.c
        device.h
        device.c
        dl11.h
        dl11.c
        kw11.h
        kw11.c)
