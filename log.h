//
// Created by palulukan on 6/11/20.
//

#ifndef LOG_H_AB6E3E228F03409499A05D2324D580A3
#define LOG_H_AB6E3E228F03409499A05D2324D580A3

#include <stdio.h>

#define DEBUG(...) (fprintf(stderr, __VA_ARGS__), fputc('\n', stderr))

#endif //LOG_H_AB6E3E228F03409499A05D2324D580A3
