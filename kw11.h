//
// Created by palulukan on 7/8/20.
//

#ifndef KW11_H_81714460DDAC4006AE29AD6552F08F79
#define KW11_H_81714460DDAC4006AE29AD6552F08F79

#include "device.h"

#define CONFIG_KW11_HZ 60

bool kw11_init(void);

void kw11_destroy(void);

device_handle kw11_getHandle(void);


#endif //KW11_H_81714460DDAC4006AE29AD6552F08F79
