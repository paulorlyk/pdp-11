//
// Created by palulukan on 6/26/20.
//

#ifndef DL11_H_A90EAA5DD3AB48D88F039CF0B9D7A071
#define DL11_H_A90EAA5DD3AB48D88F039CF0B9D7A071

#include "device.h"

typedef void *DL11;

typedef void (* dl11_tx_cb)(char ch);

// baseAddr must be aligned to CPU word size.
// baseVector can't have any of it's first 3 bits set.
// tx callback must not be NULL.
DL11 dl11_init(ph_addr baseAddr, cpu_word baseVector, dl11_tx_cb tx);

void dl11_destroy(DL11 dev);

device_handle dl11_getHandle(DL11 dev);

// Returns false if receiver is busy by processing previous character, true otherwise.
bool dl11_rx(DL11 dev, char ch);

#endif //DL11_H_A90EAA5DD3AB48D88F039CF0B9D7A071
