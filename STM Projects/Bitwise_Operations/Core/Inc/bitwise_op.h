#ifndef SRC_BITWISE_OP_H
#define SRC_BITWISE_OP_H

#include <stdint.h>

uint8_t set_wr(uint8_t byte, uint8_t wr);
uint8_t set_sector(uint8_t byte, int sector);
uint8_t set_data(uint8_t byte, uint8_t data);
uint8_t set_parity(uint8_t byte);

#endif /* SRC_BITWISE_OP_H */
