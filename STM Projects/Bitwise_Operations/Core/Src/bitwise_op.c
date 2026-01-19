#include "bitwise_op.h"

uint8_t set_wr(uint8_t byte, uint8_t wr) {
    byte &= ~(1 << 0);
    byte |= (wr & 1) << 0;
    return byte;
}


uint8_t set_sector(uint8_t byte, int sector) {
    static int lastSector = 0;
    if (sector == -1) {
        lastSector = (lastSector + 1) % 4;
        sector = lastSector;
    }
    byte &= ~(3 << 1);
    byte |= (sector & 3) << 1;
    return byte;
}

uint8_t set_data(uint8_t byte, uint8_t data) {
    byte &= ~(15 << 3);
    byte |= (data & 15) << 3;
    return byte;
}

uint8_t set_parity(uint8_t byte) {
    uint8_t count = 0;
    uint8_t temp = byte;
    while (temp) {
        count += temp & 1;
        temp >>= 1;
    }
    if (count % 2 != 0) {
        byte |= (1 << 7);
    } else {
        byte &= ~(1 << 7);
    }
    return byte;
}
