#include "rw_uart.h"
#include "rw_uart_define.h"

bool compare_buffer_n(const uint8_t *buffer1, const uint8_t *buffer2, int n){
    bool equal = true;
    for (int i = 0; i < n; ++i) {
        equal = equal && (*(buffer1+ i) == *(buffer2 + i));
    }
    return equal;
}

bool check_command_repeat(const uint8_t *buffer, MSG_type msg_type)
{
    bool check_ok = false;
    switch (msg_type.name) {
    case MSG_NAME_EXYF:
         check_ok = (buffer[7] == buffer [8]);
        break;
		
    default:
        break;
    }
    return check_ok;
}


uint16_t check_crc(const uint8_t *buffer, uint8_t buflen)
{
    uint16_t crc_table[] ={0x0000, 0xcc01, 0xd801, 0x1400, 0xf001, 0x3c00, 0x2800, 0xe401,
                                  0xa001, 0x6c00, 0x7800, 0xb401, 0x5000, 0x9c01, 0x8801, 0x4400};
    uint16_t w_crc = 0xffff;
    uint8_t chChar = 0;
    for (int i =0; i < buflen -2; i++){
        chChar = buffer[i];
        w_crc = (uint16_t)(crc_table[(chChar ^ w_crc) & 15] ^ (w_crc >>4));
        w_crc = (uint16_t)(crc_table[((chChar>>4) ^ w_crc) & 15] ^ (w_crc >>4));
    }
    //printf("crc check is %x\n", w_crc);
    return w_crc;
}
