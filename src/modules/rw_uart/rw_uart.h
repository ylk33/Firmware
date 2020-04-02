#ifndef RW_UART_H
#define RW_UART_H

#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>

#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/follow_target.h>

typedef struct {
    uint8_t name;
    uint8_t command;
}MSG_type;

typedef struct {
     int status_fd;
}MSG_orb_sub;

typedef struct {
     orb_advert_t follow_target_pd;
}MSG_orb_pub;

typedef struct {
    struct vehicle_status_s status_data;
}MSG_orb_data;

#pragma pack(1)
typedef struct {
    char head[5];
    uint16_t buflen;
    uint8_t command;
    uint8_t command_re;
    uint8_t failed;
    uint16_t CRC_test;
}EXYF_FOLLOW_ACK;


extern int uart_read;

extern bool check_command_repeat(const uint8_t *buffer, MSG_type msg_type);//

extern bool compare_buffer_n(const uint8_t *buffer1, const uint8_t *buffer2, int n); //

extern uint16_t check_crc(const uint8_t *buffer, uint8_t buflen);//

extern void msg_pack_send(MSG_orb_data msg_data, MSG_orb_pub *msg_pd);//

extern int find_r_type(const uint8_t *buffer, const MSG_orb_data msg_data, MSG_orb_pub *msg_pd, int buff_remain);//

extern void follow_ack_pack_send(uint8_t failed);//




#endif // RW_UART_H
