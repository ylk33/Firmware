#include "rw_uart.h"
#include "rw_uart_define.h"
#include <uORB/topics/follow_target.h>

static double lat_save =0;
static double lon_save =0;


void msg_pack_send( MSG_orb_data msg_data, MSG_orb_pub *msg_pd)
{
    uint8_t send_message[3]; //wqk发送*号+换行,证明活着
	send_message[0]=0x2a;
	send_message[1]=0x0d;
	send_message[2]=0x0a;
    write(uart_read, send_message, sizeof(send_message));

}


void msg_orb_param_pro(const uint8_t *buffer, MSG_orb_pub *msg_pd, MSG_orb_data msg_data,
                                     MSG_type msg_type)
{
    switch (msg_type.name) {
    case MSG_NAME_EXYF:
        switch (msg_type.command) {
        case EXYF_COMM_FOLLOW:
            if (msg_data.status_data.nav_state != 19){  //NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
               //printf("Flight mode is not follow\n");
                follow_ack_pack_send(2);
            }
            else {
                struct follow_target_s follow_target_data ={};
                memset(&follow_target_data, 0, sizeof(follow_target_data));
//                for(int i =0; i < 48; i++){
//                printf("Buffer[%d] is %x\n", i, buffer[i]);
//                }
                memcpy(&follow_target_data.lat, &buffer[9], sizeof(double));
                memcpy(&follow_target_data.lon, &buffer[17], sizeof(double));
                memcpy(&follow_target_data.alt, &buffer[25], sizeof(float));
                memcpy(&follow_target_data.vy, &buffer[29], sizeof(float));
                memcpy(&follow_target_data.vx, &buffer[33], sizeof(float));
                memcpy(&follow_target_data.vz, &buffer[37], sizeof(float));
                follow_target_data.timestamp = hrt_absolute_time();
                if (msg_pd->follow_target_pd != NULL){
                        orb_publish(ORB_ID(follow_target), msg_pd->follow_target_pd, &follow_target_data);
                       //printf("Passing 2_1\n");
                }
                else{
                        msg_pd->follow_target_pd = orb_advertise(ORB_ID(follow_target), &follow_target_data);
                       //printf("Passing 2_2\n");
                }
                follow_ack_pack_send(0);//wqk接收成功
            }
            break;
			
        default:
            break;
        }
        break;
		
    default:
        break;
    }
}
//解析数据，成功返回该帧数据长度；解析失败，返回-1；buffer中剩余数据量不足以解析当前数据帧，返回-2
int find_r_type(const uint8_t *buffer,const MSG_orb_data msg_data, MSG_orb_pub *msg_pd, int buff_remain)
{
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));	
    char *name = "$EXYF";

    if (compare_buffer_n(buffer, (uint8_t*)name, 5)){
       //printf("Passing EXYF\n");
	   
	    //buffer中剩余数据量不足以解析当前数据帧，返回-2
        uint8_t buflen;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        if(buflen > buff_remain)
			return -2;
		
		msg_type.name =MSG_NAME_EXYF;
		msg_type.command = buffer[7];
		uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
		if (check_command_repeat(buffer, msg_type) && buffer[buflen -1] == 0x3f){
			//printf("Check passed\n");
			msg_orb_param_pro(buffer, msg_pd, msg_data, msg_type);
			return buflen;
		 }else{
			follow_ack_pack_send(1);//接收失败，未定义帧			
			return -3;
		}
    }
	
    return -1;
}
