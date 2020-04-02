
#include"rw_uart.h"
#include"rw_uart_define.h"


void follow_ack_pack_send(uint8_t failed){
    EXYF_FOLLOW_ACK follow_ack;
    uint8_t send_message[12];
    follow_ack.head[0] = '$';
    follow_ack.head[1] = 'E';
    follow_ack.head[2] = 'X';
    follow_ack.head[3] = 'Y';
    follow_ack.head[4] = 'F';
	if(failed ==0){	         //成功接收
        follow_ack.command = 'R';
		follow_ack.command_re ='C';	
		follow_ack.failed = 'T';		
	}else if(failed == 1){   //接收失败
	    follow_ack.command = 'R';
		follow_ack.command_re ='C';	
		follow_ack.failed = 'F';	
	}else if(failed == 2){   //未处于follow模式
	    follow_ack.command = 'N';
		follow_ack.command_re ='I';	
		follow_ack.failed = 'F';
	}else{                    //默认接收空白
		follow_ack.command = 'R';
		follow_ack.command_re ='C';	
		follow_ack.failed = 'N';
	}
    follow_ack.buflen = 12;
    memcpy(send_message, &follow_ack, sizeof(EXYF_FOLLOW_ACK));
    uint16_t crc = check_crc(send_message, 12);
    send_message[10] = (uint8_t)(crc & 0x00ff);
    send_message[11] = (uint8_t)((crc & 0xff00)>>8);
    
    write(uart_read, send_message, sizeof(EXYF_FOLLOW_ACK));
}

