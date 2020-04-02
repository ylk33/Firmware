
#include "rw_uart.h"
#include "rw_uart_define.h"

/**
* daemon management function.
 */
__EXPORT int rw_uart_main(int argc, char *argv[]);

static bool rw_thread_should_exit = false;		/**< px4_uart exit flag */
static bool rw_uart_thread_running = false;		/**< px4_uart status flag */
static pthread_mutex_t mutex;                   //互斥信号量

int uart_read;

MSG_orb_sub msg_fd;
MSG_orb_data msg_data;
MSG_orb_pub msg_pd;


static int rw_uart_task;				/**< Handle of px4_uart task / thread */
static int rw_uart_init(void);
static int set_rw_uart_baudrate(const int fd, unsigned int baud);

void msg_orb_sub (void);
void msg_orb_data(void);
void msg_orb_unsub (void);
void wp_data_init(void);

/**
 * Mainloop of daemon.
 */
int rw_uart_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
        if (reason) {
                //printf("%s\n", reason);
        }

       //printf("usage: rw_uart {start|stop|status} [-p <additional params>]\n\n");
}

int set_rw_uart_baudrate(const int fd, unsigned int baud)
{
        int speed;

        switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
                //printf("ERR: baudrate: %d\n", baud);
                return -EINVAL;
        }

        struct termios uart_config;

        int termios_state;

        tcgetattr(fd, &uart_config); // 获取终端参数

        /* clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

        /* 无偶校验，一个停止位 */
        uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
        // CSTOPB 使用两个停止位，PARENB 表示偶校验, CRTSCTS 使用流控

        cfmakeraw(&uart_config);

         /* 设置波特率 */
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
                 //printf("ERR: %d (cfsetispeed)\n", termios_state);
                return false;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
                  //printf("ERR: %d (cfsetospeed)\n", termios_state);
                return false;
        }
        // 设置与终端相关的参数，TCSANOW 立即改变参数
        if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
                //printf("ERR: %d (tcsetattr)\n", termios_state);
                return false;
        }

        return true;
}


int rw_uart_init (void)
{
       //char *uart_name = "/dev/ttyS3";
       char *uart_name = "/dev/ttyS1";
       int serial_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
       // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
        
		if (serial_fd < 0) {
                printf("failed to open port: %s\n", uart_name);
                return false;
        }
        //printf("Open the %s\n",uart_name);
        return serial_fd;
}


void msg_orb_sub (void)
{
    memset(&msg_fd, 0, sizeof(msg_fd));
    msg_fd.status_fd = orb_subscribe(ORB_ID(vehicle_status));

}

void msg_orb_data(void)
{  
    orb_copy(ORB_ID(vehicle_status), msg_fd.status_fd, &msg_data.status_data);
}

void msg_orb_unsub (void)
{   
     orb_unsubscribe(msg_fd.status_fd);
}

int rw_uart_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage("missing command");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (rw_uart_thread_running) {
					
                    printf("px4_uart already running\n");    /* this is not an error */          
                    return 0;
                }

                rw_thread_should_exit = false;         //定义一个守护进程
                rw_uart_task = px4_task_spawn_cmd("rw_uart",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT,        //调度优先级
                        PX4_STACK_ADJUSTED(5000),      //堆栈分配大小
                        rw_uart_thread_main,
                        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                rw_thread_should_exit = true;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (rw_uart_thread_running) {
					printf("running\n");

                }
                else {
					printf("not started\n");
                }

                return 0;
        }

        usage("unrecognized command");
        return 1;
}

static void *receive_loop(void *arg)
{
    uint8_t buffer[300] ={};
    px4_pollfd_struct_t fds[1] = {
        { .fd = uart_read, .events = POLLIN }
    };
    int nread = 0;
    int read_finish = 0;
    int remain =0;
    int find_type_finish =0;

    while(!rw_thread_should_exit){

      if (poll(&fds[0], 1, 20) > 0)
        {
          //usleep(1000);
          nread= read(uart_read, &buffer[remain], sizeof(buffer) - (size_t)remain);
          if (nread < 0) nread =0;
          for ( read_finish = 0; read_finish < (nread + remain); ) {
               if ((nread + remain - read_finish) < 7)  //wqk buffer中数据量至少读出针头和有效数据量来
					break;
			   if (buffer[read_finish] == '$'){
                    find_type_finish = find_r_type(&buffer[read_finish], msg_data, &msg_pd,nread + remain - read_finish);
                    if (find_type_finish == -1) {//wqk解析出错
						read_finish++; 
                    }
					else if(find_type_finish == -2){//wqk buffer中数据量不足以解析当前帧
					    break;
					}else if(find_type_finish == -3){
						uint16_t tmp_len=(uint16_t)buffer[read_finish+5] + ((uint16_t)buffer[read_finish+6]<<8);
					    read_finish +=tmp_len;
			        }else{
						
                        read_finish += find_type_finish;                      
                    }
                }
               else {
                   read_finish++;
               }
            }
            remain = nread + remain - read_finish;
            uint8_t buffer_move[300] = {};
            memcpy(buffer_move, &buffer[read_finish], (size_t)remain);
            memcpy(buffer, buffer_move, sizeof(buffer_move));
        }
    }
    return NULL;
}

void receive_start(pthread_t *thread)
{
    pthread_attr_t receiveloop_attr;
    pthread_attr_init(&receiveloop_attr);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&receiveloop_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MAX - 80;
    (void)pthread_attr_setschedparam(&receiveloop_attr, &param);

    pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(5000));
    pthread_create(thread, &receiveloop_attr, receive_loop, NULL);

    pthread_attr_destroy(&receiveloop_attr);
}


int rw_uart_thread_main(int argc, char *argv[])
{

        /*
                GPS1:/dev/ttyS0
                TEL1:/dev/ttyS1
                TEL2:/dev/ttyS2
                TEL4:/dev/ttyS3
         */
         //打开串口文件，设置文件属性
         uart_read = rw_uart_init();  
         //设置串口属性1-8-1 波特率
         if (false == set_rw_uart_baudrate(uart_read, COM_PORT_BAUDRATE)) {  
                 //printf("set_rw_uart_baudrate is failed\n");
                 return -1;
         }
         
        //MSG_orb_sub msg_fd;  //wqk订阅vehicle_status主题
        msg_orb_sub();

        //MSG_orb_pub msg_pd;  //wqk 初始化发布主题 follow_target
        memset(&msg_pd, 0, sizeof(msg_pd));


        pthread_mutex_init(&mutex, NULL);  //初始化互斥信号量

        pthread_t receive_thread;

        rw_uart_thread_running = true;     

        receive_start(&receive_thread);

        
        while (!rw_thread_should_exit)
        {
            {
                pthread_mutex_lock(&mutex);
                memset(&msg_data, 0, sizeof(msg_data));
                msg_orb_data();
                pthread_mutex_unlock(&mutex);
                //msg_pack_send(msg_data, &msg_pd);
                usleep(200000);
            }
        }

        msg_orb_unsub();
        printf("[rw_uart] exiting\n");
        rw_uart_thread_running = false;
        close(uart_read);
        printf("uart close\n");

        fflush(stdout);
        return 0;
}
