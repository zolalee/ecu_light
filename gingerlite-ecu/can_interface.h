#ifndef __CAN_INTERFACE_H__
#define __CAN_INTERFACE_H__

#include <stdint.h>
#include <stdlib.h>

#define CAN_RX_MSG_BUF          ((int)128)
#define CAN_TX_MSG_BUF          ((int)128)
#define CAN_MSG_LEN             ((int)8)   

typedef void (*callback_fn)(int,uint8_t*,int);

typedef struct 
{
   uint16_t id;
   uint16_t time_stamp;
   uint8_t  time_flag;
   uint8_t  send_type;
   uint8_t  remote_type;
   uint8_t  extern_flag;
   uint8_t  data_len;
   uint8_t  data_buf[CAN_MSG_LEN];
   uint8_t  reserved[3];   
}can_obj_t;

typedef struct 
{
  int sock_fd;
  int epoll_fd;
  callback_fn recive_callback;
}can_device_t;

int can_init(can_device_t* device, char *can_name,callback_fn rec_cb);
void can_open(can_device_t* candevice);
void can_close(can_device_t* candevice);
int can_transimit(can_device_t *candevice, can_obj_t *obj_buff, int obj_count);
#endif

