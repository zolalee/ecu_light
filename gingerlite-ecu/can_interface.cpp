#include <stdio.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <unistd.h>
#include "can_interface.h"

#include <string>
#include <string.h>

#define EPOLL_SIZE 1024

pthread_t can_thread = -1;
static pthread_mutex_t recive_lock;

void *can_recive_thread(void *param);
int can_port_init(char *can_name);
int add_epoll_fd(int sock_fd);

int can_init(can_device_t *device, char *can_name, callback_fn rec_cb)
{
    /*bind can 端口*/
    device->sock_fd = can_port_init(can_name);
    if (device->sock_fd < 0)
    {
        printf(">>: can socket create failed!\r\n");
        return -1;
    }
 
    /*开启epoll事件监听*/
    device ->epoll_fd = add_epoll_fd(device->sock_fd);
    if (device->epoll_fd < 0)
    {
        close(device->sock_fd);
        printf(">>: can socket epoll create failed!\r\n");
        return -1;
    }
    /**注册接收回调函数*/
    if (rec_cb)
    {
        device->recive_callback = rec_cb;
    }
 
    printf(">>: can init success, sock_fd:  %d,   epoll_fd: %d !\r\n", device->sock_fd, device->epoll_fd);
    return 1;
}

void can_open(can_device_t *candevice)
{
    /*创建接收进程*/
    pthread_mutex_init(&recive_lock, NULL);
    pthread_create(&can_thread, NULL, can_recive_thread, candevice);
    printf(">>: epoll thread fd: %d\r\n", (int)can_thread);
}

void can_close(can_device_t *candevice)
{
    if (can_thread > 0)
    {
        void *recycle;
        /*回收线程资源  */
        pthread_join(can_thread, &recycle);
        pthread_mutex_destroy(&recive_lock);
        printf(">>: epoll thread destroy!\r\n");
    }
    /*关闭epoll*/
    if (candevice->epoll_fd > 0)
    {
        close(candevice->epoll_fd);
        printf(">>: epoll  close!\r\n");
    }
    /*关闭CAN外设*/
    if (candevice->sock_fd > 0)
    {
        close(candevice->sock_fd);
        printf(">>:  can socket thread destroy!\r\n");
    }
    printf(">>:  can device close!\r\n");
}

int can_transimit(can_device_t *candevice, can_obj_t *obj_buff, int obj_count)
{
    int tran_count_ret = 0;
    struct can_frame tx_frame;
 
    bzero(&tx_frame, sizeof(tx_frame));
    tx_frame.can_id = obj_buff->id;
    tx_frame.can_dlc = obj_buff->data_len;
    memcpy(tx_frame.data, obj_buff->data_buf, obj_buff->data_len);
 
    tran_count_ret = write(candevice->sock_fd, &tx_frame, sizeof(tx_frame));
    if (tran_count_ret != sizeof(tx_frame))
    {
        printf(">>:  transimit failed!\r\n");
    }
    return tran_count_ret;
}

int can_recive(can_device_t *candevice, can_obj_t *obj_buff, int obj_count)
{
    int recive_count_ret = 0;
    pthread_mutex_lock(&recive_lock);
 
    pthread_mutex_unlock(&recive_lock);
    return recive_count_ret;
}

int can_port_init(char *can_name)
{
    int can_fd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    /*创建套接字*/
    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd < 0)
    {
        return -1;
    }
    /*指定 can 设备*/
    strcpy(ifr.ifr_name, can_name);
    ioctl(can_fd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
 
    /*关闭回环模式*/
    int loopback = 0;   /* 0 = disabled, 1 = enabled (default) */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
 
    /*关闭自收自发*/
    int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
    setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
 
    /*将套接字与 can0 绑定*/
    bind(can_fd, (struct sockaddr *)&addr, sizeof(addr));
    return can_fd;
}

int add_epoll_fd(int sock_fd)
{
    int epfd;
    struct epoll_event event;
 
    /*创建epoll模型*/
    epfd = epoll_create(EPOLL_SIZE);
    if (epfd < 0)
    {
        return -1;
    }
 
    /*监听sock_fd 的可读事件*/
    event.data.fd = sock_fd;
    event.events = EPOLLIN;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, sock_fd, &event) < 0)
    {
        return -1;
    }
 
    return epfd;
}

void *can_recive_thread(void *param)
{
    int i, nfds;
    int timeout = 2;
    uint64_t nbytes;
    struct can_frame rx_frame;
    struct epoll_event events[EPOLL_SIZE];
    can_device_t *can_device_temp = (can_device_t *)param;
 
    while (1)
    {
        nfds = epoll_wait(can_device_temp->epoll_fd, events, EPOLL_SIZE, timeout);
        if (nfds < 0)
        {
            //   printf(">>:  epoll wait error!\r\n");
        }
        for (int i = 0; i < nfds; i++)
        {
            if (events[i].events & EPOLLIN)
            {
                nbytes = read(events[i].data.fd, &rx_frame, sizeof(rx_frame));
                if (nbytes > 0)
                {
                    if (can_device_temp->recive_callback)
                    {
                        can_device_temp->recive_callback(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
                    }
                }
            }
        }
    }
}

