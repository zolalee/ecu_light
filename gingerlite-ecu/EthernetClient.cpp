/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
//#include "main.h"
#include "EthernetClient.h"
#include "ring_buffer.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>

#define RXBUF_SIZE        				1024
//#define ADDR "192.168.1.100"
//#define SERVERPORT 11411

using namespace std;

static uint8_t rxbuf[RXBUF_SIZE];
static ring_buffer rb1_recv;
static unsigned char connect_flag = 0;
int sock;
pthread_t recv_thread = -1;
void *tcp_receive_thread(void *param);

void *tcp_receive_thread(void *param)
{
	uint8_t get_msg[RXBUF_SIZE] = {0};
	uint16_t i;
	while(1)
	{
		int read_msg = read(sock, get_msg, RXBUF_SIZE);
                if(read_msg <= 0)
		{
			close(sock);
			connect_flag = 0;
			return (void *)0;
		}
		for(i = 0; i < read_msg; i++)
		{
			rb_push_insert(&rb1_recv, get_msg[i]);
		}
	}
}

int EthernetClient::connect(unsigned char a,unsigned char b,unsigned char c,unsigned char d, uint16_t port)
{
	struct sockaddr_in serv_addr;
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if(sock == -1){
		return -1;
	}
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	//serv_addr.sin_addr.s_addr = inet_addr(ADDR);
	serv_addr.sin_addr.s_addr = (a | (b << 8) | (c << 16) | (d << 24));
	//serv_addr.sin_port = htons(SERVERPORT);
	serv_addr.sin_port = htons(port);
	if(::connect(sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1) {
		cout << "connect error\n";
		close(sock);
		return -1;
	}
	else {
		connect_flag = 1;
		cout << "connected ...\n" << endl;
	}
	pthread_create(&recv_thread, NULL, tcp_receive_thread, NULL);
	return 0;
}

int EthernetClient::availableForWrite(void)
{
	return 0;
}

int16_t EthernetClient::write(uint8_t b)
{
	return 0;
}

int16_t EthernetClient::write(const uint8_t *buf, int16_t size)
{
	int len;
	len = ::write(sock, buf, size);
	return len;
}

int EthernetClient::available()
{
	return rb_full_count(&rb1_recv);
}

int EthernetClient::read(uint8_t *buf, int16_t size)
{
	return 0;
}

int EthernetClient::peek()
{
	return 0;
}

int EthernetClient::read()
{
	if(!this->available())
		return -1;
	return rb_remove(&rb1_recv);
}

void EthernetClient::flush()
{
	rb_reset(&rb1_recv);
	return;
}

void EthernetClient::begin()
{
	rb_init(&rb1_recv, sizeof(rxbuf), rxbuf);
	return;
}

void EthernetClient::stop()
{
	return;
}

uint8_t EthernetClient::connected()
{
	if(connect_flag == 0)
		return 0;
	else
		return 1;
}

uint8_t EthernetClient::status()
{
	return 0;
}

