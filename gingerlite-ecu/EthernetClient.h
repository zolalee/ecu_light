#include "stdint.h"

class EthernetClient {
public:
	EthernetClient() { }

	uint8_t status();
	virtual int connect(unsigned char a,unsigned char b,unsigned char c,unsigned char d, uint16_t port);
	virtual int availableForWrite(void);
	virtual int16_t write(uint8_t);
	virtual int16_t write(const uint8_t *buf, int16_t size);
	virtual int available();
	virtual int read();
	virtual int read(uint8_t *buf, int16_t size);
	virtual int peek();
	virtual void flush();
	virtual void begin();
	virtual void stop();
	virtual uint8_t connected();
};
