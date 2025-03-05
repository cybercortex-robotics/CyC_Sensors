#ifndef STREAM_H__
#define STREAM_H__

#include "Print.h"
#include <memory>

//static const size_t RS232_PORTNR = 32;
//static const char* const comports[RS232_PORTNR] = {
//	"\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
//	"\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
//	"\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
//	"\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16",
//	"\\\\.\\COM17", "\\\\.\\COM18", "\\\\.\\COM19", "\\\\.\\COM20",
//	"\\\\.\\COM21", "\\\\.\\COM22", "\\\\.\\COM23", "\\\\.\\COM24",
//	"\\\\.\\COM25", "\\\\.\\COM26", "\\\\.\\COM27", "\\\\.\\COM28",
//	"\\\\.\\COM29", "\\\\.\\COM30", "\\\\.\\COM31", "\\\\.\\COM32"
//};

class Stream : public Print {
public:
	Stream(const char* devName);
	Stream();

	virtual ~Stream();

	bool isConnected()
	{
		return m_IsConnected;
	}

	bool available();

	size_t write(uint8_t byte);
	size_t write(uint8_t* byte, size_t num_of_bytes);

	uint8_t read()
	{
		return m_recvdByte;
	}

	void begin(uint16_t baud_rate);

private:
	void close_comm();
	bool open_comm();

	struct SData;
	std::unique_ptr<SData> m_data;

	bool m_IsAvailable;
	bool m_IsConnected;
	uint8_t m_recvdByte;
	uint16_t m_baudRate;
	std::string m_devName;
};

//typedef Stream Serial;
extern Stream Serial;

#endif // STREAM__
