#pragma once
#include"Protocols.h"
#include<iostream>


struct FilterAttributes
{
	std::string ENB = "enabled";
	std::string STA = "start";
	std::string END = "end";
};

struct FilterConfig
{
	bool enabled;
	std::string start;
	std::string end;
};

struct ChannelProperties
{
	struct Attributes
	{
		std::string IDX = "id";
		std::string FMT = "format";
		std::string ENB = "enabled";
		std::string MRG = "merge";
		std::string RST = "resistance";
	};

	std::string CLK = "clock-rate";
	std::string ART = "abit-rate";
	std::string DRT = "dbit-rate";
	std::string FLT = "filter";
};

struct ChannelConfig
{
	unsigned int id;
	std::byte format; // 0 for CAN, 1 for CANFD
	bool enabled;
	bool resistance;
	bool merge;
	std::string clockRate;
	std::string abitRate;
	std::string dbitRate;
	FilterConfig filter;
};

enum CanProvider
{
	UNKNOWN = 0,
	ZLG,
};

class CANConfig {
public:
	CanProvider provider;
	ZCAN_DeviceType deviceType;
	unsigned int deviceIndex;
	std::string deviceName;
	std::vector<ChannelConfig> channels;  // Channels will be used.

	CANConfig();

	CANConfig(ZCAN_DeviceType deviceType);

	CANConfig& operator=(const CANConfig& other);

	bool configured();

	void addChannel(unsigned int channelIdx, FilterConfig filter,
		std::string abRate = "1000000", std::string dbRate = "5000000",
		std::string baudRate = "60000000");

private:

};



class ZCanHandMessage  {
	
public:
	ZCanHandMessage();
	ZCanHandMessage(ZCAN_DeviceType Type);

	bool stop();

	bool start();

	CANConfig getConfiguration();

	~ZCanHandMessage();

	ZCanHandMessage& operator=(const ZCanHandMessage& other)=delete;
	ZCanHandMessage(const ZCanHandMessage&) = delete;
	ZCanHandMessage(const ZCanHandMessage&&) = delete;

	bool send(ZCAN_FD_MSG* message, U32 Type, int32_t Card, int32_t Port);

	int Recive(ZCAN_FD_MSG* message,U32 Type, U32 Card, U32 Port);

	bool instantiated();

	bool isStarted();

	CANConfig _config;
	bool instanced;
	bool started;
	bool initDevice();
};



