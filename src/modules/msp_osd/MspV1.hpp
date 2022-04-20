
#pragma once

class MspV1
{
public:
	MspV1(int fd);
	int GetMessageSize(int message_type);
	bool Send(const uint8_t message_id, const void *payload);

private:
	int _fd;
};

