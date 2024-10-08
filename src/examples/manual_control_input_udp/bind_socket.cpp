#include "bind_socket.hpp"
#include <sys/socket.h>
#include <string.h>
#include <string>
#include <px4_platform_common/log.h>


constexpr struct addrinfo hints = {
	.ai_flags = AI_PASSIVE,
	.ai_family = AF_UNSPEC,
	.ai_socktype = SOCK_DGRAM,
	.ai_protocol = 0,
	.ai_addrlen = 0,
	.ai_addr = nullptr,
	.ai_canonname = nullptr,
	.ai_next = nullptr,
};


int bind_socket(in_port_t port)
{
	const std::string port_str = std::to_string(port);

	struct addrinfo *res;

	const int err = getaddrinfo(NULL, port_str.c_str(), &hints, &res);

	if (err != 0) {
		PX4_ERR("Failed to get address: %s\n", gai_strerror(err));
		return -1;
	}

	const int socketfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);

	if (socketfd == -1) {
		PX4_ERR("Failed to create socket: %s", strerror(errno));
		return -1;
	}

	if (bind(socketfd, res->ai_addr, res->ai_addrlen) == -1) {
		PX4_ERR("Failed to bind address: %s", strerror(errno));
		return -1;
	}

	freeaddrinfo(res);

	return socketfd;
}
