/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file client.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#ifdef __PX4_WINDOWS
#include <netinet/in.h>
#include <windows.h>
#else
#include <sys/un.h>
#endif
#include <unistd.h>

#include <string>
#include <system_error>
#include <thread>

#include <px4_platform_common/log.h>
#include "client.h"

namespace px4_daemon
{

namespace
{

#ifdef __PX4_WINDOWS
// SHUT_WR is the POSIX value of the half-close-send constant; on Winsock,
// the equivalent is SD_SEND. Use a portable alias so the relay code below is
// platform-neutral.
static constexpr int kShutWr = SD_SEND;
#else
static constexpr int kShutWr = SHUT_WR;
#endif

#ifdef __PX4_WINDOWS
bool ends_with_exe_suffix(const std::string &arg)
{
	if (arg.size() < 4) {
		return false;
	}

	const std::string suffix = arg.substr(arg.size() - 4);
	return suffix == ".exe" || suffix == ".EXE"
	       || suffix == ".Exe" || suffix == ".eXe"
	       || suffix == ".exE" || suffix == ".EXe"
	       || suffix == ".eXE" || suffix == ".ExE";
}
#endif

// Reads up to `n` bytes from this process's standard input. Returns 0 on EOF
// and -1 on error. Wraps the platform-specific stdin-read primitive so the
// forwarding loop below stays platform-neutral.
static ssize_t read_local_stdin(char *buf, size_t n)
{
#ifdef __PX4_WINDOWS
	HANDLE stdin_h = GetStdHandle(STD_INPUT_HANDLE);

	if (stdin_h == INVALID_HANDLE_VALUE || stdin_h == nullptr) {
		return -1;
	}

	DWORD got = 0;

	if (!ReadFile(stdin_h, buf, (DWORD)n, &got, nullptr)) {
		return -1;
	}

	return (ssize_t)got;
#else
	return read(STDIN_FILENO, buf, n);
#endif
}

// Worker that reads bytes from this process's stdin and forwards them over
// the socket so the daemon can hand them to the running module's stdin pipe.
// Stops on stdin EOF (half-closes the socket's send side, signalling EOF to
// the daemon) or on any send() error (typically the daemon already closed
// the connection because the command finished).
static void stdin_forward_loop(socket_handle_t fd)
{
	char buffer[256];

	while (true) {
		const ssize_t bytes_read = read_local_stdin(buffer, sizeof(buffer));

		if (bytes_read <= 0) {
			break;
		}

		const char *buf = buffer;
		ssize_t remaining = bytes_read;

		while (remaining > 0) {
			const int n_sent = send(fd, buf, (int)remaining, 0);

			if (n_sent <= 0) {
				// Daemon closed the connection (command finished). Done.
				return;
			}

			buf += n_sent;
			remaining -= n_sent;
		}
	}

	// Local stdin reached EOF — half-close the send side so the daemon's
	// stdin relay sees EOF on its recv() and drains the module's stdin pipe.
	shutdown(fd, kShutWr);
}

} // namespace

Client::Client(int instance_id) :
	_fd(invalid_socket_handle),
	_instance_id(instance_id)
{}

int
Client::process_args(const int argc, const char **argv)
{
#ifdef __PX4_WINDOWS
	const uint16_t port = get_socket_port(_instance_id);

	_fd = socket(AF_INET, SOCK_STREAM, 0);

	if (_fd == invalid_socket_handle) {
		PX4_ERR("error creating socket");
		return -1;
	}

	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons(port);

	if (connect(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("error connecting to 127.0.0.1:%u: %s", port, strerror(errno));
		return -1;
	}

	// No SO_RCVTIMEO: match POSIX/AF_UNIX behaviour where the client blocks
	// in recv() until the daemon either replies or shuts the connection
	// down. A short timeout here (the previous 5 s default) would fire
	// before commands that legitimately take longer to return — most
	// notably `commander takeoff`, whose lockstep wait_for_vehicle_command_reply
	// can sit idle for the full 5 s acknowledgement window followed by a
	// second 5 s wait for the arming reply. Letting the OS deliver an EOF
	// or RST from the server is the correct termination signal here.
#else
	std::string sock_path = get_socket_path(_instance_id);

	_fd = socket(AF_UNIX, SOCK_STREAM, 0);

	if (_fd < 0) {
		PX4_ERR("error creating socket");
		return -1;
	}

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);

	if (connect(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("error connecting to socket: %s", strerror(errno));
		return -1;
	}

#endif

	int ret = _send_cmds(argc, argv);

	if (ret != 0) {
		PX4_ERR("Could not send commands");
		return -3;
	}

	// Forward this process's stdin to the daemon while we wait for the
	// command's stdout. The thread is detached and will exit when the local
	// stdin closes or the daemon hangs up the socket — process exit reaps it.
	// Best-effort: commands that don't read stdin are unaffected; interactive
	// ones simply won't see keystrokes if thread creation fails.
	try {
		std::thread(stdin_forward_loop, _fd).detach();

	} catch (const std::system_error &) {
		// intentionally swallowed: stdin relay is best-effort. If the OS cannot
		// spawn the helper thread, interactive commands lose keystroke
		// forwarding but the command itself still runs to completion.
		(void)0;
	}

	return _listen();
}

int
Client::_send_cmds(const int argc, const char **argv)
{
	std::string cmd_buf;

	for (int i = 0; i < argc; ++i) {
		std::string arg = argv[i];

#ifdef __PX4_WINDOWS

		// Client executables are real .exe files on Windows, not POSIX
		// symlinks. The pxh command namespace remains extensionless
		// ("commander", "shutdown", ...), so strip only argv[0].
		if (i == 0 && ends_with_exe_suffix(arg)) {
			arg.resize(arg.size() - 4);
		}

#endif

		cmd_buf += arg;

		if (i + 1 != argc) {
			// TODO: Use '\0' as argument separator (and parse this server-side as well),
			// so (quoted) whitespace within arguments doesn't get lost.
			cmd_buf += " ";
		}
	}

	// Last byte is 'isatty'.
	cmd_buf.push_back(isatty(STDOUT_FILENO));

	size_t n = cmd_buf.size();
	const char *buf = cmd_buf.data();

	while (n > 0) {
		// send() instead of write() so the same code path works with AF_UNIX
		// on POSIX and AF_INET SOCKETs on Windows — write() does not operate
		// on WinSock SOCKET handles.
		int n_sent = send(_fd, buf, n, 0);

		if (n_sent < 0) {
			PX4_ERR("send() failed: %s", strerror(errno));
			return -1;
		}

		n -= n_sent;
		buf += n_sent;
	}

	return 0;
}

int
Client::_listen()
{
	char buffer[1024];
	int n_buffer_used = 0;

	// The response ends in {0, retval}. So when we detect a 0, or a 0 followed
	// by another byte, we don't output it yet, until we know whether it was
	// the end of the stream or not.
	while (true) {
		int n_read = recv(_fd, buffer + n_buffer_used, sizeof buffer - n_buffer_used, 0);

		if (n_read < 0) {
#ifdef __PX4_WINDOWS
			const int wsa_err = WSAGetLastError();

			// WSAECONNRESET / WSAESHUTDOWN can fire instead of a clean
			// recv() == 0 if the daemon's closesocket() races our recv()
			// (the OS already delivered the bytes — TCP just abandoned the
			// graceful FIN). Treat it as end-of-stream so we still pick up
			// the {0, retval} sentinel that was already buffered.
			if (wsa_err == WSAECONNRESET || wsa_err == WSAESHUTDOWN) {
				if (n_buffer_used == 2) {
					return buffer[1];
				}

				return -1;
			}

			PX4_ERR("unable to read from socket: WSA error = %d", wsa_err);
#else

			// ECONNRESET / EPIPE can fire on AF_UNIX too if the daemon
			// closes the socket between sending the {0, retval} sentinel
			// and our recv() picking up the EOF. The bytes have already
			// been delivered into our kernel buffer, so honour the
			// sentinel rather than reporting a spurious socket error.
			if (errno == ECONNRESET || errno == EPIPE) {
				if (n_buffer_used == 2) {
					return buffer[1];
				}

				return -1;
			}

			PX4_ERR("unable to read from socket: %s", strerror(errno));
#endif
			return -1;

		} else if (n_read == 0) {
			if (n_buffer_used == 2) {
				return buffer[1];

			} else {
				// Missing return value at end of stream. Stream was abruptly ended.
				return -1;
			}

		} else {
			n_read += n_buffer_used;

			if (n_read >= 2 && buffer[n_read - 2] == 0) {
				// If the buffer ends in {0, retval}, keep it.
				fwrite(buffer, n_read - 2, 1, stdout);
				buffer[0] = 0;
				buffer[1] = buffer[n_read - 1];
				n_buffer_used = 2;

			} else if (n_read >= 1 && buffer[n_read - 1] == 0) {
				// If the buffer ends in a 0-byte, keep it.
				fwrite(buffer, n_read - 1, 1, stdout);
				buffer[0] = 0;
				n_buffer_used = 1;

			} else {
				fwrite(buffer, n_read, 1, stdout);
				n_buffer_used = 0;
			}
		}
	}
}

Client::~Client()
{
	if (_fd != invalid_socket_handle) {
#ifdef __PX4_WINDOWS
		closesocket(_fd);
#else
		close(_fd);
#endif
	}
}

} // namespace px4_daemon
