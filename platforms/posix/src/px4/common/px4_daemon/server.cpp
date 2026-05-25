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
 * @file server.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <pthread.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#ifdef __PX4_WINDOWS
#include <netinet/in.h>
#include <io.h>
#else
#include <sys/un.h>
#endif
#include <vector>

#include <px4_platform_common/log.h>

#include "pxh.h"
#include "server.h"

namespace px4_daemon
{

namespace
{

struct ClientThreadArgs {
	socket_handle_t client_fd;
	FILE *thread_stdout;
#ifdef __PX4_WINDOWS
	int stdout_read_fd;
#endif
};

// Platform-neutral wrappers for the half-close-send and stdin-pipe-create
// primitives so the relay code below can stay free of #ifdefs on the hot path.
#ifdef __PX4_WINDOWS
static constexpr int kShutWr = SD_SEND;
#else
static constexpr int kShutWr = SHUT_WR;
#endif

// Returns {read_fd, write_fd} for an anonymous pipe, or {-1, -1} on failure.
// Wraps CreatePipe()+_open_osfhandle() on Windows and pipe() on POSIX.
static std::pair<int, int> create_relay_pipe()
{
#ifdef __PX4_WINDOWS
	HANDLE r = nullptr;
	HANDLE w = nullptr;

	if (!CreatePipe(&r, &w, nullptr, 0)) {
		return {-1, -1};
	}

	int rfd = _open_osfhandle(reinterpret_cast<intptr_t>(r), O_RDONLY);
	int wfd = _open_osfhandle(reinterpret_cast<intptr_t>(w), 0);

	if (rfd < 0 || wfd < 0) {

		if (rfd >= 0) { ::close(rfd); } else { CloseHandle(r); }

		if (wfd >= 0) { ::close(wfd); } else { CloseHandle(w); }

		return {-1, -1};
	}

	return {rfd, wfd};
#else
	int fds[2];

	if (pipe(fds) != 0) {
		return {-1, -1};
	}

	return {fds[0], fds[1]};
#endif
}

#ifdef __PX4_WINDOWS
// Forwards bytes from the per-client anonymous stdout pipe to the client
// socket. Used only on Windows because Winsock SOCKETs are not CRT file
// descriptors and therefore cannot back a FILE* directly via fdopen() — the
// daemon writes to a CRT pipe and this thread copies it onto the socket.
// On POSIX the socket is fdopen()'d directly and no relay is needed.
static void stdout_relay_loop(int stdout_read_fd, socket_handle_t client_fd)
{
	char buffer[1024];

	while (true) {
		const ssize_t n_read = read(stdout_read_fd, buffer, sizeof(buffer));

		if (n_read <= 0) {
			break;
		}

		const char *buf = buffer;
		ssize_t remaining = n_read;

		while (remaining > 0) {
			const int n_sent = send(client_fd, buf, (int)remaining, 0);

			if (n_sent <= 0) {
				::close(stdout_read_fd);
				return;
			}

			buf += n_sent;
			remaining -= n_sent;
		}
	}

	::close(stdout_read_fd);
}
#endif // __PX4_WINDOWS

// Forwards bytes that arrive on the client socket after the cmd terminator
// into the daemon's anonymous stdin pipe. The pipe's read-end is dup2'd onto
// STDIN_FILENO for the duration of Pxh::process_line(), so the running module
// sees keystrokes typed in the client's terminal.
static void stdin_relay_loop(socket_handle_t client_fd, int stdin_write_fd, std::string prefill)
{
	auto write_all = [&](const char *buf, ssize_t n) -> bool {
		while (n > 0)
		{
			const int n_written = ::write(stdin_write_fd, buf, (unsigned)n);

			if (n_written <= 0) {
				return false;
			}

			buf += n_written;
			n -= n_written;
		}

		return true;
	};

	if (!prefill.empty()) {
		if (!write_all(prefill.data(), (ssize_t)prefill.size())) {
			::close(stdin_write_fd);
			return;
		}
	}

	char buffer[1024];

	while (true) {
		const int n_read = recv(client_fd, buffer, sizeof(buffer), 0);

		if (n_read <= 0) {
			break;
		}

		if (!write_all(buffer, n_read)) {
			break;
		}
	}

	// EOF/error: close the pipe write end so the module sees EOF on stdin.
	::close(stdin_write_fd);
}

// Process-wide mutex serializing the STDIN_FILENO redirect. Concurrent commands
// take turns owning the live console-stdin replacement; only one can hold it at
// a time. Interactive use is single-client, so contention is negligible in
// practice.
static pthread_mutex_t s_stdin_redirect_mutex = PTHREAD_MUTEX_INITIALIZER;

static ssize_t socket_read(socket_handle_t fd, char *buffer, size_t buffer_size)
{
#ifdef __PX4_WINDOWS
	const int n_read = recv(fd, buffer, (int)buffer_size, 0);

	if (n_read == SOCKET_ERROR) {
		return -1;
	}

	return n_read;
#else
	return read(fd, buffer, buffer_size);
#endif
}

static bool is_shutdown_command(const std::string &cmd)
{
	const std::string whitespace{" \t\r\n"};
	const std::size_t command_start = cmd.find_first_not_of(whitespace);

	if (command_start == std::string::npos) {
		return false;
	}

	const std::size_t command_end = cmd.find_first_of(whitespace, command_start);
	const std::size_t command_length = (command_end == std::string::npos) ? std::string::npos : command_end - command_start;

	if (cmd.compare(command_start, command_length, "shutdown") != 0) {
		return false;
	}

	// Only treat the bare "shutdown" command as the daemon-teardown shortcut.
	// If there are any positional arguments (e.g. "shutdown status"), fall
	// through to the normal command path so shutdown_main can reject them via
	// its usage banner -- otherwise an accidental "shutdown <subcommand>" would
	// silently kill the daemon.
	if (command_end == std::string::npos) {
		return true;
	}

	return cmd.find_first_not_of(whitespace, command_end) == std::string::npos;
}

} // namespace

#ifdef __PX4_WINDOWS
static SOCKET poll_socket(socket_handle_t fd)
{
	return fd;
}
#else
static int poll_socket(int fd)
{
	return fd;
}
#endif

Server *Server::_instance = nullptr;

Server::Server(int instance_id)
	: _mutex(PTHREAD_MUTEX_INITIALIZER),
	  _instance_id(instance_id)
{
	_instance = this;
}

Server::~Server()
{
	_instance = nullptr;
}

int
Server::start()
{
#ifdef __PX4_WINDOWS
	const uint16_t port = get_socket_port(_instance_id);

	_fd = socket(AF_INET, SOCK_STREAM, 0);

	if (_fd == invalid_socket_handle) {
		PX4_ERR("error creating socket");
		return -1;
	}

	// Avoid EADDRINUSE when a prior instance left the port in TIME_WAIT.
	int opt = 1;
	setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

	sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons(port);

	if (bind(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		// Winsock bind() does not set errno on failure — the error code
		// lives in WSAGetLastError(). Reading errno returns whatever
		// stale value the CRT had (often 0 → "Success"), which is
		// useless. Report the winsock code directly.
		PX4_ERR("error binding socket 127.0.0.1:%u, WSA error = %d", port, WSAGetLastError());
		return -1;
	}

#else
	std::string sock_path = get_socket_path(_instance_id);

	// Delete socket in case it exists already.
	unlink(sock_path.c_str());

	_fd = socket(AF_UNIX, SOCK_STREAM, 0);

	if (_fd < 0) {
		PX4_ERR("error creating socket");
		return -1;
	}

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);

	if (bind(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("error binding socket %s, error = %s", sock_path.c_str(), strerror(errno));
		return -1;
	}

#endif

	if (listen(_fd, 10) < 0) {
		PX4_ERR("error listening to socket: %s", strerror(errno));
		return -1;
	}

	if (0 != pthread_create(&_server_main_pthread,
				nullptr,
				_server_main_trampoline,
				this)) {
		PX4_ERR("error creating client handler thread");
		return -1;
	}

	return 0;
}

void *
Server::_server_main_trampoline(void *self)
{
	((Server *)self)->_server_main();
	return nullptr;
}

void Server::_pthread_key_destructor(void *arg)
{
	delete ((CmdThreadSpecificData *)arg);
}

void
Server::_server_main()
{
	int ret = pthread_key_create(&_key, _pthread_key_destructor);

	if (ret != 0) {
		PX4_ERR("failed to create pthread key");
		return;
	}

#ifdef __PX4_WINDOWS

	while (true) {
		pollfd listen_fd {poll_socket(_fd), POLLIN, 0};
		int n_ready = poll(&listen_fd, 1, -1);

		if (n_ready < 0) {
			if (errno != EINTR) {
				PX4_ERR("poll() failed: %s", strerror(errno));
				break;
			}

			continue;
		}

		if (!(listen_fd.revents & POLLIN)) {
			continue;
		}

		const socket_handle_t client = accept(_fd, nullptr, nullptr);

		if (client == invalid_socket_handle) {
			PX4_ERR("failed to accept client: %s", strerror(errno));
			continue;
		}

		int stdout_pipe[2] {-1, -1};

		if (pipe(stdout_pipe) != 0) {
			PX4_ERR("could not create stdout pipe for new thread");
			closesocket((SOCKET)client);
			continue;
		}

		FILE *thread_stdout = fdopen(stdout_pipe[1], "wb");

		if (thread_stdout == nullptr) {
			PX4_ERR("could not open stdout pipe for new thread");
			close(stdout_pipe[0]);
			close(stdout_pipe[1]);
			closesocket((SOCKET)client);
			continue;
		}

		// Windows sockets are WinSock handles, not CRT file descriptors, so
		// they cannot be used with fdopen()/FILE*. PX4 command output still
		// expects FILE*, therefore each client gets a real CRT pipe and the
		// handler relays that pipe to the socket after command execution.
		setvbuf(thread_stdout, nullptr, _IOLBF, BUFSIZ);

		ClientThreadArgs *thread_args = new ClientThreadArgs {client, thread_stdout, stdout_pipe[0]};
		pthread_t thread {};
		ret = pthread_create(&thread, nullptr, Server::_handle_client, thread_args);

		if (ret != 0) {
			PX4_ERR("could not start pthread (%i)", ret);
			fclose(thread_stdout);
			close(stdout_pipe[0]);
			closesocket((SOCKET)client);
			delete thread_args;
			continue;
		}

		pthread_detach(thread);
	}

	closesocket((SOCKET)_fd);
#else
	// The list of file descriptors to watch.
	std::vector<pollfd> poll_fds;

	// Watch the listening socket for incoming connections.
	poll_fds.push_back(pollfd {poll_socket(_fd), POLLIN, 0});

	// The list of FILE pointers that we'll need to fclose().
	// stdouts[i] corresponds to poll_fds[i+1].
	std::vector<FILE *> stdouts;

	while (true) {
		int n_ready = poll(poll_fds.data(), poll_fds.size(), -1);

		if (n_ready < 0) {
			// Reboot command causes System Interrupt to stop poll(). This is not an error
			if (errno != EINTR) {
				PX4_ERR("poll() failed: %s", strerror(errno));
				break;
			}
		}

		_lock();

		// Handle any new connections.
		if (poll_fds[0].revents & POLLIN) {
			--n_ready;
			int client = accept(_fd, nullptr, nullptr);

			if (client == -1) {
				PX4_ERR("failed to accept client: %s", strerror(errno));
				_unlock();
				return;
			}

			FILE *thread_stdout = fdopen(client, "w");

			if (thread_stdout == nullptr) {
				PX4_ERR("could not open stdout for new thread");
				close(client);

			} else {
				// Set stream to line buffered.
				setvbuf(thread_stdout, nullptr, _IOLBF, BUFSIZ);

				// Start a new thread to handle the client.
				ClientThreadArgs *thread_args = new ClientThreadArgs {client, thread_stdout};
				pthread_t *thread = &_fd_to_thread[client];
				ret = pthread_create(thread, nullptr, Server::_handle_client, thread_args);

				if (ret != 0) {
					PX4_ERR("could not start pthread (%i)", ret);
					_fd_to_thread.erase(client);
					fclose(thread_stdout);
					delete thread_args;

				} else {
					// We won't join the thread, so detach to automatically release resources at its end
					pthread_detach(*thread);

					// Start listening for the client hanging up.
					poll_fds.push_back(pollfd {poll_socket(client), POLLHUP, 0});

					// Remember the FILE *, so we can fclose() it later.
					stdouts.push_back(thread_stdout);
				}
			}
		}

		// Handle any closed connections.
		for (size_t i = 1; n_ready > 0 && i < poll_fds.size();) {
			if (poll_fds[i].revents) {
				--n_ready;
				auto thread = _fd_to_thread.find(poll_fds[i].fd);

				if (thread != _fd_to_thread.end()) {
					// Thread is still running, so we cancel it.
					// TODO: use a more graceful exit method to avoid resource leaks
					pthread_cancel(thread->second);
					_fd_to_thread.erase(thread);
				}

				fclose(stdouts[i - 1]);
				stdouts.erase(stdouts.begin() + i - 1);
				poll_fds.erase(poll_fds.begin() + i);

			} else {
				++i;
			}
		}

		_unlock();
	}

	std::string sock_path = get_socket_path(_instance_id);
	unlink(sock_path.c_str());
	close(_fd);
#endif // __PX4_WINDOWS
}

void
*Server::_handle_client(void *arg)
{
	ClientThreadArgs *client_args = static_cast<ClientThreadArgs *>(arg);
	FILE *out = client_args->thread_stdout;
	socket_handle_t fd = client_args->client_fd;

	// Read until the end of the incoming stream.
	std::string cmd;
	// Bytes the cmd-read loop pulled past the terminator. These belong to the
	// post-command stdin stream and must be handed to the stdin relay thread.
	std::string stdin_prefill;

	while (true) {
		size_t n = cmd.size();
		cmd.resize(n + 1024);
		ssize_t n_read = socket_read(fd, &cmd[n], cmd.size() - n);

		if (n_read <= 0) {
#ifdef __PX4_WINDOWS
			fclose(out);
			close(client_args->stdout_read_fd);
			closesocket((SOCKET)fd);
			delete client_args;
#else
			delete client_args;
			_cleanup(fd);
#endif
			return nullptr;
		}

		cmd.resize(n + n_read);

		// Command ends in 0x00 (no tty) or 0x01 (tty). Scan from the start of
		// this read because a single recv() may carry the terminator together
		// with a leading slice of the stdin stream the client started sending
		// once it had finished writing the cmd buffer.
		bool found_terminator = false;

		for (size_t i = n; i < cmd.size(); ++i) {
			if ((unsigned char)cmd[i] < 2) {
				if (i + 1 < cmd.size()) {
					stdin_prefill.assign(cmd, i + 1, cmd.size() - (i + 1));
				}

				cmd.resize(i + 1);
				found_terminator = true;
				break;
			}
		}

		if (found_terminator) {
			break;
		}
	}

	if (cmd.size() < 2) {
#ifdef __PX4_WINDOWS
		fclose(out);
		close(client_args->stdout_read_fd);
		closesocket((SOCKET)fd);
		delete client_args;
#else
		delete client_args;
		_cleanup(fd);
#endif
		return nullptr;
	}

	// Last byte is 'isatty'.
	uint8_t isatty = cmd.back();
	cmd.pop_back();

	// We register thread specific data. This is used for PX4_INFO (etc.) log calls.
	CmdThreadSpecificData *thread_data_ptr = static_cast<CmdThreadSpecificData *>(pthread_getspecific(_instance->_key));

	if (thread_data_ptr == nullptr) {
		thread_data_ptr = new CmdThreadSpecificData;
		thread_data_ptr->thread_stdout = out;
		thread_data_ptr->is_atty = isatty;

		(void)pthread_setspecific(_instance->_key, (void *)thread_data_ptr);
	}

#ifdef __PX4_WINDOWS
	std::thread stdout_relay_thread;
	bool stdout_relay_started = false;

	try {
		stdout_relay_thread = std::thread(stdout_relay_loop, client_args->stdout_read_fd, fd);
		stdout_relay_started = true;

	} catch (const std::system_error &) {
		PX4_ERR("could not start stdout relay thread");
		close(client_args->stdout_read_fd);
	}

#endif

	// Redirect STDIN_FILENO onto an anonymous pipe and shovel client-socket
	// bytes into it for the duration of process_line(). Modules that read
	// STDIN_FILENO (uorb top, mavlink shell, etc.) thereby see keystrokes the
	// user types in the px4-* client process. Skip this for the shutdown
	// command — it returns immediately and never reads stdin.
	int stdin_pipe_read_fd = -1;
	int stdin_pipe_write_fd = -1;
	int stdin_backup_fd = -1;
	std::thread stdin_relay_thread;
	bool stdin_relay_started = false;
	bool stdin_redirect_active = false;

	if (!is_shutdown_command(cmd)) {
		auto pipe_fds = create_relay_pipe();

		if (pipe_fds.first < 0 || pipe_fds.second < 0) {
			PX4_ERR("could not create stdin pipe for module");

		} else {
			stdin_pipe_read_fd = pipe_fds.first;
			stdin_pipe_write_fd = pipe_fds.second;

			pthread_mutex_lock(&s_stdin_redirect_mutex);
			stdin_backup_fd = dup(STDIN_FILENO);

			if (stdin_backup_fd < 0 || dup2(stdin_pipe_read_fd, STDIN_FILENO) < 0) {
				PX4_ERR("could not redirect STDIN_FILENO to the relay pipe");

				if (stdin_backup_fd >= 0) {
					::close(stdin_backup_fd);
					stdin_backup_fd = -1;
				}

				::close(stdin_pipe_read_fd);
				::close(stdin_pipe_write_fd);
				stdin_pipe_read_fd = stdin_pipe_write_fd = -1;
				pthread_mutex_unlock(&s_stdin_redirect_mutex);

			} else {
				stdin_redirect_active = true;

				try {
					stdin_relay_thread = std::thread(stdin_relay_loop, fd, stdin_pipe_write_fd, std::move(stdin_prefill));
					stdin_relay_started = true;

				} catch (const std::system_error &) {
					PX4_ERR("could not start stdin relay thread");
					::close(stdin_pipe_write_fd);
					stdin_pipe_write_fd = -1;
				}
			}
		}
	}


	if (is_shutdown_command(cmd)) {
		// shutdown exits the daemon process asynchronously. Reply before
		// requesting shutdown so px4-shutdown gets the normal success marker
		// instead of racing process teardown and reporting 255.
		char buf[2] = {0, 0};
		(void)fwrite(buf, sizeof buf, 1, out);
		fflush(out);

#ifdef __PX4_WINDOWS
		fclose(out);

		if (stdout_relay_started) {
			stdout_relay_thread.join();
		}

		// kShutWr (SD_SEND) only — never SD_RECEIVE/SHUT_RD: half-closing the
		// receive side on Winsock discards the unread receive buffer and
		// causes a RST on closesocket(), which races the queued response
		// bytes and the peer sees WSAECONNRESET before reading them. A
		// half-close-send queues a clean FIN behind any pending sends so the
		// client recv() returns 0 after consuming the full reply.
		shutdown(fd, kShutWr);
		closesocket((SOCKET)fd);
		delete client_args;
#else
		delete client_args;
		_cleanup(fd);
#endif

		(void)Pxh::process_line(cmd, true);
		return nullptr;
	}

	// Run the actual command.
	int retval = Pxh::process_line(cmd, true);

	// Tear down the stdin redirect while the socket is still open. We avoid
	// shutdown(fd, SHUT_RD/SD_RECEIVE) here: on Winsock, half-closing the
	// receive side resets the connection if there is any unread data buffered
	// in the kernel, which would race the {0, retval} reply; on POSIX it
	// would also discard data the relay had not yet drained. Instead, restore
	// STDIN_FILENO and close the pipe read end; the relay thread is detached
	// and exits naturally once its recv() on the client socket returns 0
	// (after the closesocket()/_cleanup below) or its write to the now-closed
	// pipe fails.
	if (stdin_redirect_active) {
		dup2(stdin_backup_fd, STDIN_FILENO);
		::close(stdin_backup_fd);
		::close(stdin_pipe_read_fd);
		stdin_pipe_read_fd = -1;
		pthread_mutex_unlock(&s_stdin_redirect_mutex);

		if (stdin_relay_started) {
			stdin_relay_thread.detach();
		}
	}

	// Report return value.
	char buf[2] = {0, (char)retval};

	if (fwrite(buf, sizeof buf, 1, out) != 1) {
		// Don't care it went wrong, as we're cleaning up anyway.
	}

	// Flush the FILE*'s buffer before we shut down the connection.
	fflush(out);

#ifdef __PX4_WINDOWS
	fclose(out);

	if (stdout_relay_started) {
		stdout_relay_thread.join();
	}

	// See kShutWr comment in the shutdown-command branch above. On modules
	// that return immediately (e.g. `<module> status` when the module is
	// not autostarted, which prints "not running" and returns 1) the entire
	// reply is small enough to still be in flight when this thread tears
	// down the socket — a full half-close of both directions would make the
	// client observe truncation.
	shutdown(fd, kShutWr);
	closesocket((SOCKET)fd);
	delete client_args;
#else
	delete client_args;
	_cleanup(fd);
#endif
	return nullptr;
}

void
Server::_cleanup(int fd)
{
	_instance->_lock();
	_instance->_fd_to_thread.erase(fd);
	_instance->_unlock();

	// We can't close() the fd here, since the main thread is probably
	// polling for it: close()ing it causes a race condition.
	// So, we only call shutdown(), which causes the main thread to register a
	// 'POLLHUP', such that the main thread can close() it for us.
	// We already removed this thread from _fd_to_thread, so there is no risk
	// of the main thread trying to cancel this thread after it already exited.
	shutdown(fd, SHUT_RDWR);
}

} //namespace px4_daemon
