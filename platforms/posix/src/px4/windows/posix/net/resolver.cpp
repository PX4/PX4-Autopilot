/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file resolver.cpp
 *
 * Name- and address-lookup surface (inet_aton / inet_netof /
 * gethostbyname / getprotobyname / getservbyname / ...). MinGW
 * ships the BSD socket types but not the libc resolver DB
 * frontends, so we parse /etc/hosts / /etc/networks /
 * /etc/protocols / /etc/services ourselves, caching the parsed
 * entries in process-local tables.
 *
 * The exported `hostent`, `netent`, `protoent`, and `servent` APIs are old C
 * interfaces that return pointers into static storage. The structs below own
 * strings in C++ containers and then finalize pointer arrays that remain valid
 * until the database is cleared.
 */

#include "px4_windows_internal.h"

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


static std::string px4_windows_etc_path(const char *filename)
{
	char system_dir[MAX_PATH] {};
	const UINT len = GetSystemDirectoryA(system_dir, MAX_PATH);

	if (len > 0 && len < MAX_PATH) {
		std::string path(system_dir, len);
		path += "\\drivers\\etc\\";
		path += filename;
		return path;
	}

	return std::string("C:\\Windows\\System32\\drivers\\etc\\") + filename;
}

static std::vector<std::string> px4_tokenize_db_line(const std::string &line)
{
	/* Windows uses the same whitespace-and-comment syntax as the BSD resolver
	 * files, so a tiny lexer is enough for PX4's needs. */
	const std::string body = line.substr(0, line.find('#'));
	std::istringstream stream(body);
	std::vector<std::string> tokens;
	std::string token;

	while (stream >> token) {
		tokens.push_back(token);
	}

	return tokens;
}

struct PX4HostDbEntry {
	std::string name;
	std::vector<std::string> aliases_storage;
	std::vector<char *> aliases;
	int addrtype = AF_UNSPEC;
	int addrlen = 0;
	std::array<unsigned char, 16> address {};
	std::vector<char *> address_list;
	struct hostent host {};

	void finalize()
	{
		/* hostent wants mutable `char *` arrays. Store strings first, then point
		 * into them after vector growth is complete. */
		aliases.clear();

		for (std::string &alias : aliases_storage) {
			aliases.push_back(alias.data());
		}

		aliases.push_back(nullptr);
		address_list = {reinterpret_cast<char *>(address.data()), nullptr};

		host.h_name = name.empty() ? nullptr : name.data();
		host.h_aliases = aliases.data();
		host.h_addrtype = (short)addrtype;
		host.h_length = (short)addrlen;
		host.h_addr_list = address_list.data();
	}

	bool matches_name(const char *query) const
	{
		if (!query) {
			return false;
		}

		if (_stricmp(name.c_str(), query) == 0) {
			return true;
		}

		for (const std::string &alias : aliases_storage) {
			if (_stricmp(alias.c_str(), query) == 0) {
				return true;
			}
		}

		return false;
	}
};

struct PX4NetDbEntry {
	std::string name;
	std::vector<std::string> aliases_storage;
	std::vector<char *> aliases;
	struct netent net {};

	void finalize()
	{
		aliases.clear();

		for (std::string &alias : aliases_storage) {
			aliases.push_back(alias.data());
		}

		aliases.push_back(nullptr);

		net.n_name = name.empty() ? nullptr : name.data();
		net.n_aliases = aliases.data();
	}

	bool matches_name(const char *query) const
	{
		if (!query) {
			return false;
		}

		if (_stricmp(name.c_str(), query) == 0) {
			return true;
		}

		for (const std::string &alias : aliases_storage) {
			if (_stricmp(alias.c_str(), query) == 0) {
				return true;
			}
		}

		return false;
	}
};

struct PX4ProtoDbEntry {
	std::string name;
	std::vector<std::string> aliases_storage;
	std::vector<char *> aliases;
	struct protoent proto {};

	void finalize()
	{
		aliases.clear();

		for (std::string &alias : aliases_storage) {
			aliases.push_back(alias.data());
		}

		aliases.push_back(nullptr);

		proto.p_name = name.empty() ? nullptr : name.data();
		proto.p_aliases = aliases.data();
	}
};

struct PX4ServiceDbEntry {
	std::string name;
	std::vector<std::string> aliases_storage;
	std::vector<char *> aliases;
	std::string protocol;
	struct servent service {};

	void finalize()
	{
		aliases.clear();

		for (std::string &alias : aliases_storage) {
			aliases.push_back(alias.data());
		}

		aliases.push_back(nullptr);

		service.s_name = name.empty() ? nullptr : name.data();
		service.s_aliases = aliases.data();
		service.s_proto = protocol.empty() ? nullptr : protocol.data();
	}
};

template<typename Entry>
struct PX4ResolverDb {
	std::vector<Entry> entries;
	size_t next = 0;
	bool loaded = false;
	bool stay_open = false;

	void reset()
	{
		next = 0;
	}

	void close()
	{
		next = 0;

		/* POSIX set*ent(stay_open=1) asks libc to keep the database cached
		 * across end*ent(). Honor that so repeated PX4 lookups avoid reparsing
		 * the Windows etc files. */
		if (!stay_open) {
			entries.clear();
			loaded = false;
		}
	}
};

static PX4ResolverDb<PX4HostDbEntry> g_hosts_db;
static PX4ResolverDb<PX4NetDbEntry> g_networks_db;
static PX4ResolverDb<PX4ProtoDbEntry> g_protocols_db;
static PX4ResolverDb<PX4ServiceDbEntry> g_services_db;

static void px4_finalize_hosts_db(PX4ResolverDb<PX4HostDbEntry> &db)
{
	for (PX4HostDbEntry &entry : db.entries) {
		entry.finalize();
	}
}

static void px4_finalize_networks_db(PX4ResolverDb<PX4NetDbEntry> &db)
{
	for (PX4NetDbEntry &entry : db.entries) {
		entry.finalize();
	}
}

static void px4_finalize_protocols_db(PX4ResolverDb<PX4ProtoDbEntry> &db)
{
	for (PX4ProtoDbEntry &entry : db.entries) {
		entry.finalize();
	}
}

static void px4_finalize_services_db(PX4ResolverDb<PX4ServiceDbEntry> &db)
{
	for (PX4ServiceDbEntry &entry : db.entries) {
		entry.finalize();
	}
}

static void px4_load_hosts_db(PX4ResolverDb<PX4HostDbEntry> &db)
{
	if (db.loaded) {
		return;
	}

	db.entries.clear();
	db.next = 0;

	std::ifstream file(px4_windows_etc_path("hosts"));
	std::string line;

	/* hosts lines are: address canonical-name [aliases...]. */
	while (std::getline(file, line)) {
		const std::vector<std::string> tokens = px4_tokenize_db_line(line);

		if (tokens.size() < 2) {
			continue;
		}

		PX4HostDbEntry entry {};

		if (InetPtonA(AF_INET, tokens[0].c_str(), entry.address.data()) == 1) {
			entry.addrtype = AF_INET;
			entry.addrlen = sizeof(struct in_addr);

		} else if (InetPtonA(AF_INET6, tokens[0].c_str(), entry.address.data()) == 1) {
			entry.addrtype = AF_INET6;
			entry.addrlen = sizeof(struct in6_addr);

		} else {
			continue;
		}

		entry.name = tokens[1];

		for (size_t i = 2; i < tokens.size(); ++i) {
			entry.aliases_storage.push_back(tokens[i]);
		}

		db.entries.push_back(std::move(entry));
	}

	px4_finalize_hosts_db(db);
	db.loaded = true;
}

static void px4_load_networks_db(PX4ResolverDb<PX4NetDbEntry> &db)
{
	if (db.loaded) {
		return;
	}

	db.entries.clear();
	db.next = 0;

	std::ifstream file(px4_windows_etc_path("networks"));
	std::string line;

	while (std::getline(file, line)) {
		const std::vector<std::string> tokens = px4_tokenize_db_line(line);

		if (tokens.size() < 2) {
			continue;
		}

		PX4NetDbEntry entry {};
		entry.name = tokens[0];
		entry.net.n_addrtype = AF_INET;
		entry.net.n_net = (u_long)inet_network(tokens[1].c_str());

		if (entry.net.n_net == INADDR_NONE) {
			char *end = nullptr;
			const unsigned long value = strtoul(tokens[1].c_str(), &end, 0);

			if (!end || *end != '\0') {
				continue;
			}

			entry.net.n_net = (u_long)value;
		}

		for (size_t i = 2; i < tokens.size(); ++i) {
			entry.aliases_storage.push_back(tokens[i]);
		}

		db.entries.push_back(std::move(entry));
	}

	px4_finalize_networks_db(db);
	db.loaded = true;
}

static void px4_load_protocols_db(PX4ResolverDb<PX4ProtoDbEntry> &db)
{
	if (db.loaded) {
		return;
	}

	db.entries.clear();
	db.next = 0;

	std::ifstream file(px4_windows_etc_path("protocol"));
	std::string line;

	while (std::getline(file, line)) {
		const std::vector<std::string> tokens = px4_tokenize_db_line(line);

		if (tokens.size() < 2) {
			continue;
		}

		char *end = nullptr;
		const long number = strtol(tokens[1].c_str(), &end, 10);

		if (!end || *end != '\0') {
			continue;
		}

		PX4ProtoDbEntry entry {};
		entry.name = tokens[0];
		entry.proto.p_proto = (short)number;

		for (size_t i = 2; i < tokens.size(); ++i) {
			entry.aliases_storage.push_back(tokens[i]);
		}

		db.entries.push_back(std::move(entry));
	}

	px4_finalize_protocols_db(db);
	db.loaded = true;
}

static void px4_load_services_db(PX4ResolverDb<PX4ServiceDbEntry> &db)
{
	if (db.loaded) {
		return;
	}

	db.entries.clear();
	db.next = 0;

	std::ifstream file(px4_windows_etc_path("services"));
	std::string line;

	/* services lines are: service-name port/protocol [aliases...]. */
	while (std::getline(file, line)) {
		const std::vector<std::string> tokens = px4_tokenize_db_line(line);

		if (tokens.size() < 2) {
			continue;
		}

		const size_t slash = tokens[1].find('/');

		if (slash == std::string::npos) {
			continue;
		}

		char *end = nullptr;
		const long port = strtol(tokens[1].substr(0, slash).c_str(), &end, 10);

		if (!end || *end != '\0') {
			continue;
		}

		PX4ServiceDbEntry entry {};
		entry.name = tokens[0];
		entry.protocol = tokens[1].substr(slash + 1);
		entry.service.s_port = htons((u_short)port);

		for (size_t i = 2; i < tokens.size(); ++i) {
			entry.aliases_storage.push_back(tokens[i]);
		}

		db.entries.push_back(std::move(entry));
	}

	px4_finalize_services_db(db);
	db.loaded = true;
}

extern "C" int inet_aton(const char *cp, struct in_addr *inp)
{
	if (!cp || !inp) {
		return 0;
	}

	return InetPtonA(AF_INET, cp, inp) == 1 ? 1 : 0;
}

extern "C" char *inet_ntoa_r(struct in_addr in, char *buf, size_t buflen)
{
	if (!buf || buflen < INET_ADDRSTRLEN) {
		errno = ENOSPC;
		return nullptr;
	}

	if (InetNtopA(AF_INET, &in, buf, (DWORD)buflen) == nullptr) {
		return nullptr;
	}

	return buf;
}

extern "C" in_addr_t inet_network(const char *cp)
{
	struct in_addr addr {};

	if (!inet_aton(cp, &addr)) {
		return INADDR_NONE;
	}

	return ntohl(addr.s_addr);
}

extern "C" in_addr_t inet_lnaof(struct in_addr in)
{
	const in_addr_t addr = ntohl(in.s_addr);

	if (IN_CLASSA(addr)) {
		return addr & IN_CLASSA_HOST;

	} else if (IN_CLASSB(addr)) {
		return addr & IN_CLASSB_HOST;

	} else if (IN_CLASSC(addr)) {
		return addr & IN_CLASSC_HOST;
	}

	return 0;
}

extern "C" in_addr_t inet_netof(struct in_addr in)
{
	const in_addr_t addr = ntohl(in.s_addr);

	if (IN_CLASSA(addr)) {
		return (addr & IN_CLASSA_NET) >> IN_CLASSA_NSHIFT;

	} else if (IN_CLASSB(addr)) {
		return (addr & IN_CLASSB_NET) >> IN_CLASSB_NSHIFT;

	} else if (IN_CLASSC(addr)) {
		return (addr & IN_CLASSC_NET) >> IN_CLASSC_NSHIFT;
	}

	return addr;
}

extern "C" struct in_addr inet_makeaddr(in_addr_t net, in_addr_t host)
{
	in_addr_t addr = 0;

	if (net < IN_CLASSA_MAX) {
		addr = (net << IN_CLASSA_NSHIFT) | (host & IN_CLASSA_HOST);

	} else if (net < IN_CLASSB_MAX) {
		addr = (net << IN_CLASSB_NSHIFT) | (host & IN_CLASSB_HOST);

	} else {
		addr = (net << IN_CLASSC_NSHIFT) | (host & IN_CLASSC_HOST);
	}

	struct in_addr out {};

	out.s_addr = htonl(addr);

	return out;
}

extern "C" const char *hstrerror(int err)
{
	return px4_hstrerror_text(err);
}

extern "C" void sethostent(int stay_open)
{
	g_hosts_db.stay_open = stay_open != 0;
	px4_load_hosts_db(g_hosts_db);
	g_hosts_db.reset();
}

extern "C" void endhostent(void)
{
	g_hosts_db.close();
}

extern "C" struct hostent *gethostent(void)
{
	px4_load_hosts_db(g_hosts_db);

	if (g_hosts_db.next >= g_hosts_db.entries.size()) {
		WSASetLastError(WSANO_DATA);
		errno = ENOENT;
		return nullptr;
	}

	return &g_hosts_db.entries[g_hosts_db.next++].host;
}

extern "C" void setnetent(int stay_open)
{
	g_networks_db.stay_open = stay_open != 0;
	px4_load_networks_db(g_networks_db);
	g_networks_db.reset();
}

extern "C" void endnetent(void)
{
	g_networks_db.close();
}

extern "C" struct netent *getnetent(void)
{
	px4_load_networks_db(g_networks_db);

	if (g_networks_db.next >= g_networks_db.entries.size()) {
		WSASetLastError(WSANO_DATA);
		errno = ENOENT;
		return nullptr;
	}

	return &g_networks_db.entries[g_networks_db.next++].net;
}

extern "C" struct netent *getnetbyname(const char *name)
{
	px4_load_networks_db(g_networks_db);

	for (PX4NetDbEntry &entry : g_networks_db.entries) {
		if (entry.matches_name(name)) {
			return &entry.net;
		}
	}

	WSASetLastError(WSANO_DATA);
	errno = ENOENT;
	return nullptr;
}

extern "C" struct netent *getnetbyaddr(uint32_t net, int type)
{
	px4_load_networks_db(g_networks_db);

	for (PX4NetDbEntry &entry : g_networks_db.entries) {
		if ((type == AF_UNSPEC || entry.net.n_addrtype == type) && entry.net.n_net == (u_long)net) {
			return &entry.net;
		}
	}

	WSASetLastError(WSANO_DATA);
	errno = ENOENT;
	return nullptr;
}

extern "C" void setprotoent(int stay_open)
{
	g_protocols_db.stay_open = stay_open != 0;
	px4_load_protocols_db(g_protocols_db);
	g_protocols_db.reset();
}

extern "C" void endprotoent(void)
{
	g_protocols_db.close();
}

extern "C" struct protoent *getprotoent(void)
{
	px4_load_protocols_db(g_protocols_db);

	if (g_protocols_db.next >= g_protocols_db.entries.size()) {
		WSASetLastError(WSANO_DATA);
		errno = ENOENT;
		return nullptr;
	}

	return &g_protocols_db.entries[g_protocols_db.next++].proto;
}

extern "C" void setservent(int stay_open)
{
	g_services_db.stay_open = stay_open != 0;
	px4_load_services_db(g_services_db);
	g_services_db.reset();
}

extern "C" void endservent(void)
{
	g_services_db.close();
}

extern "C" struct servent *getservent(void)
{
	px4_load_services_db(g_services_db);

	if (g_services_db.next >= g_services_db.entries.size()) {
		WSASetLastError(WSANO_DATA);
		errno = ENOENT;
		return nullptr;
	}

	return &g_services_db.entries[g_services_db.next++].service;
}
