#include <string>
#include <cstring>
#include <cstdlib>

#include <px4/regcache.h>

static const std::string i2c_id("i2c:");
static const std::string spi_id("spi:");
static const std::string mem_id("mem:");

// @path is:
//    "<drivername>@<bus-type>:<bus-num>.<cli-addr>"
//
// i.e.:
//    "mpu9250@i2c:2.0x69"
//

void px4::regcache::bus(int &id, int &cli) const
{
	std::size_t idx;
	std::string buscli = addr();

	id = std::stoi(buscli, &idx, 0);
	cli = std::stoi(buscli.substr(idx + 1), 0, 0);
}

std::string px4::regcache::bus() const
{
	return path.substr(path.find_first_of("@") + 1);
}

std::string px4::regcache::addr() const
{
	return path.substr(path.find_first_of(":") + 1);
}

px4::regcache::regcache(const std::string &p, px4::reg_t *map) : path(p) {
	dev = NULL;
	this->map = map;

	// bgat: we could replace this with a CRTP, but I'm not sure
	// that would improve anything
	if (p.find(i2c_id)) {
		int id, cli;
		bus(id, cli);
		dev = new px4::i2c_regio(id, cli);
	}
	else if (p.find(spi_id))
		dev = new px4::spi_regio(addr());
	else if (p.find(mem_id))
		dev = new px4::mem_regio(addr());
}
