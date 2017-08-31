#include <iostream>
#include <string>
#include <map>

//////////


#include <px4/regcache.h>
#include <px4/device_driver.h>

static std::map<std::string, px4::driver*> driver_list;

int px4::register_driver(const std::string &name, px4::driver *d)
{
	std::pair<std::map<std::string, driver*>::iterator,bool> ret;
	ret = driver_list.insert(std::pair<std::string, px4::driver*>(name, d));
	return ret.second;
}

// called when we've decided a chip exists and needs to be bound to a driver
px4::device *px4::new_device(std::string &path)
{
	// @path is of the form "hmc5335@i2c:1.0x23"
	std::string type = path.substr(0, path.find("@"));
	std::map<std::string, px4::driver*>::iterator it
		= driver_list.find(type);
	if (it == driver_list.end()) return NULL;

	px4::driver *clone = it->second;
	px4::device *dev = clone(path);
	if ((dev) && dev->is_ok())
		dev->probe();
	return dev;
}
