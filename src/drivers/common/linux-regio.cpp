#include <string>

#include <px4/regcache.h>
#include <px4/linux-regio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#if !defined(I2C_SMBUS_READ)
// instead of forcing libi2c-dev, recreate the parts we need

/* smbus_access read or write markers */
#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// block data transfer command
#define I2C_SMBUS_BLOCK_DATA	5
#define I2C_SMBUS_BLOCK_MAX	32

union i2c_smbus_data {
	__u8 byte;
	__u16 word;
	__u8 block[I2C_SMBUS_BLOCK_MAX + 2];
};
#endif

px4::i2c_regio::i2c_regio(int bus, int addr) {
	int ret;
	std::string path;
	path = "/dev/i2c-" + std::to_string(bus);
	fd = open(path.c_str(), O_RDWR);
	if (fd == -1)
		return;
	ret = ioctl(fd, I2C_SLAVE, addr);
	if (ret)
		goto err_open;
	return;
err_open:
	close(fd);
	fd = -1;
	return;
}

px4::i2c_regio::~i2c_regio() { if (fd != -1) close(fd); }

bool px4::i2c_regio::read_reg(const px4::reg_t &reg, int &val) {
	union i2c_smbus_data data;
	struct i2c_smbus_ioctl_data args;
	int ret;

	__u8 *buf = &data.block[1];
	data.block[0] = reg.width;

	args.read_write = I2C_SMBUS_READ;
	args.command = reg.offset;
	args.data = &data;
	args.size = I2C_SMBUS_BLOCK_DATA;

	ret = ioctl(fd, I2C_SMBUS, &args);
	if (ret || !data.block[0])
		return false;

	val = *buf++;

	// note: this will merely truncate, rather than clobber memory,
	// if the register size is larger than sizeof(int)
	while (--data.block[0])
		val = (val << 8) + *buf++;

	return true;
}

bool px4::i2c_regio::write_reg(const px4::reg_t &reg, int val) { return false; }
