#if !defined(__LINUX_REGIO__INCLUDED__)
#define __LINUX_REGIO__INCLUDED__

class i2c_regio : public regio {
public:
	i2c_regio(int bus, int addr);
	~i2c_regio();
	bool is_ok() { return fd != -1 ? true : false; }
	bool read_reg(const reg_t &reg, int &val);
	bool write_reg(const reg_t &reg, int val);

private:
	i2c_regio(/* i2c clients need addresses specified */ ) {}
	int fd;
};


// TODO: Linux userspace spidev register accessor
class spi_regio : public regio {
public:
	// @id: client identification string, in the form "spi:<bus>.<slave>"
	spi_regio(const std::string &id) {
		// TODO: ...
		fd = -1;
	}
	~spi_regio() {}

	// TODO: implement these
	bool read_reg(const reg_t &reg, int &val) { return false; }
	bool write_reg(const reg_t &reg, int val) { return false; }
private:
	spi_regio(/* spi clients need addresses specified */ ) {}
	int fd;
};

// TODO: Linux mmio accessor
class mem_regio : public regio {
public:
	// @id: client identification string, in the form "mem:<len>@<physaddr>"
	mem_regio(const std::string &id) {
		// TODO: ...
		mem = NULL;
	}
	~mem_regio() {}

	// TODO: implement these
	bool read_reg(const reg_t &reg, int &val) { return false; }
	bool write_reg(const reg_t &reg, int val) { return false; }
private:
	mem_regio(/* mem clients need addresses specified */ ) {}
	void *mem;
};

#endif
