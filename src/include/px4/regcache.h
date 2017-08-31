#if !defined (__REGCACHE_H_INCLUDED__)
#define __REGCACHE_H_INCLUDED__

#include <cstring>
#include <string>

namespace px4 {

	// The @reg_t type describes a "register". Instances of @reg_t
	// usually correspond one-to-one to the literal, physical
	// registers of i2c or spi chips, and to memory addresses
	// within a memory-mapped peripheral.
	//
	// For i2c and spi devices, the @offset field is the literal
	// register address; for memory-mapped devices, @offset is the
	// distance from a base address defined by the platform
	// integrator or driver author. Being careful in the values
	// used will allow driver implementations to manage more than
	// one chip with multiple instances of the same class.
	//
	// The @ioperm field indicates whether the register is
	// readable, writeable, or both.
	//
	// The @width field is the size of the register, in bytes.
	//
	// The @rmask field indicates the bits in the register that
	// are marked as "reserved" in the chip's datasheet. The
	// inverse of this bitfield's value is bitwise-ANDed with values
	// going to or from the physical register itself:
	//
	//   val &= ~reg_t.rmask
	//
	// The above operation takes place within the methods of the
	// @regio class as values transit the low-level bus driver,
	// and guarantees that reserved bits are always read and
	// written as zeros. Chip drivers need not perform this
	// operation themselves.
	//
	// The preferred way to use the @reg_t type is to define
	// instances within an array, indexed by a separate
	// enumerator:
	//
	// enum {REG_STATUS, REG_ID_A, REG_CONFIG_A, ...};
	// static px4::reg_t regs[] = {
	//	[REG_STATUS] = {0x9, px4::ioperm::READ, 1, 0xfc},
	//	[REG_ID_A] = {0xa, px4::ioperm::READ, 1, 0},
	//	[REG_CONFIG_A] = {0, px4::ioperm::READ | px4::ioperm::WRITE, 1, 1 << 7},
	//	...
	// };
	//
	// This approach makes regs[] compact, even for chips with
	// sparsely-populated register spaces. It also allows driver
	// code to refer to registers by the name rather than address.
	//
	// The above approach assumes that chips have identical
	// register maps regardless of the bus used to reach them
	// (which is usually the case for mainstream multi-bus chips).
	// If a chip driver really needs to know the underlying bus
	// type, they can strstr() the chip's complete @id() string in
	// the associated @regcache object.
	//
	// Finally, @reg_t instances need not be limited to a
	// one-to-one correspondence with a chip's physical register
	// map, and the combination of offset+width are free to
	// overlap the spaces of adjacent registers. This feature
	// allows chip drivers to provide per-register interfaces
	// useful for troubleshooting, while still taking advantage of
	// cross-register locking and other features that most i2c/spi
	// chips use to guarantee synchronized capture and transfer of
	// multi-byte values.
	//
	// For example, the following fragment describes the X-axis
	// acceleration registers of the Bosch BMA250E accelerometer
	// as two unrelated, octet-wide addresses, and then again as a
	// single, two-octet-wide register.
	//
	//    enum {..., ACCD_X_LSB, ACCD_X_MSB, ACCD_X, ...};
	//    static px4::reg_t bma250e_regs[] = {
	//        ...
	//        [ACCD_X_LSB] = {2, px4::ioperm::READ, 1, 0xc1},
	//        [ACCD_X_MSB] = {3, px4::ioperm::READ, 1, 0},
	//        [ACCD_X]     = {2, px4::ioperm::READ, 2, 0xc100},
	//        ...
	//    };
	//
	// If a chip driver reads from the ACCD_X (pseudo-)register, a
	// two-octet burst read will be performed by the underlying
	// bus driver. According to the BMA250E datasheet, this
	// guarantees data consistency between the MSB and LSB values.
	//
	// With i2c and spi, multi-octet transfers start with the
	// register at the lowest ordinal address, and proceed in
	// increasing address order. Thus, ACCD_X must have the same
	// address as ACCD_X_LSB, and not ACCD_X_MSB.
	//
	// The above conventions are all adopted for memory busses
	// too, except in cases where register widths match a native
	// type. In those situations, the transfer is generally
	// performed as though it was of that type. TODO: clarify this
	// across endians.
	//
	// TODO: the above could be extended to support a six-octet
	// burst transfer (or larger) for reading X, Y, and Z
	// simultaneously, by providing a method with a larger data
	// buffer than <int>.
	//

	typedef struct {
		int offset;	// address
		int ioperm;	// i/o permissions
		int width;	// size in octets
		int rmask;	// reserved bits
	} reg_t;
	enum ioperm {READ = (1 << 0), WRITE = (1 << 1), READWRITE = (1 << 0 | 1 << 1)};

	// The @regio class provides a uniform interface to an
	// associated bus, regardless of the bus type.
	//
	// Bus drivers extend this class to bridge the gap between the
	// abstract register model expressed by @reg_t, and the
	// underlying hardware itself, i.e. i2c, spi, memory-mapped
	// I/O, etc. Chip drivers don't ever see their associated
	// @regio directly.
	//
	// Kernels vary in how they provide access to the
	// aforementioned busses, and those differences are
	// accommodated by bus driver classes as well. In other words,
	// there are one (or more) classes implemented for i2c under
	// NuttX, and a different one or talking to /dev/i2c-* on
	// Linux.
	//
	// Chip drivers get to their chips via the bus driver class
	// that's appropriate for the bus and operating system in use
	// at the time of access. Chip drivers are agnostic to what
	// the underlying bus is, all chips are modeled as simply a
	// "cache of registers" that are selected using enumerators
	// within the driver's namespace.
	//
	// Bus drivers must be careful to accommodate appropriate data
	// widths. For i2c and spi, that usually means that each
	// register contains a single octet, but it's often desirable
	// to read a block of adjacent registers in one transfer. The
	// @width field in struct @reg_t facilitates this.
	//
	// Drivers for busses that support multi-octet transfers must
	// implement them, or risk breaking chip drivers that assume
	// and/or depend on the correct behavior.
	//
	class regio {
	public:
		regio() { }
		virtual ~regio() {}

		virtual bool is_ok() { return true; }

		inline bool is_readable(const reg_t &reg) const { return reg.ioperm & ioperm::READ ? true : false; }
		inline bool is_writable(const reg_t &reg) const { return reg.ioperm & ioperm::WRITE ? true : false; }
		inline int mask_reserved(const reg_t &reg, int &val) { return val & ~reg.rmask; }

		bool read(const reg_t &reg, int &val) {
			if (is_readable(reg) == false)
				return false;
			if (read_reg(reg, val) == false)
				return false;
			val = mask_reserved(reg, val);
			return true;
		}
		bool write(const reg_t &reg, int val) {
			if (is_writable(reg) == false)
				return false;
			if (write_reg(reg, mask_reserved(reg, val)) == false)
				return false;
			return true;
		}

	protected:
		// these get overloaded to support specific bus and OS types
		virtual bool read_reg(const reg_t &reg, int &val) { return false; }
		virtual bool write_reg(const reg_t &reg, int val) { return false; }
	};


	// The @regcache class implements the general behavior of our
	// uniform, abstract register model. The term reads as a
	// "cache of registers", since this class doesn't "cache"
	// registers in the save-and-restore or keep-a-local-copy
	// senses.
	//
	// Device driver authors will receive a reference to a
	// @regcache object, to use as a handle for accessing the
	// values of registers contained in their chips. A @regcache
	// object is created, factory-style, each time a chip device
	// is declared.
	//
	class regcache {
	public:
		regcache(const std::string &path, reg_t *map);
		~regcache() { delete dev; }

		void bus(int &id, int &cli) const;
		std::string bus() const; // "i2c:1.2"
		std::string addr() const; // "1.2"
		inline bool read(const reg_t &reg, int &val) { return dev->read(reg, val); }
		inline bool read(int idx, int &val) { return read(map[idx], val); }
		inline bool write(const reg_t &reg, int val) { return dev->write(reg, val); }
		inline bool write(int idx, int val) { return write(map[idx], val); }

		bool is_ok() { return dev ? dev->is_ok() : false; }

		// TODO: operator[]?

	private:
		regcache(/* a register cache with no registers makes no sense */);
		const std::string &path; // i.e. "mpu9250@i2c:2.3"
		const reg_t *map;
		regio *dev;
	};

#if defined(__linux__)
#include <px4/linux-regio.h>
#else
#error "I don't (yet) know how to do regcache i/o for this target"
#endif
}

#endif
