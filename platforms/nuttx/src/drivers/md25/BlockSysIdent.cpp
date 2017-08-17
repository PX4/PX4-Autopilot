#include "BlockSysIdent.hpp"

BlockSysIdent::BlockSysIdent() :
	Block(NULL, "SYSID"),
	_freq(this, "FREQ"),
	_ampl(this, "AMPL")
{
}
