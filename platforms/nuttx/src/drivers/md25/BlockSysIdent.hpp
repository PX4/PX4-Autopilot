#include <controllib/block/Block.hpp>
#include <controllib/block/BlockParam.hpp>

class BlockSysIdent : public control::Block
{
public:
	BlockSysIdent();
private:
	BlockParamFloat _freq;
	BlockParamFloat _ampl;
};
