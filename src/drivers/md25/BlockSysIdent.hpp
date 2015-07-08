#include <controllib/block/Block.hpp>
#include <controllib/block/BlockParam.hpp>

class BlockSysIdent : public control::Block
{
public:
	BlockSysIdent();
private:
	control::BlockParam<float> _freq;
	control::BlockParam<float> _ampl;
};
