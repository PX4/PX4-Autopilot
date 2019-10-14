#include "stub_devmgr.h"


namespace DriverFramework
{
int DevMgr::getNextDeviceName(unsigned int &index, const char **instancename)
{
	return stub_getNextDeviceName_callback(index, instancename);
}
}
