#pragma once

namespace sdcard_check
{

struct Result {
	int num_checked;
	int num_missing;
	int num_mismatch;
};

Result verify_all();

} // namespace sdcard_check
