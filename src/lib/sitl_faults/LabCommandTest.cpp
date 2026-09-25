#include <cassert>
#include <initializer_list>
#include <vector>

static void test_log(const char *, ...) {}
#define PX4_INFO(...) test_log(__VA_ARGS__)
#define PX4_ERR(...) test_log(__VA_ARGS__)
#include <systemcmds/failure/lab.h>

static int run(std::initializer_list<const char *> args, bool enabled = true)
{
	std::vector<char *> argv;

	for (auto arg : args) { argv.push_back(const_cast<char *>(arg)); }

	return sitl_faults::command(static_cast<int>(argv.size()), argv.data(), enabled);
}

int main()
{
	assert(run({"failure", "lab", "fade", "14601", "rx", "5000", "200", "1000"}, false) == 1);
	assert(run({"failure", "lab", "fade", "14601", "rx", "5000", "200", "1000"}) == 0);
	assert(run({"failure", "lab", "fade", "14601", "rx", "5000", "200", "1000"}) == 1);
	assert(run({"failure", "lab", "status"}, false) == 0);
	assert(run({"failure", "lab", "reset"}, false) == 0);
	assert(run({"failure", "lab", "fade", "0", "both", "5000", "200", "1000"}) == 1);
	assert(run({"failure", "lab", "dds", "bad", "5000"}) == 1);
	assert(run({"failure", "lab", "dds", "both", "-1"}) == 1);
	assert(run({"failure", "lab", "dds", "both", "120001"}) == 1);
	assert(run({"failure", "lab", "dds", "both", "5000"}) == 0);
	assert(run({"failure", "lab", "storage", "501", "5000"}) == 1);
	assert(run({"failure", "lab", "storage", "100", "5000"}) == 0);
	assert(run({"failure", "lab", "reset"}) == 0);
}
