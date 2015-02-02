#include <systemlib/visibility.h>
#include <systemlib/param/param.h>

#include "gtest/gtest.h"

//#PARAM_DEFINE_INT32(TEST_A, 5);


static const struct param_info_s testparam = {
	"test",
	PARAM_TYPE_INT32,
	.val.i = 2
};


extern param_info_s *__param_start, *__param_end;
extern struct param_info_s	param_array[];
const struct param_info_s	*ib = __param_start;
const struct param_info_s	*il = __param_end;

TEST(ParamTest, ResetAll) {
	param_array[0] = testparam;

	printf("diff: %i\n", (unsigned)(il - ib));
	printf("start: %i\n", __param_start);
	printf("end: %i\n", __param_end);
	
	param_t testparam = param_find("test");
	ASSERT_NE(PARAM_INVALID, testparam) << "param_find failed";

	int32_t value;
	int result = param_get(testparam, &value);
	ASSERT_EQ(0, result) << "param_get failed";
	ASSERT_EQ(2, value) << "wrong param value";

	//ASSERT_TRUE(false) << "fail";
}
