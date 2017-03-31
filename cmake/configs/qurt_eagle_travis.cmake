include(configs/qcom/qurt_eagle_common)
include(configs/qcom/qurt_modules_default)

# Run a full link with build stubs to make sure qurt target isn't broken
set(QURT_ENABLE_STUBS "1")

