#if defined(RL_TOOLS_COMMIT_HASH_EXTERNAL)
#define RL_TOOLS_COMMIT_HASH_SET
#undef RL_TOOLS_COMMIT_HASH
#define RL_TOOLS_COMMIT_HASH RL_TOOLS_COMMIT_HASH_EXTERNAL
#else
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("RL_TOOLS_COMMIT_HASH and RL_TOOLS_COMMIT_HASH_SHORT are not passed by the build system")
#endif
// Note that this commit hash will always be delayed
#define RL_TOOLS_COMMIT_HASH 111f0b82ba87ee4f1938f4d85ee2d266e70f4498
#define RL_TOOLS_COMMIT_HASH_SHORT 1122059 // decimal of the first 24 bits of the commit hash: julia -e 'println(parse(Int, "111f0b", base=16))'
#endif

#define RL_TOOLS_STRINGIFY_INNER(x) #x
#define RL_TOOLS_STRINGIFY(x) RL_TOOLS_STRINGIFY_INNER(x)

#if (defined(RL_TOOLS_TARGET_COMMIT_HASH) && !defined(RL_TOOLS_TARGET_COMMIT_HASH_SHORT)) || (!defined(RL_TOOLS_TARGET_COMMIT_HASH) && defined(RL_TOOLS_TARGET_COMMIT_HASH_SHORT))
#error "RL_TOOLS_TARGET_COMMIT_HASH and RL_TOOLS_TARGET_COMMIT_HASH_SHORT must be defined together"
#endif

#if defined(RL_TOOLS_TARGET_COMMIT_HASH_SHORT) && RL_TOOLS_COMMIT_HASH_SHORT != RL_TOOLS_TARGET_COMMIT_HASH_SHORT
#ifdef RL_TOOLS_ENABLE_WARNINGS
#pragma message("Discarding rl_tools commit " RL_TOOLS_STRINGIFY(RL_TOOLS_COMMIT_HASH)
#endif " because it does not match the target commit " RL_TOOLS_STRINGIFY(RL_TOOLS_TARGET_COMMIT_HASH))
#define RL_TOOLS_USE_THIS_VERSION 0
#else
#define RL_TOOLS_USE_THIS_VERSION 1
#endif