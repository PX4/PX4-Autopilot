#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC int px4_eag_publish_main(int argc, char *argv[]);

#undef EXTERNC
