/* This file is being deprecated. Eventually all configuration options will
 * need to be moved into the NuttX configuration system.
 */

/* Version Number */

#define __UCLIBCXX_MAJOR__ 0
#define __UCLIBCXX_MINOR__ 2
#define __UCLIBCXX_SUBLEVEL__ 4

/* Target Features and Options */

#define __UCLIBCXX_HAS_TLS__ 1

/* String and I/O Stream Support */

#undef __UCLIBCXX_HAS_WCHAR__
#define __UCLIBCXX_HAS_LFS__ 1
#define __UCLIBCXX_SUPPORT_CDIR__ 1
#define __UCLIBCXX_SUPPORT_CIN__ 1
#define __UCLIBCXX_SUPPORT_COUT__ 1
#define __UCLIBCXX_SUPPORT_CERR__ 1
#undef __UCLIBCXX_SUPPORT_CLOG__

/* STL and Code Expansion */

#define __UCLIBCXX_STL_BUFFER_SIZE__ 32
#define __UCLIBCXX_CODE_EXPANSION__ 1
#define __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__ 1
#define __UCLIBCXX_EXPAND_STRING_CHAR__ 1
#define __UCLIBCXX_EXPAND_VECTOR_BASIC__ 1
#define __UCLIBCXX_EXPAND_IOS_CHAR__ 1
#define __UCLIBCXX_EXPAND_STREAMBUF_CHAR__ 1
#define __UCLIBCXX_EXPAND_ISTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_OSTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_FSTREAM_CHAR__ 1
#define __UCLIBCXX_EXPAND_SSTREAM_CHAR__ 1
