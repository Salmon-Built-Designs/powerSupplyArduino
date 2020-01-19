#include "build_num.h"
#define MAJOR 0
#define MINOR 1
#define STRINGIZE2(s) #s
#define STRINGIZE(s) STRINGIZE2(s)
#define VERSION STRINGIZE(MAJOR) "." STRINGIZE(MINOR) "." STRINGIZE(BUILD_N)

