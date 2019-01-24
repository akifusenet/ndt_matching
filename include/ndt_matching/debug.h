#ifndef NDT_MATCHING__CDEBUG_H
#define NDT_MATCHING__CDEBUG_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

namespace ndt_matching {
#define timeDiff(start, end) ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec)
} // namespace ndt_matching

#endif // NDT_MATCHING__CDEBUG_H
