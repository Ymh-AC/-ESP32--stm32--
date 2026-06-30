#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>
#include <string.h>

#define DEBUG_ENABLE 1
//strrchr从后往前找
#define __FILE_NAME_ (strrchr (__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define __FILE_NAME (strrchr (__FILE_NAME_, '/') ? strrchr(__FILE_NAME_, '/') + 1 : __FILE_NAME_)

#if DEBUG_ENABLE
    #include <stdio.h>
    #define debug_printf(format, ...) printf("[%s:%d] " format, __FILE_NAME, __LINE__, ##__VA_ARGS__)
#else
    #define debug_printf(format, ...)
#endif

#endif