#pragma once

#ifndef NDEBUG
    #define DEBUG_PRINT(...) printf("[DEBUG %s:%d] " , __FILE__, __LINE__); printf(__VA_ARGS__)
    #define DEBUG_EXECUTE(x) x
#else
    #define DEBUG_PRINT(...)
    #define DEBUG_EXECUTE(x)
#endif
