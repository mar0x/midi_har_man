
#pragma once

#if defined(DEBUG)

#if !defined(DEBUG_PRINT)
#undef DEBUG
#endif

#endif

#if defined(DEBUG)

uint8_t debug_level_ = 0;

template<typename T1>
void debug_(const T1& a1) {
    DEBUG_PRINT.println(a1);
}

template<typename T1, typename ...Args>
void debug_(const T1& a1, Args... args) {
    DEBUG_PRINT.print(a1);
    debug_(args...);
}

template<typename T1, typename ...Args>
void debug(uint8_t l, const T1& a1, Args... args) {
    if (l <= debug_level_) {
#if defined(DEBUG_TWI)
        DEBUG_PRINT.start(8);
#endif
        debug_(a1, args...);
#if defined(DEBUG_TWI)
        DEBUG_PRINT.end();
#endif
    }
}

#else

#define debug(...)

#endif
