#ifndef MDEE_UTIL_H
#define MDEE_UTIL_H

#include <esp_timer.h>

#include <cstddef>
#include <cstdint>

inline int64_t MdeeNowMs() { return esp_timer_get_time() / 1000; }

inline int MdeeClamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float MdeeClampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Copy without snprintf so an unknown source length cannot trip -Wformat-truncation.
inline void MdeeCopy(char* dst, size_t dst_len, const char* src) {
    if (dst_len == 0) return;
    size_t i = 0;
    if (src != nullptr) {
        while (src[i] != '\0' && (i + 1) < dst_len) {
            dst[i] = src[i];
            i++;
        }
    }
    dst[i] = '\0';
}

inline void MdeeAppend(char* dst, size_t dst_len, const char* src) {
    if (dst_len == 0) return;
    size_t used = 0;
    while (used + 1 < dst_len && dst[used] != '\0') used++;
    MdeeCopy(dst + used, dst_len - used, src);
}

#endif
