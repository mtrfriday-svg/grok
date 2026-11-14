#ifndef MDEE_NET_H
#define MDEE_NET_H

#include <cstddef>
#include <cstdint>

namespace MdeeNet {
bool GetIp(char* buf, size_t len);
int Rssi();
void StartSntpOnce();
}

#endif
