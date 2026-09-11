#pragma once
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
namespace recorder {
constexpr const char* version = "native-recorder-v2-20260911";
inline std::string quote(const std::string& value) {
    std::ostringstream s; s << '"';
    for (unsigned char c:value) {
        switch(c) {
            case '"': s << "\\\""; break; case '\\': s << "\\\\"; break;
            case '\n': s << "\\n"; break; case '\r': s << "\\r"; break; case '\t': s << "\\t"; break;
            default: if(c<32) s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
                     else s << c;
        }
    }
    return s.str()+'"';
}
inline std::string number(double value) {
    if(!std::isfinite(value)) return "null";
    std::ostringstream s;s<<std::setprecision(17)<<value;return s.str();
}
inline double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
