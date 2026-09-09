#pragma once
#include <functional>
#include <vector>
#include <string>
#include <cstdlib>
namespace core {
inline std::vector<std::function<void()>>& callbacks() { static std::vector<std::function<void()>> cb; return cb; }
inline void spinOnce() { if (getenv("FAKE_COMMANDS")) for (auto& cb : callbacks()) cb(); }
template<class T> struct Publisher { void publish(const T&) {} };
template<class T> struct Subscriber {};
struct NodeHandler {
 template<class T> Publisher<T>& advertise(const char*) { static Publisher<T> p; return p; }
 template<class T> Subscriber<T>& subscribe(const char*, int, void(*cb)(T)) {
  static Subscriber<T> s; callbacks().push_back([cb]{cb(T{});}); return s;
 }
};
}
