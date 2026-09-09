#include "driver_lifecycle.hpp"
#include <cassert>
int main() {
    int fds[2]; assert(pipe(fds) == 0);
    assert(!lifecycle::endpoint_lost(fds[0], fds[1]));
    close(fds[1]);
    assert(lifecycle::endpoint_lost(fds[0], fds[0])); // EOF/POLLHUP
    close(fds[0]);
    assert(lifecycle::endpoint_lost(fds[0], fds[0])); // POLLNVAL
}
