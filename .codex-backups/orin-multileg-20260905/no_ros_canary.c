#include <unistd.h>
static void reject(void) {
    static const char message[]="[CANARY FAILURE] read-only check attempted ROS initialization or publication\n";
    write(2,message,sizeof(message)-1);
    _exit(98);
}
int rcl_init(void) { reject(); return 0; }
int rmw_init(void) { reject(); return 0; }
int rcl_node_init(void) { reject(); return 0; }
int rcl_publisher_init(void) { reject(); return 0; }
int rcl_publish(void) { reject(); return 0; }
int socket(int domain, int type, int protocol) { (void)domain;(void)type;(void)protocol;reject(); return -1; }
