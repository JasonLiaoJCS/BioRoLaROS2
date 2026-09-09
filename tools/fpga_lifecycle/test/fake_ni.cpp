// No NI library or network code is linked into this test executable.
#include "NiFpga.h"
#include "NiFpga_FPGA_POWER_RS485_v2.h"
#include "driver_lifecycle.hpp"
#include <map>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <unistd.h>
static std::map<uint32_t, uint32_t> regs;
static bool safe_phase = false;
static FILE* trace() {
    static FILE* file = fopen(getenv("FAKE_TRACE"), "a");
    return file;
}
static void record(const char* action, uint32_t reg=0, uint32_t value=0) {
    fprintf(trace(), "%s %u %u\n", action, reg, value); fflush(trace());
}
static int write_reg(uint32_t reg, uint32_t value) {
    if (lifecycle::stopping()) safe_phase = true;
    record(safe_phase ? "OFF_WRITE" : "WRITE", reg, value);
    if (safe_phase && getenv("FAKE_WRITE_FAIL")) return -1;
    regs[reg] = value; return 0;
}
static int read_reg(uint32_t reg, uint32_t& value) {
    value = regs[reg];
    if (safe_phase && getenv("FAKE_MISMATCH")) value = 1;
    record(safe_phase ? "OFF_READ" : "READ", reg, value);
    return safe_phase && getenv("FAKE_READ_FAIL") ? -1 : 0;
}
extern "C" {
NiFpga_Status NiFpga_Initialize(void) { record("INITIALIZE"); return 0; }
NiFpga_Status NiFpga_Finalize(void) { record("FINALIZE"); return 0; }
NiFpga_Status NiFpga_Open(const char*,const char*,const char*,uint32_t,NiFpga_Session* s) {
    record("OPEN"); *s=1; return getenv("FAKE_OPEN_FAIL") ? -1 : 0;
}
NiFpga_Status NiFpga_Run(NiFpga_Session,uint32_t) { record("RUN"); return 0; }
NiFpga_Status NiFpga_Close(NiFpga_Session,uint32_t) { record("CLOSE"); return 0; }
#define WRITE(T,N) NiFpga_Status NiFpga_Write##N(NiFpga_Session,uint32_t r,T v){return write_reg(r,v);}
#define READ(T,N) NiFpga_Status NiFpga_Read##N(NiFpga_Session,uint32_t r,T* v){uint32_t x;int s=read_reg(r,x);*v=x;return s;}
WRITE(NiFpga_Bool,Bool)
WRITE(uint16_t,U16)
WRITE(uint32_t,U32)
READ(NiFpga_Bool,Bool)
READ(uint16_t,U16)
READ(uint32_t,U32)
NiFpga_Status NiFpga_ReadI32(NiFpga_Session,uint32_t r,int32_t* v) {
    uint32_t x; int rc=read_reg(r,x); *v=x;
    return getenv("FAKE_MODULE_FAIL") ? -1 : rc;
}
NiFpga_Status NiFpga_ReadArrayU16(NiFpga_Session,uint32_t,uint16_t* a,size_t n) {
    for(size_t i=0;i<n;++i)a[i]=0; return 0;
}
NiFpga_Status NiFpga_ReadArrayU8(NiFpga_Session,uint32_t,uint8_t* a,size_t n) {
    for(size_t i=0;i<n;++i)a[i]=0; return 0;
}
NiFpga_Status NiFpga_WriteArrayU8(NiFpga_Session,uint32_t,const uint8_t*,size_t) {return 0;}
}
void important_message(std::string text) { fprintf(stderr,"%s\n",text.c_str()); }

#include <ncurses.h>
extern "C" int __real_wgetch(WINDOW*);
extern "C" int __wrap_wgetch(WINDOW* window) {
    return getenv("FAKE_GETCH_ERR") ? ERR : __real_wgetch(window);
}
extern "C" int __real_poll(pollfd*, nfds_t, int);
extern "C" int __wrap_poll(pollfd* fds, nfds_t count, int timeout) {
    if (getenv("FAKE_POLL_ERR") && count) { fds[0].revents = POLLERR; return 1; }
    return __real_poll(fds, count, timeout);
}
