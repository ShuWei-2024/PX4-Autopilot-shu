#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise production DYP spawn/Run against separate CLI and worker file tables."""
from pathlib import Path
import subprocess, tempfile, sys
root=Path(__file__).resolve().parents[2]
source=Path(sys.argv[1]).read_text() if len(sys.argv)>1 else (root/'src/drivers/distance_sensor/dyp_a02/DypA02.cpp').read_text()
def extract(signature):
    start=source.index(signature); opening=source.index('{',start); depth=1; end=opening+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}'); end+=1
    return source[start:end]
spawn=extract('int DypA02::task_spawn('); spawn='static int spawn() { const char *left="L", *right="R";'+spawn[spawn.index('auto *instance'):]
run=extract('void Run() override').replace('void Run() override','void Run()').replace('::read(', 'mock_read(')
harness=r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include "Protocol.hpp"
static int group=1, opens=0, closes=0, reads=0;
static uint64_t clock_now=1000000;
static bool fail_left=false;
uint64_t hrt_absolute_time(){return clock_now;}
using hrt_abstime=uint64_t;
constexpr int PX4_ERROR=-1,PX4_OK=0,task_id_is_work_queue=-2;
struct distance_sensor_s {static constexpr int MODE_DISABLED=0;};
struct Range {float distance=0;int quality=0;void set_mode(int){} void update(uint64_t,float d,int q){distance=d;quality=q;}};
struct Channel {
 int fd=-1, owner=0; bool left=false;
 Range rangefinder;
 dyp_a02::Parser parser;
 dyp_a02::Sample sample;
 std::atomic<uint32_t> frames{0},checksum{0},partial_timeout{0},io_errors{0},latest_mm{UINT32_MAX},rx_bytes{0};
 std::atomic<int> last_error{0}; std::atomic<bool> uart_open{false};
 uint64_t next_open_attempt=0,last_publish=0;
};
class DypA02 {
public:
 static std::atomic<DypA02*> _object; static int _task_id;
 Channel _channels[2]; bool exiting=false,cleaned=false,scheduled=false;
 bool should_exit(){return exiting;} void ScheduleClear(){scheduled=false;} void exit_and_cleanup(){cleaned=true;}
 void ScheduleOnInterval(int i){assert(i==10000);scheduled=true;}
 bool configure_channel(unsigned i,const char*){_channels[i].left=i==0;return true;}
 bool open_channel(unsigned i,const char *p){configure_channel(i,p);return open_uart(_channels[i]);}
 bool open_uart(Channel &c){++opens;c.owner=group;c.fd=c.left?10:11;if(fail_left&&c.left){errno=EIO;return false;}c.uart_open=true;return true;}
 void close_uart(Channel &c){if(c.fd>=0){assert(c.owner==group);++closes;c.fd=-1;}c.uart_open=false;}
 long mock_read(int fd,void *buffer,unsigned){
  ++reads;Channel *c=nullptr;for(auto &candidate:_channels)if(candidate.fd==fd)c=&candidate;
  assert(c && c->owner==group);
  const uint8_t frame[]={0xff,0x01,0xf4,0xf4}; // 500 mm
  memcpy(buffer,frame,4);return 4;
 }
'''+spawn+run+r'''
};
std::atomic<DypA02*> DypA02::_object{nullptr}; int DypA02::_task_id=-1;
DypA02 *start(){group=1;int before=opens;assert(DypA02::spawn()==0);assert(before==opens);return DypA02::_object.load();}
int main(){
 auto *d=start();group=2;d->Run();assert(opens==2&&reads==2);
 for(auto &c:d->_channels){assert(c.frames==1&&c.rx_bytes==4&&c.latest_mm==500);assert(c.rangefinder.distance==0.5f);}
 d->Run();assert(opens==2&&reads==4);d->exiting=true;d->Run();assert(closes==2&&d->cleaned);delete d;
 d=start();group=2;d->exiting=true;d->Run();assert(opens==2&&closes==2);delete d;
 fail_left=true;d=start();group=2;d->Run();assert(opens==4&&closes==3);
 assert(d->_channels[0].io_errors==1&&d->_channels[0].last_error==EIO&&d->_channels[1].frames==1);
 d->Run();assert(opens==4);clock_now+=1000000;fail_left=false;d->Run();assert(opens==5&&d->_channels[0].frames==1);
 d->exiting=true;d->Run();assert(closes==5);delete d;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src=Path(tmp)/'check.cpp'; src.write_text(harness)
    binary=Path(tmp)/'check'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'src/drivers/distance_sensor/dyp_a02'),str(src),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('PASS: production spawn/Run use worker-owned UART; valid frame publication; close-on-stop; stop-before-open; per-side open failure/retry/recovery')
