#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check production named telemetry for encoder speed, travel, freshness and pressure coexistence."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[4]
header=root/'src/modules/mavlink/streams/NAMED_VALUE_FLOAT.hpp'
code='\n'.join(line for line in header.read_text().splitlines() if not line.startswith('#include'))
stub=r'''
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <vector>
#define VGSOLAR_PRESSURE_ADC_CHANNEL 1
#define PX4_ISFINITE(x) std::isfinite(x)
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN 18
#define MAVLINK_NUM_NON_PAYLOAD_BYTES 12
struct debug_key_value_s { uint64_t timestamp{}; char key[10]{}; float value{}; };
struct vgsolar_inputs_s { bool enabled{}, pressure_valid{}; uint64_t timestamp{}; float pressure_kpa{}; };
struct vgsolar_encoder_status_s { bool capture_active{}; uint64_t timestamp{}; float right_speed_m_s{}, left_speed_m_s{}; float right_distance_m{2.f}, left_distance_m{4.f}, right_path_m{6.f}, left_path_m{8.f}; };
struct mavlink_named_value_float_t { uint32_t time_boot_ms{}; char name[10]{}; float value{}; };
static std::vector<mavlink_named_value_float_t> packets;
static uint64_t clock_now=1000000;
uint64_t hrt_absolute_time() { return clock_now; }
void mavlink_msg_named_value_float_send_struct(int, const mavlink_named_value_float_t *m) { packets.push_back(*m); }
struct Mavlink { int get_channel() { return 0; } };
struct MavlinkStream {
 Mavlink *_mavlink;
 explicit MavlinkStream(Mavlink *m): _mavlink(m) {}
 virtual ~MavlinkStream()=default;
 virtual const char *get_name() const=0;
 virtual uint16_t get_id()=0;
 virtual unsigned get_size()=0;
 virtual bool send()=0;
};
enum { debug_key_value, vgsolar_inputs, vgsolar_encoder_status };
#define ORB_ID(x) x
static bool advertised[3]{};
static debug_key_value_s key_data;
static vgsolar_inputs_s input_data;
static vgsolar_encoder_status_s encoder_data;
namespace uORB { class Subscription {
 int id;
public:
 explicit Subscription(int i):id(i) {}
 bool advertised() const { return ::advertised[id]; }
 bool copy(debug_key_value_s *d) { *d=key_data; return advertised(); }
 bool copy(vgsolar_inputs_s *d) { *d=input_data; return advertised(); }
 bool copy(vgsolar_encoder_status_s *d) { *d=encoder_data; return advertised(); }
 bool update(debug_key_value_s *d) { if (!advertised()) return false; *d=key_data; ::advertised[id]=false; return true; }
}; }
'''
test=r'''
float value(const char *name) { for (const auto &p: packets) if (strcmp(p.name,name)==0) return p.value; assert(false); return NAN; }
int main() {
 Mavlink mav;
 auto *stream=MavlinkStreamNamedValueFloat::new_instance(&mav);
 advertised[1]=advertised[2]=true;
 input_data={true,true,clock_now,-49.f};
 encoder_data={true,clock_now,0.3f,-0.1f};
 assert(stream->get_size()==12*30);
 assert(stream->send());
 assert(packets.size()==12);
 assert(value("SUCT_PKPA")==-49.f && value("SUCT_HLT")==1.f);
 assert(value("ENCR_MPS")==0.3f && value("ENCL_MPS")==-0.1f);
 assert(value("ENCR_M")==2.f && value("ENCL_M")==4.f && value("ENCC_M")==3.f);
 assert(value("ENCR_TRIP")==6.f && value("ENCL_TRIP")==8.f && value("ENC_TRIP")==7.f);
 assert(std::fabs(value("ENCC_MPS")-0.1f)<1e-6f && value("ENC_VALID")==1.f);
 packets.clear(); clock_now+=50000;
 assert(!stream->send() && packets.empty()); // rate gate
 clock_now+=50000; encoder_data.timestamp=clock_now; encoder_data.left_speed_m_s=NAN;
 assert(stream->send() && packets.size()==10);
 assert(value("ENCR_MPS")==0.3f && std::isnan(value("ENCL_MPS")));
 assert(std::isnan(value("ENCC_MPS")) && value("ENC_VALID")==0.f);
 packets.clear(); clock_now+=600000; // stale even while previously active
 assert(stream->send());
 assert(std::isnan(value("ENCR_MPS")) && value("ENC_VALID")==0.f);
 assert(std::isnan(value("ENCC_M")) && std::isnan(value("ENC_TRIP")));
 assert(std::isnan(value("SUCT_PKPA")) && value("SUCT_HLT")==0.f);
 packets.clear(); clock_now+=100000; encoder_data={false,clock_now,0.f,0.f};
 assert(stream->send()); assert(std::isnan(value("ENCR_MPS")));
 packets.clear(); clock_now+=100000; encoder_data={true,clock_now+1,0.f,0.f};
 assert(stream->send()); assert(value("ENC_VALID")==0.f); // future timestamp
 packets.clear(); clock_now+=100000; encoder_data={true,clock_now,0.f,0.f};
 advertised[0]=true; key_data.timestamp=clock_now; strcpy(key_data.key,"OTHER"); key_data.value=7.f;
 assert(stream->send()); assert(value("ENC_VALID")==1.f && value("ENCC_MPS")==0.f && value("OTHER")==7.f);
 delete stream;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src=Path(tmp)/'test.cpp'; binary=Path(tmp)/'test'
    src.write_text(stub+code+test)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(src),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('PASS: actual NAMED_VALUE_FLOAT header: routing, mean, per-side NaN, rate, stale/future/inactive, pressure and debug coexistence')
