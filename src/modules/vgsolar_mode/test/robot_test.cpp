// SPDX-License-Identifier: BSD-3-Clause
#include "../VgsolarRobotLogic.hpp"
#include <cassert>
#include <cstdio>
using namespace vgsolar_robot;

static Config fast_config() {
	Config c{}; c.vacuum_kpa = -50.f; c.slew_us_s = 5000.f; c.lift_delay_ms = 20; c.vacuum_delay_ms = 500;
	c.vent_delay_ms = 100; c.lift_timeout_ms = 500; c.vacuum_debounce_ms = 20; c.action_timeout_ms = 2000;
	return c;
}
int main() {
	Config config = fast_config();
	assert(config.valid());
	Config invalid = config; invalid.slew_us_s = NAN; assert(!invalid.valid());
	uint64_t now = 1000000;
	Inputs input{true, true, true, true, -60.f};
	Suction cup; cup.configure(config);
	assert(cup.raised() && !cup.pump() && !cup.sealed());
	assert(cup.lower(now));
	for (unsigned i = 0; i < 100 && cup.state() != SuctionState::WaitDown; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.state() == SuctionState::WaitDown && !cup.pump());
	input.blade = false;
	for (unsigned i = 0; i < 20 && !cup.lowered(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.lowered() && cup.pump() && cup.sealed());
	cup.freeze(); assert(cup.frozen() && !cup.pump() && cup.sealed());
	now += 10000; cup.tick(now, true, input); assert(!cup.pump());
	cup.unfreeze(); assert(cup.pump());
	assert(cup.raise(now) && !cup.pump() && !cup.sealed());
	input.blade = true;
	for (unsigned i = 0; i < 200 && !cup.raised(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.pump() && !cup.sealed());

	// APM intentionally latches the first blade edge: after the blade passes the
	// optical slot, losing the level must not restart the lift extension timer.
	assert(cup.raise(now));
	input.blade = false;
	for (unsigned i = 0; i < 200 && cup.state() != SuctionState::WaitUp; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.state() == SuctionState::WaitUp);
	input.blade = true; now += 10000; cup.tick(now, true, input);
	input.blade = false;
	for (unsigned i = 0; i < 4; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.fault());

	// A disconnected/stale sensor fails closed during an action.
	assert(cup.lower(now)); input.fresh = false; now += 10000; cup.tick(now, true, input);
	assert(cup.fault() && !cup.pump() && !cup.sealed());
	assert(!cup.lower(now)); cup.clear_fault(); assert(cup.raised());

	// Low IR alone must not falsely prove that the lift lowered: it must first see a blade.
	input = {true, true, false, true, -60.f};
	assert(cup.lower(now));
	for (unsigned i = 0; i < 100 && !cup.fault(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.fault() && !cup.pump());
	cup.clear_fault();

	// Explicit sensor-disabled fallback reproduces timed suction and release.
	config.use_ir = false; config.use_pressure = false; cup.configure(config);
	assert(cup.lower(now)); input = {};
	for (unsigned i = 0; i < 150 && !cup.lowered(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.lowered());
	now += 10000; cup.tick(now, false, input);
	assert(!cup.raised() && !cup.pump() && !cup.sealed());
	for(unsigned i=0;i<150 && !cup.raised();++i) { now+=10000; cup.tick(now,true,input); }
	assert(cup.raised());

	// Loss of vacuum while holding revokes pump/valve permissions after debounce.
	config.use_ir = true; config.use_pressure = true; cup.configure(config);
	input = {true, true, true, true, -60.f}; assert(cup.lower(now));
	for (unsigned i = 0; i < 30; ++i) { now += 10000; cup.tick(now, true, input); }
	input.blade = false;
	for (unsigned i = 0; i < 10; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.lowered());
	input.pressure_kpa = -10.f;
	for (unsigned i = 0; i < 5; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(!cup.fault() && cup.vacuum_warning() && cup.pump() && cup.sealed());
 // Hysteresis retains warning through the dead band and clears at the establish threshold.
 input.pressure_kpa = -48.f;
 now += 10000; cup.tick(now, true, input); assert(cup.vacuum_warning());
 input.pressure_kpa = -60.f;
 now += 10000; cup.tick(now, true, input); assert(!cup.vacuum_warning());
 cup.freeze(); input.pressure_kpa = 0.f;
 for (unsigned i=0;i<5;++i) { now+=10000; cup.tick(now,true,input); }
 assert(cup.vacuum_warning() && cup.sealed() && !cup.pump() && !cup.fault());
 cup.unfreeze();
 const float lower_pwm = cup.pwm(); cup.emergency_release(now);
 assert(!cup.raised() && cup.state()==SuctionState::Venting && cup.pwm()==lower_pwm && !cup.sealed());
 const uint64_t vent_start = now;
 for(unsigned i=0;i<5;++i) { now+=10000; cup.emergency_release(now); cup.tick(now,true,input); }
 assert(now-vent_start<100000 && cup.pwm()==lower_pwm);
 input.blade = true;
 for(unsigned i=0;i<150 && !cup.raised();++i) { now+=10000; cup.tick(now,true,input); }
 assert(cup.raised() && !cup.fault());

	// Freeze & emergency_release across Lowering, WaitDown, Vacuum, Venting, Raising, WaitUp:
	// never jump PWM to an endpoint and never re-seal an already releasing cup.
	config = fast_config();
	config.slew_us_s = 1000.f; // 10 us per 10 ms tick so Lowering/Raising take multiple steps
	cup.configure(config);
	input = {true, true, true, true, -60.f};

	// Lowering -> freeze -> emergency_release -> Venting -> Raising -> Raised (no PWM jump)
	assert(cup.lower(now));
	now += 10000; cup.tick(now, true, input);
	const float mid_lower_pwm = cup.pwm();
	assert(mid_lower_pwm < config.raised_pwm && mid_lower_pwm > config.lowered_pwm);
	cup.freeze();
	assert(cup.frozen() && !cup.pump() && !cup.sealed() && cup.pwm() == mid_lower_pwm);
	cup.emergency_release(now);
	assert(!cup.frozen() && cup.state() == SuctionState::Venting && !cup.pump() && !cup.sealed() && cup.pwm() == mid_lower_pwm);
	for (unsigned i = 0; i < 200 && !cup.raised(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.fault());

	// WaitDown -> freeze -> emergency_release -> smooth release
	assert(cup.lower(now));
	for (unsigned i = 0; i < 150 && cup.state() != SuctionState::WaitDown; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.state() == SuctionState::WaitDown);
	cup.freeze();
	assert(cup.frozen() && !cup.pump() && !cup.sealed() && cup.pwm() == config.lowered_pwm);
	cup.emergency_release(now);
	assert(cup.state() == SuctionState::Venting && !cup.pump() && !cup.sealed() && cup.pwm() == config.lowered_pwm);
	for (unsigned i = 0; i < 200 && !cup.raised(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.fault());

	// Vacuum -> freeze -> emergency_release -> smooth release (never jumps to raised_pwm)
	assert(cup.lower(now));
	for (unsigned i = 0; i < 150 && cup.state() != SuctionState::WaitDown; ++i) { now += 10000; cup.tick(now, true, input); }
	input.blade = false;
	for (unsigned i = 0; i < 10 && cup.state() != SuctionState::Vacuum; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.state() == SuctionState::Vacuum && cup.pump() && cup.sealed());
	cup.freeze();
	assert(cup.frozen() && !cup.pump() && cup.pwm() == config.lowered_pwm);
	cup.emergency_release(now);
	assert(!cup.frozen() && cup.state() == SuctionState::Venting && !cup.pump() && !cup.sealed() && cup.pwm() == config.lowered_pwm);
	input.blade = true;
	for (unsigned i = 0; i < 200 && !cup.raised(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.fault());

	// Re-freeze while Venting & Raising: must not re-seal or jump PWM, and long freeze must not false-timeout.
	assert(cup.lower(now));
	for (unsigned i = 0; i < 150 && cup.state() != SuctionState::WaitDown; ++i) { now += 10000; cup.tick(now, true, input); }
	input.blade = false;
	for (unsigned i = 0; i < 30 && !cup.lowered(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.lowered());
	assert(cup.raise(now) && cup.state() == SuctionState::Venting);
	cup.freeze();
	assert(cup.frozen() && !cup.pump() && !cup.sealed() && cup.pwm() == config.lowered_pwm);
	now += 5000000; // > action_timeout_ms while frozen
	cup.tick(now, true, input);
	assert(!cup.fault());
	cup.emergency_release(now);
	assert(!cup.frozen() && cup.state() == SuctionState::Venting && !cup.pump() && !cup.sealed());
	for (unsigned i = 0; i < 50 && cup.state() != SuctionState::Raising; ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.state() == SuctionState::Raising);
	now += 10000; cup.tick(now, true, input);
	const float mid_raise_pwm = cup.pwm();
	assert(mid_raise_pwm > config.lowered_pwm && mid_raise_pwm < config.raised_pwm);
	cup.freeze();
	assert(cup.frozen() && !cup.pump() && !cup.sealed() && cup.pwm() == mid_raise_pwm);
	cup.emergency_release(now);
	assert(!cup.frozen() && cup.state() == SuctionState::Raising && cup.pwm() == mid_raise_pwm);
	input.blade = true;
	for (unsigned i = 0; i < 200 && !cup.raised(); ++i) { now += 10000; cup.tick(now, true, input); }
	assert(cup.raised() && !cup.fault());

	puts("PASS: suction sequence, ramp, IR transition, pressure debounce/loss, freeze, release, stale input and timed fallback");
}
