// SPDX-License-Identifier: BSD-3-Clause
// 仅测试AB相纯解码逻辑，不访问GPIO或连接实物。
#include "../QuadratureDecoder.hpp"
#include "../WheelKinematics.hpp"
#include <assert.h>
#include <cmath>
#include <stdio.h>

using vgsolar_encoders::QuadratureDecoder;

int main()
{
	QuadratureDecoder decoder;
	decoder.reset(0b00);
	assert(decoder.update(0b01) == 1);
	assert(decoder.update(0b11) == 1);
	assert(decoder.update(0b10) == 1);
	assert(decoder.update(0b00) == 1);
	assert(decoder.snapshot().count == 4);

	assert(decoder.update(0b10) == -1);
	assert(decoder.update(0b11) == -1);
	assert(decoder.update(0b01) == -1);
	assert(decoder.update(0b00) == -1);
	assert(decoder.snapshot().count == 0);

	// 同一相位不计数；单步有效反向边沿可撤销此前计数。
	assert(decoder.update(0b00) == 0);
	assert(decoder.update(0b01) == 1);
	assert(decoder.update(0b00) == -1);
	assert(decoder.snapshot().count == 0);

	// 两位同时变化无法判定方向，拒绝计数并单独增加诊断计数。
	assert(decoder.update(0b11) == 0);
	const auto result = decoder.snapshot();
	assert(result.count == 0);
	assert(result.transitions == 10);
	assert(result.invalid_transitions == 1);
	assert(decoder.update(0b10) == 1); // 跳变后以新相位恢复，不能沿用错误发生前的相位。

	// 从四个初始相位分别检查下一步的全部16种组合。
	const uint8_t cycle[] = {0b00, 0b01, 0b11, 0b10};
	for (unsigned start = 0; start < 4; ++start) {
		for (unsigned end = 0; end < 4; ++end) {
			decoder.reset(cycle[start]);
			const unsigned advance = (end + 4 - start) % 4;
			const int expected = advance == 1 ? 1 : (advance == 3 ? -1 : 0);
			assert(decoder.update(cycle[end]) == expected);
			assert(decoder.snapshot().count == expected);
			assert(decoder.snapshot().invalid_transitions == (advance == 2 ? 1u : 0u));
		}
	}

	// 两轮各自积累相反方向，启动时不在00也不应多计初始相位。
	QuadratureDecoder other;
	decoder.reset(0b11);
	other.reset(0b11);
	for (unsigned edge = 1; edge <= 100000; ++edge) {
		decoder.update(cycle[(2 + edge) % 4]);
		other.update(cycle[(2 + 400000 - edge) % 4]);
	}
	assert(decoder.snapshot().count == 100000);
	assert(other.snapshot().count == -100000);
	assert(decoder.snapshot().transitions == 100000 && decoder.snapshot().invalid_transitions == 0);
	decoder.reset(0b10);
	assert(decoder.snapshot().count == 0 && decoder.snapshot().transitions == 0);

	// Forty transitions per driven-wheel revolution maps four transitions in 100 ms to 1 rev/s.
	const auto wheel = vgsolar_encoders::calculate_wheel_kinematics(40, 4, 40, 100000, false);
	assert(wheel.configured);
	assert(std::fabs(wheel.angle_rad - static_cast<float>(vgsolar_encoders::TwoPi)) < 1e-5f);
	assert(std::fabs(wheel.speed_rad_s - static_cast<float>(vgsolar_encoders::TwoPi)) < 1e-5f);

	const auto reverse = vgsolar_encoders::calculate_wheel_kinematics(40, 4, 40, 100000, true);
	assert(std::fabs(reverse.angle_rad + static_cast<float>(vgsolar_encoders::TwoPi)) < 1e-5f);
	assert(std::fabs(reverse.speed_rad_s + static_cast<float>(vgsolar_encoders::TwoPi)) < 1e-5f);

	const auto uncalibrated = vgsolar_encoders::calculate_wheel_kinematics(40, 4, 0, 100000, false);
	assert(!uncalibrated.configured && std::isnan(uncalibrated.angle_rad) && std::isnan(uncalibrated.speed_rad_s));
	const auto first_sample = vgsolar_encoders::calculate_wheel_kinematics(40, 4, 40, 0, false);
	assert(first_sample.configured && std::isfinite(first_sample.angle_rad) && std::isnan(first_sample.speed_rad_s));

	// One driven-wheel revolution per second at the historical initial radius.
	using vgsolar_encoders::calculate_linear_speed;
	assert(std::fabs(calculate_linear_speed(wheel.speed_rad_s, 0.0404f, true) - 0.253840686f) < 1e-6f);
	assert(std::fabs(calculate_linear_speed(reverse.speed_rad_s, 0.0404f, true) + 0.253840686f) < 1e-6f);
	assert(calculate_linear_speed(0.f, 0.0404f, true) == 0.f);
	assert(std::isnan(calculate_linear_speed(wheel.speed_rad_s, 0.f, true)));
	assert(std::isnan(calculate_linear_speed(wheel.speed_rad_s, -1.f, true)));
	assert(std::isnan(calculate_linear_speed(wheel.speed_rad_s, NAN, true)));
	assert(std::isnan(calculate_linear_speed(wheel.speed_rad_s, INFINITY, true)));
	assert(std::isnan(calculate_linear_speed(uncalibrated.speed_rad_s, 0.0404f, true)));
	assert(std::isnan(calculate_linear_speed(first_sample.speed_rad_s, 0.0404f, true)));
	assert(std::isnan(calculate_linear_speed(wheel.speed_rad_s, 0.0404f, false)));

	vgsolar_encoders::WheelDistance distance;
	distance.update(100, 100, 40, 0.0404f, false, true);
	assert(distance.distance() == 0.f && distance.path() == 0.f);
	distance.update(140, 140, 40, 0.0404f, false, true);
	assert(std::fabs(distance.distance() - 0.253840686f) < 1e-6f);
	distance.update(100, 180, 40, 0.0404f, false, true);
	assert(std::fabs(distance.distance()) < 1e-6f);
	assert(std::fabs(distance.path() - 2.f * 0.253840686f) < 1e-6f);
	// Forward and back within one sample still increase traveled path.
	distance.update(100, 220, 40, 0.0404f, false, true);
	assert(std::fabs(distance.path() - 3.f * 0.253840686f) < 1e-6f);
	distance.update(100, 220, 40, 0.0404f, true, true); // direction change starts a new baseline
	assert(distance.distance() == 0.f && distance.path() == 0.f);
	distance.update(140, 260, 40, 0.0404f, true, true);
	assert(std::fabs(distance.distance() + 0.253840686f) < 1e-6f);
	assert(std::fabs(distance.path() - 0.253840686f) < 1e-6f);
	distance.update(140, 260, 0, 0.0404f, true, true);
	assert(std::isnan(distance.distance()) && std::isnan(distance.path()));
	distance.update(200, 320, 40, 0.0404f, false, true); // uncalibrated movement is not backfilled
	assert(distance.distance() == 0.f && distance.path() == 0.f);
	distance.update(200, 320, 80, 0.0404f, false, true); // CPR change
	assert(distance.path() == 0.f);
	distance.update(200, 320, 80, 0.0808f, false, true); // radius change
	assert(distance.path() == 0.f);
	distance.update(200, 320, 80, 0.0808f, false, false);
	assert(std::isnan(distance.path()));
	distance.update(200, UINT32_MAX - 1, 40, 0.0404f, false, true);
	distance.update(204, 2, 40, 0.0404f, false, true); // uint32 transition rollover
	assert(std::fabs(distance.path() - 0.0253840686f) < 1e-6f);
	distance.update(204, 2, 40, NAN, false, true);
	assert(std::isnan(distance.path()));

	puts("PASS: quadrature transitions/recovery, 100k signed edges, wheel CPR/direction/speed conversion, NaN handling, signed travel/path, reversal, calibration reset, and rollover");
}
