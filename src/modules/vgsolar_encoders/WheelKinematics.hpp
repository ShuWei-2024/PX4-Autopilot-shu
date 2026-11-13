// SPDX-License-Identifier: BSD-3-Clause
// Convert calibrated quadrature counts to wheel-shaft angle and angular speed.
#pragma once

#include <math.h>
#include <cmath>
#include <stdint.h>

namespace vgsolar_encoders
{
static constexpr double TwoPi = 6.28318530717958647692;

struct WheelKinematics {
	float angle_rad;
	float speed_rad_s;
	bool configured;
};

inline float calculate_linear_speed(float speed_rad_s, float radius_m, bool capture_active)
{
	return capture_active && std::isfinite(speed_rad_s) && std::isfinite(radius_m) && radius_m > 0.f ?
	       speed_rad_s * radius_m : NAN;
}

// Accumulate only calibrated samples; never reinterpret earlier counts with a new scale.
class WheelDistance
{
public:
	void update(int64_t count, uint32_t transitions, int32_t cpr, float radius, bool reverse, bool active)
	{
		_valid = active && cpr > 0 && std::isfinite(radius) && radius > 0.f;

		if (!_valid) {
			_initialized = false;
			return;
		}

		if (!_initialized || cpr != _cpr || (radius > _radius || radius < _radius) || reverse != _reverse) {
			_distance = 0.;
			_path = 0.;
			_initialized = true;
		} else {
			const double meters_per_count = TwoPi * static_cast<double>(radius) / cpr;
			_distance += (static_cast<double>(count) - static_cast<double>(_previous_count)) * meters_per_count *
				     (reverse ? -1. : 1.);
			// Accepted transitions count every edge, including reversal within one publication interval.
			_path += static_cast<uint32_t>(transitions - _previous_transitions) * meters_per_count;
		}

		_previous_count = count;
		_previous_transitions = transitions;
		_cpr = cpr;
		_radius = radius;
		_reverse = reverse;
	}

	float distance() const { return _valid ? static_cast<float>(_distance) : NAN; }
	float path() const { return _valid ? static_cast<float>(_path) : NAN; }

private:
	int64_t _previous_count{0};
	uint32_t _previous_transitions{0};
	int32_t _cpr{0};
	float _radius{0.f};
	double _distance{0.};
	double _path{0.};
	bool _reverse{false};
	bool _initialized{false};
	bool _valid{false};
};

inline WheelKinematics calculate_wheel_kinematics(int64_t count, int64_t delta, int32_t counts_per_revolution,
		uint64_t interval_us, bool reverse_direction)
{
	const float unavailable = NAN;

	if (counts_per_revolution <= 0) {
		return {unavailable, unavailable, false};
	}

	const double direction = reverse_direction ? -1.0 : 1.0;
	const double radians_per_count = TwoPi / static_cast<double>(counts_per_revolution);
	const float angle = static_cast<float>(static_cast<double>(count) * radians_per_count * direction);
	const float speed = interval_us > 0 ? static_cast<float>(static_cast<double>(delta) * radians_per_count * 1e6 /
			static_cast<double>(interval_us) * direction) : unavailable;
	return {angle, speed, true};
}
}
