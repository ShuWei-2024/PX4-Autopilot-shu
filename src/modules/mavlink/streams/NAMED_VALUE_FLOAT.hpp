/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef NAMED_VALUE_FLOAT_HPP
#define NAMED_VALUE_FLOAT_HPP

#include <uORB/topics/debug_key_value.h>

#include <board_config.h>
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
#include <uORB/topics/vgsolar_inputs.h>
#include <uORB/topics/vgsolar_encoder_status.h>
#endif

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNamedValueFloat(mavlink); }

	static constexpr const char *get_name_static() { return "NAMED_VALUE_FLOAT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		unsigned count = _debug_key_value_sub.advertised() ? 1 : 0;
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
		count += _suction_sub.advertised() ? 2 : 0;
		count += _encoder_sub.advertised() ? 10 : 0;
#endif
		return count * (MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
	uORB::Subscription _suction_sub{ORB_ID(vgsolar_inputs)};
	uORB::Subscription _encoder_sub{ORB_ID(vgsolar_encoder_status)};
	uint64_t _last_suction{0};
	uint64_t _last_encoder{0};
#endif

	void send_value(uint64_t timestamp, const char *name, float value)
	{
		mavlink_named_value_float_t msg{};
		msg.time_boot_ms = timestamp / 1000ULL;
		strncpy(msg.name, name, sizeof(msg.name) - 1);
		msg.value = value;
		mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);
	}

	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _debug_key_value_sub{ORB_ID(debug_key_value)};

	bool send() override
	{
		bool sent = false;
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
		const uint64_t now = hrt_absolute_time();

		if (_suction_sub.advertised() && now - _last_suction >= 500000) {
			vgsolar_inputs_s input{};
			_suction_sub.copy(&input);
			_last_suction = now;
			const bool valid = input.enabled && input.pressure_valid && input.timestamp > 0 && input.timestamp <= now
					   && now - input.timestamp < 500000 && PX4_ISFINITE(input.pressure_kpa);
			send_value(now, "SUCT_PKPA", valid ? input.pressure_kpa : NAN);
			send_value(now, "SUCT_HLT", valid ? 1.f : 0.f);
			sent = true;
		}

		if (_encoder_sub.advertised() && now - _last_encoder >= 100000) {
			vgsolar_encoder_status_s encoder{};
			_encoder_sub.copy(&encoder);
			_last_encoder = now;
			// Publication freshness is not proof of physical encoder connectivity.
			const bool fresh = encoder.capture_active && encoder.timestamp > 0 && encoder.timestamp <= now
					   && now - encoder.timestamp < 500000;
			const float right = fresh ? encoder.right_speed_m_s : NAN;
			const float left = fresh ? encoder.left_speed_m_s : NAN;
			const bool valid = PX4_ISFINITE(right) && PX4_ISFINITE(left);
			send_value(now, "ENCR_MPS", right);
			send_value(now, "ENCL_MPS", left);
			send_value(now, "ENCC_MPS", valid ? 0.5f * (right + left) : NAN);
			send_value(now, "ENC_VALID", valid ? 1.f : 0.f);
			const float right_distance = fresh ? encoder.right_distance_m : NAN;
			const float left_distance = fresh ? encoder.left_distance_m : NAN;
			const float right_path = fresh ? encoder.right_path_m : NAN;
			const float left_path = fresh ? encoder.left_path_m : NAN;
			send_value(now, "ENCR_M", right_distance);
			send_value(now, "ENCL_M", left_distance);
			send_value(now, "ENCC_M", 0.5f * (right_distance + left_distance));
			send_value(now, "ENCR_TRIP", right_path);
			send_value(now, "ENCL_TRIP", left_path);
			send_value(now, "ENC_TRIP", 0.5f * (right_path + left_path));
			sent = true;
		}
#endif

		debug_key_value_s debug;

		if (_debug_key_value_sub.update(&debug)) {
			mavlink_named_value_float_t msg{};
			msg.time_boot_ms = debug.timestamp / 1000ULL;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;
			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

#endif // NAMED_VALUE_FLOAT_HPP
