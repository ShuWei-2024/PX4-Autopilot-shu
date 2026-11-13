uint64 timestamp					# publication time since system start (microseconds)
uint64 timestamp_sample				# latest accepted quadrature edge

int64 right_count				# signed raw transitions; not wheel revolutions
int64 left_count
int32 right_delta				# count change since previous publication
int32 left_delta
uint32 right_transitions			# accepted phase transitions
uint32 left_transitions
uint32 right_invalid_transitions		# two-bit phase jumps rejected by decoder
uint32 left_invalid_transitions
int8 right_direction				# -1, 0 (no movement in sample), +1 phase order
int8 left_direction
bool capture_active				# GPIO edge callbacks are installed

float32 right_speed_m_s			# signed wheel surface speed; NaN if uncalibrated/inactive
float32 left_speed_m_s			# requires driven-wheel CPR and positive effective radius

float32 right_distance_m                         # signed travel since calibration/start baseline; NaN if invalid
float32 left_distance_m
float32 right_path_m                             # total wheel surface travel, forward and reverse both add
float32 left_path_m
