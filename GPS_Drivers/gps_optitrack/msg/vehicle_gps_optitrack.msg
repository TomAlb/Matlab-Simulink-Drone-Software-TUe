# GPS optitrack state.
# the auto-generated field 'timestamp' is for the position & velocity (microseconds)
uint32 frame_number			# Optitrack Frame Number
uint8 rigid_body_ID                     # Rigid body ID
float32 pos_x                           # Optitarck X-position, (metres)
float32 pos_y                           # Optitarck Y-position, (metres)
float32 pos_z                           # Optitarck Z-position, (metres)
float32 orientation_qx                  # Optitarck qx-orientation
float32 orientation_qy                  # Optitarck qy-orientation
float32 orientation_qz                  # Optitarck qz-orientation
float32 orientation_qw                  # Optitarck qw-orientation
float32 mean_error                      # Optitarck position mean error

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

