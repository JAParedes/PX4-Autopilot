# Fused local position in NED.

uint64 timestamp			# time since system start (microseconds)

bool    beta_mot_fr_sw			# Switch for gain that limits the QC FR motor PWM signal
float32 beta_mot_fr			# Gain that limits the QC FR motor PWM signal

float32[6] actuator_pwm_out		# MC Actuator PWM out
