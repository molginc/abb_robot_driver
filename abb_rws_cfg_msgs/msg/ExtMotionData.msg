#-------------------------------------------------------------------------------
# Description:
#   One 'External Motion Interface Data' instance (configuration type
#   'EXT_MOTION_DATA', topic Motion) from an ABB robot controller.
#
#   The instance configures the EGM control loop
#     speed = kp * (pos_ref - pos) + speed_ref
#   (see "Application manual - Externally Guided Motion", sections 2.3 and 4.1).
#
# Note:
#   A float64 field holds NaN, and 'level' holds -1, when the controller did
#   not report the corresponding attribute.
#-------------------------------------------------------------------------------

# Instance name (the 'ExtConfigName' argument of the EGMSetupXX RAPID instructions).
string name

# Level at which corrections are applied (0 = raw, 1 = filtering, 2 = path correction).
int32 level

# Default ramp time [s] for stopping External Motion Interface movements.
float64 ramp_time

# Default proportional position gain of the EGM position feedback control.
float64 kp

# Default bandwidth [Hz] of the low-pass filter applied to the EGM speed contribution.
float64 filter_bandwidth
