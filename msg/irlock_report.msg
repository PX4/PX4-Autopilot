# IRLOCK_REPORT message data

uint16 signature

float32 pos_x # tan(theta), where theta is the angle between the target and the camera center of projection in x-axis
float32 pos_y # tan(theta), where theta is the angle between the target and the camera center of projection in y-axis
float32 size_x #/** size of target along x-axis in units of tan(theta) **/
float32 size_y #/** size of target along y-axis in units of tan(theta) **/
