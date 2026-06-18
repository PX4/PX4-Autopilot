#ifndef MAGIC_NUMBER_H
#define MAGIC_NUMBER_H

// Magic numbers in sorted order

#define VIO_MAGIC_NUMBER            (0x05455524)
#define VFC_MAGIC_NUMBER            (0x05455525)
#define OT_MAGIC_NUMBER             (0x05455526)

// Spells "BARO" in ASCII
#define BARO_MAGIC_NUMBER           (0x4241524F)

// Spells "CRSF" in ASCII
#define CRSF_RAW_MAGIC_NUMBER       (0x43525346)

// Spells "CPU2" in ASCII
#define CPU_STATS2_MAGIC_NUMBER     (0x43505532)

// Spells "IONB" in ASCII
#define MPA_ION_BUFFER_MAGIC_NUMBER (0x494F4E42)

// Spells "PAYL" in ASCII
#define PAYLOAD_STATUS_MAGIC_NUMBER (0x5041594C)

// Spells "RCCH" in ASCII
#define RC_CHANNELS_MAGIC_NUMBER    (0x52434348)

// These all spell "VOXL" in ASCII. If these were to be cast as a float it would
// have a value of 5.7x10^13 which is an impossible value for translations/rotation
// readings making it unique as an identifier.
#define CAMERA_MAGIC_NUMBER         (0x564F584C)
#define TOF_MAGIC_NUMBER            (0x564F584C)
#define IMU_MAGIC_NUMBER            (0x564F584C)
#define POINT_CLOUD_MAGIC_NUMBER    (0x564F584C)
#define TAG_DETECTION_MAGIC_NUMBER  (0x564F584C)
#define POSE_4DOF_MAGIC_NUMBER      (0x564F584C)
#define POSE_VEL_6DOF2_MAGIC_NUMBER (0x564F584C)
#define CPU_MON_MAGIC_NUMBER        (0x564F584C)

#define TOF2_MAGIC_NUMBER           (0x564F584D)

// Spells "POSE" in ASCII
#define POSE_VEL_6DOF_MAGIC_NUMBER  (0x706F7365)

#endif // MAGIC_NUMBER_H
