#pragma once



#define TOGAN_HOLD_SOLENOID_TIME_MS						100
#define TOGAN_WAIT_BETWEEN_SOLENOID_FIRES_MS			1000

#define TOGAN_DEFAULT_INTER_RELEASE_TIME_MS				1000 // 06_11_2021 testlerinde gecici olarak 2000 yapildi. orjinali 1000
#define TOGAN_MIN_INTER_RELEASE_TIME_MS					1000
#define TOGAN_MAX_INTER_RELEASE_TIME_MS					10000

#define TOGAN_DEFAULT_SLAVE_DELTA_RELEASE_TIME_MS		100
#define TOGAN_MIN_SLAVE_DELTA_RELEASE_TIME_MS 			TOGAN_DEFAULT_SLAVE_DELTA_RELEASE_TIME_MS
#define TOGAN_MAX_SLAVE_DELTA_RELEASE_TIME_MS			2000

#define TAPA_ERR_BASE  25000

#define HURKUS_C_LEFT_WING_RT_ADDR 29
#define HURKUS_C_RIGHT_WING_RT_ADDR 15
#define ALBATROS_RT_ADDR 1

#define JETTISON_ALL_GRIPPERS  15    // tum gripperlar jettison icin


#define NUM_MODEL  8
#define NUM_PARAMETRE  192

#define KT2MS 0.51444444444f
#define MS2KT   1.943844492440605f
#define FT_M 0.3048f
#define dlzTimeAdvance_s 3.0f // DLZ entry release pointten 3 saniye one aliniyor
#define DLZTIMEDELAY_SEC 2.0f
#define tossTimeAdvance_s 10.0f // IP release pointten 10 saniye one aliniyor
#define dlzToleranceAngle_deg 60.0f // DLZ valid vermek icin maximum steering angle degeri
#define ctssAdvanceTime_ms 300.0f // komut ile gripper acma arasındaki gecikmeyi tolere eder.
#define S_MS 1000.0f
#define kKilopascalToPascal  1000.0f
// DLL internal variables:
#define kDegreesToRadians   M_PI_F / 180.0f
#define kRadiansToDegrees   (float)(180.0f / M_PI_F)
#define iForm   1.0f
#define dia_m   81e-3f
#define mass_kg   3.125f
#define latitude_rads   36.0f * M_PI_F / 180.0f
#define rEarth_m   6.356766e6
#define omegaEarth_rads   7.292115e-5f
#define kMsnToKnot   1.943844492440605f
#define kKnotToMsn   0.51444444444444
#define kFeetToMeter   0.3048
#define kMeterToFeet   3.280839895013123
#define localmomentarmx_m   0.0 // istasyondan ilk atılacak mühimmata olan moment kolu
#define localmomentarmy_m   0.0
#define localmomentarmz_m   0.0

#define DEZ_ARC_WIDTH_DEG   60
#define DLZ_ENTRY_ARC_WIDTH_DEG 45
#define DLZ_EXIT_ARC_WIDTH_DEG 45


#define SATE_CLEAR_RADIUS_METER 50
#define SATE_UNCLEAR_RADIUS_METER 10

#define MIN_VEL_DRONES_MS   5
#define MAX_VEL_DRONES_MS   500

typedef enum {
	PLATFORM_UNKNOWN,
	PLATFORM_HURKUS_LEFT,
	PLATFORM_HURKUS_RIGHT,
	PLATFORM_DRONES

}E_TOGAN_PLATFORMS;


#define TOGAN_YAZILIM_REV	"G0.6____"

#define TOGAN_MASTER_CARD_IP "192.168.1.50"
#define TOGAN_SLAVE_CARD_IP "192.168.1.60"

#define TOGAN_MASTER 0
#define TOGAN_SLAVE 1

#define ALBATROS_PLT_IP_ADDR "192.168.1.200"
#define ALBATROS_RX_PORT	2006
#define ALBATROS_TX_PORT	2005

#define EMULATOR_ARDUINO_IP_ADDR "192.168.1.222"
#define EMULATOR_ARDUINO_TX_PORT	21000
#define EMULATOR_ARDUINO_RX_PORT	21000


#define SLAVE_CARD_LISTEN_PORT 2010
#define MASTER_CARD_LISTEN_PORT 2011

#define BAKIS_DISABLED 0



typedef unsigned char  			BYTE;
typedef unsigned char      		BOOLEAN;                  /* Boolean ifadesi tanimlamasi                                     */
typedef unsigned char      		INT08U;                    /* 8 bit isaretsiz integer tanimlamasi                             */
typedef signed char        		INT08S;                    /* 8 bit isaretli  integer tanimlamasi                             */
typedef unsigned short     		INT16U;                   /* 16 bit isaretsiz  integer tanimlamasi                           */
typedef signed   short     		INT16S;                   /* 16 bit isaretli   integer tanimlamasi                           */
typedef unsigned int       		INT32U;                   /* 32 bit isaretsiz  integer tanimlamasi                           */
typedef signed   int       		INT32S;                   /* 32 bit isaretli   integer tanimlamasi                           */
typedef float              		FP32;                     /* 32 bit floating point tanimlamasi                               */
typedef long double        		FP40;                     /* 40 bit floating point tanimlamasi,donanimda olmadigi icin 32 bit*/
typedef long double        		FP64;                     /* 64 bit floating point tanimlamasi                               */
typedef float		          	GUDUM_F;                  /* 64 bit GUDUM float  tanimlamasi                                 */

typedef unsigned long long 		INT64U;                  /*  64 bit isaretsiz integer  tanimlamasi                           */
typedef signed long long   		INT64S;                  /*  64 bit isaretli integer  tanimlamasi                            */
typedef INT64U             		CPU_CYCLE;               /*  Islemci cycle counter tanimlamasi                               */
typedef char               		CHAR;                    /*  Karakter tanimlamasi                                            */

typedef FP32                	CPU_ZMN;                /*  CPU_ZMN tanimlamasi                                             */

typedef char*               	PCHAR;                    /*  Karakter tanimlamasi                                            */
typedef unsigned char      		INT8U;                    /* 8 bit isaretsiz integer tanimlamasi                             */
typedef signed char             INT8S;                    /* 8 bit isaretli  integer tanimlamasi                             */

#define SEMICIRCLE_TO_RAD  M_PI_F
#define RAD_TO_SEMICIRCLE  (0.318309886183791f)
#define RAD_TO_DEG	  	   (57.295779513082323f)
#define DEG_TO_RAD	  	   (0.017453292519943f)

#define FEET_TO_METRE      ( 0.3048f )
#define METER_TO_FEET      ( 3.280839895013123f )
#define METER_PER_SEC_TO_MACH (0.003389297412f)
