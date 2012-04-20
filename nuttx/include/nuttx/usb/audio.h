/********************************************************************************************
 * include/nuttx/usb/audio.h
 * Audio Device Class (ADC) definitions
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:  This header file is based on information provided by the
 * documents for the Audio v2.0 package:
 *
 * 1. Universal Serial Bus Device Class Definition for Audio Devices, Release
 *    2.0, May 31, 2006, 
 * 2. Universal Serial Bus Device Class Definition for Audio Data Formats,
 *    Release 2.0, May 31, 2006
 * 3. Universal Serial Bus Device Class Definition for Terminal Types,\
 *    Release 2.0, May 31, 2006,
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_AUDIO_H
#define __INCLUDE_NUTTX_USB_AUDIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>

/********************************************************************************************
 * Preprocessor definitions
 ********************************************************************************************/
/* Device Descriptor
 *
 * Because audio functionality is always considered to reside at the interface level, this
 * class specification does not define a specific audio device descriptor. For both composite
 * devices and audio-only devices, the device descriptor must indicate that class information
 * is to be found at the interface level. Therefore, the bDeviceClass, bDeviceSubClass and
 * bDeviceProtocol fields of the device descriptor must contain the values 0xef, 0x02, and
 * 0x01 respectively.
 */

#define ADC_DEVICE_CLASS            USB_CLASS_MISC
#define ADC_DEVICE_SUBCLASS         0x02
#define ADC_DEVICE_PROTOCOL         0x01

/* Audio Interface Class Code (defined in usb.h) */

#define ADC_CLASS                   USB_CLASS_AUDIO

/* Audio Interface Sub-class Codes */

#define ADC_SUBCLASS_UNDEF          0x00       /* Undefined */
#define ADC_SUBCLASS_AUDIOCONTROL   0x01       /* Audio control */
#define ADC_SUBCLASS_AUDIOSTREAMING 0x02       /* Audio streaming */
#define ADC_SUBCLASS_MIDISTREAMING  0x03       /* MIDI streaming */

/* Audio Protocol Codes */

#define ADC_PROTOCOL_UNDEF          0x00       /* Undefined */
#define ADC_PROTOCOLv20             0x20       /* IP version 2.0 */

/* Interface Association Descriptor (IAD) */

#define ADC_IAD_CLASS               ADC_CLASS
#define ADC_IAD_SUBCLASS            ADC_SUBCLASS_UNDEF
#define ADC_IAD_PROTOCOL            ADC_PROTOCOLv20

/* Standard AC Interface Descriptor */

#define ADC_ACIF_CLASS              ADC_CLASS
#define ADC_ACIF_SUBCLASS           ADC_SUBCLASS_AUDIOCONTROL
#define ADC_ACIF_PROTOCOL           ADC_PROTOCOLv20

/* Standard AS Interface Descriptor */

#define ADC_ASIF_CLASS              ADC_CLASS
#define ADC_ASIF_SUBCLASS           ADC_SUBCLASS_AUDIOSTREAMING
#define ADC_ASIF_PROTOCOL           ADC_PROTOCOLv20

/* Class-specific Descriptor Codes: */

#define ADC_CS_UNDEF                0x20
#define ADC_CS_DEVICE               0x21
#define ADC_CS_CONFIGURATION        0x22
#define ADC_CS_STRING               0x23
#define ADC_CS_INTERFACE            0x24
#define ADC_CS_ENDPOINT             0x25

/* Audio Class-Specific AC Interface Descriptor Subtypes */

#define ADC_AC_UNDEF                0x00
#define ADC_AC_HEADER               0x01
#define ADC_AC_INPUT_TERMINAL       0x02
#define ADC_AC_OUTPUT_TERMINAL      0x03
#define ADC_AC_MIXER_UNIT           0x04
#define ADC_AC_SELECTOR_UNIT        0x05
#define ADC_AC_FEATURE_UNIT         0x06
#define ADC_AC_EFFECT_UNIT          0x07
#define ADC_AC_PROCESSING_UNIT      0x08
#define ADC_AC_EXTENSION_UNIT       0x09
#define ADC_AC_CLOCK_SOURCE         0x0a
#define ADC_AC_CLOCK_SELECTOR       0x0b
#define ADC_AC_CLOCK_MULTIPLIER     0x0c
#define ADC_AC_SAMPLERATE_CONVERTER 0x0d

/* Audio Class-Specific AS Interface Descriptor Subtypes */

#define ADC_AS_UNDEF                0x00
#define ADC_AS_GENERAL              0x01
#define ADC_AS_FORMAT_TYPE          0x02
#define ADC_AS_ENCODER              0x03
#define ADC_AS_DECODER              0x04

/* Clock Source Descriptor Clock Types */

#define ADC_CLKSRC_EXTERNAL         0x00       /* External clock */
#define ADC_CLKSRC_INTERNAL_FIXED   0x01       /* Internal fixed clock */
#define ADC_CLKSRC_INTERNAL_VAR     0x02       /* Internal variable clock */
#define ADC_CLKSRC_INTERNAL_PROG    0x03       /* Internal programmable clock */

/* Effect Unit Effect Types */

#define ADC_EFFECT_UNDEF            0x00
#define ADC_EFFECT_PARAM_EQ_SECTION 0x01
#define ADC_EFFECT_REVERBERATION    0x02
#define ADC_EFFECT_MOD_DELAY        0x03
#define ADC_EFFECT_DYN_RANGE_COMP   0x04

/* Processing Unit Process Types */

#define ADC_PROCESS_UNDEF           0x00
#define ADC_PROCESS_UPDOWNMIX       0x01
#define ADC_PROCESS_DOLBY_PROLOGIC  0x02
#define ADC_PROCESS_STEREO_EXTENDER 0x03

/* Audio Class-Specific Endpoint Descriptor Subtypes */

#define ADC_EPTYPE_UNDEF            0x00
#define ADC_EPTYPE_GENERAL          0x01

/* Audio Class-Specific Request Codes */

#define ADC_REQUEST_UNDEF           0x00
#define ADC_REQUEST_CUR             0x01
#define ADC_REQUEST_RANGE           0x02
#define ADC_REQUEST_MEM             0x03

/* Encoder Type Codes */

#define ADC_ENCODER_UNDEF           0x00
#define ADC_ENCODER_OTHER           0x01
#define ADC_ENCODER_MPEG            0x02
#define ADC_ENCODER_AC3             0x03
#define ADC_ENCODER_WMA             0x04
#define ADC_ENCODER_DTS             0x05

/* Decoder Type Codes */

#define ADC_DECODER_UNDEF           0x00
#define ADC_DECODER_OTHER           0x01
#define ADC_DECODER_MPEG            0x02
#define ADC_DECODER_AC3             0x03
#define ADC_DECODER_WMA             0x04
#define ADC_DECODER_DTS             0x05

/* bmChannelConfig: a bitmap field that indicates which spatial locations
 * are occupied by the channels present in the cluster. The bit allocations
 * are as follows:
 */

#define ADC_LOCATION_FL             (1 << 0)   /*  Front Left */
#define ADC_LOCATION_FR             (1 << 1)   /*  Front Right */
#define ADC_LOCATION_FC             (1 << 2)   /*  Front Center */
#define ADC_LOCATION_LFE            (1 << 3)   /*  Low Frequency Effects */
#define ADC_LOCATION_BL             (1 << 4)   /*  Back Left */
#define ADC_LOCATION_BR             (1 << 5)   /*  Back Right */
#define ADC_LOCATION_FLC            (1 << 6)   /*  Front Left of Center */
#define ADC_LOCATION_FRC            (1 << 7)   /*  Front Right of Center */
#define ADC_LOCATION_BC             (1 << 8)   /*  Back Center */
#define ADC_LOCATION_SL             (1 << 9)   /*  Side Left */
#define ADC_LOCATION_SR             (1 << 10)  /*  Side Right */
#define ADC_LOCATION_TC             (1 << 11)  /*  Top Center */
#define ADC_LOCATION_TFL            (1 << 12)  /*  Top Front Left */
#define ADC_LOCATION_TFC            (1 << 13)  /*  Top Front Center */
#define ADC_LOCATION_TFR            (1 << 14)  /*  Top Front Right */
#define ADC_LOCATION_TBL            (1 << 15)  /*  Top Back Left */
#define ADC_LOCATION_TBC            (1 << 16)  /*  Top Back Center */
#define ADC_LOCATION_TBR            (1 << 17)  /*  Top Back Right */
#define ADC_LOCATION_TFLC           (1 << 18)  /*  Top Front Left of Center */
#define ADC_LOCATION_TFRC           (1 << 19)  /*  Top Front Right of Center */
#define ADC_LOCATION_LLFE           (1 << 20)  /*  Left Low Frequency Effects */
#define ADC_LOCATION_RLFE           (1 << 21)  /*  Right Low Frequency Effects */
#define ADC_LOCATION_TSL            (1 << 22)  /*  Top Side Left */
#define ADC_LOCATION_TSR            (1 << 23)  /*  Top Side Right */
#define ADC_LOCATION BC             (1 << 24)  /*  Bottom Center */
#define ADC_LOCATION_BLC            (1 << 25)  /*  Back Left of Center */
#define ADC_LOCATION_BRC            (1 << 26)  /*  Back Right of Center */
                                               /* Bits 27-30: Reserved */
#define ADC_LOCATION_RD             (1 << 31)  /*  Raw Data */

/* Audio Function Category Codes */

#define ADC_CATEGORY_UNDEF          0x00    /* Undefined */
#define ADC_CATEGORY_SPEAKER        0x01    /* Desktop speaker */
#define ADC_CATEGORY_THEATER        0x02    /* Home theater */
#define ADC_CATEGORY_MICROPHONE     0x03    /* Microphone */
#define ADC_CATEGORY_HEADSET        0x04    /* Headset */
#define ADC_CATEGORY_TELEPHONE      0x05    /* Telephone */
#define ADC_CATEGORY_CONVERTER      0x06    /* Converter */
#define ADC_CATEGORY_RECORDER       0x07    /* Voice/Sound recorder */
#define ADC_CATEGORY_IO_BOX         0x08    /* I/O box */
#define ADC_CATEGORY_INSTRUMENT     0x09    /* Musical instrument */
#define ADC_CATEGORY_PROAUDIO       0x0a    /* Pro-audio */
#define ADC_CATEGORY_AV             0x0b    /* Audio/video */
#define ADC_CATEGORY_CONTROL        0x0c    /* Control panel */
#define ADC_CATEGORY_OTHER          0xff

/* Clock Source Control Selectors */

#define ADC_CS_CONTROL_UNDEF        0x00
#define ADC_CS_CONTROL_SAM_FREQ     0x01
#define ADC_CS_CONTROL_CLOCK_VALID  0x02

/* Clock Selector Control Selectors */

#define ADC_CX_CONTROL_UNDEF        0x00
#define ADC_CX_CONTROL_CLOCKSEL     0x01

/* Clock Multiplier Control Selectors */

#define ADC_CM_CONTROL_UNDEF        0x00
#define ADC_CM_CONTROL_NUMERATOR    0x01
#define ADC_CM_CONTROL_DENOMINATOR  0x02

/* Terminal Control Selectors */

#define ADC_TE_CONTROL_UNDEF        0x00
#define ADC_TE_CONTROL_COPY_PROTECT 0x01
#define ADC_TE_CONTROL_CONNECTOR    0x02
#define ADC_TE_CONTROL_OVERLOAD     0x03
#define ADC_TE_CONTROL_CLUSTER      0x04
#define ADC_TE_CONTROL_UNDERFLOW    0x05
#define ADC_TE_CONTROL_OVERFLOW     0x06
#define ADC_TE_CONTROL_LATENCY      0x07

/* Mixer Control Selectors */

#define ADC_MU_CONTROL_UNDEF        0x00
#define ADC_MU_CONTROL_MIXER        0x01
#define ADC_MU_CONTROL_CLUSTER      0x02
#define ADC_MU_CONTROL_UNDERFLOW    0x03
#define ADC_MU_CONTROL_OVERFLOW     0x04
#define ADC_MU_CONTROL_LATENCY      0x05

/* Selector Control Selectors */

#define ADC_SU_CONTROL_UNDEF        0x00
#define ADC_SU_CONTROL_SELECTOR     0x01
#define ADC_SU_CONTROL_LATENCY      0x02

/* Feature Unit Control Selectors */

#define ADC_FU_CONTROL_UNDEF        0x00
#define ADC_FU_CONTROL_MUTE         0x01
#define ADC_FU_CONTROL_VOLUME       0x02
#define ADC_FU_CONTROL_BASS         0x03
#define ADC_FU_CONTROL_MID          0x04
#define ADC_FU_CONTROL_TREBLE       0x05
#define ADC_FU_CONTROL_EQUALIZER    0x06
#define ADC_FU_CONTROL_AGC          0x07
#define ADC_FU_CONTROL_DELAY        0x08
#define ADC_FU_CONTROL_BASS_BOOST   0x09
#define ADC_FU_CONTROL_LOUDNESS     0x0a
#define ADC_FU_CONTROL_INP_GAIN     0x0b
#define ADC_FU_CONTROL_INP_GAIN_PAD 0x0c
#define ADC_FU_CONTROL_PHASE_INVERT 0x0d
#define ADC_FU_CONTROL_UNDERFLOW    0x0e
#define ADC_FU_CONTROL_OVERFLOW     0x0f
#define ADC_FU_CONTROL_LATENCY      0x10

/* Parametric Equalizer Section Effect Unit Control Selectors */

#define ADC_PE_CONTROL_UNDEF        0x00
#define ADC_PE_CONTROL_ENABLE       0x01
#define ADC_PE_CONTROL_CENTERFREQ   0x02
#define ADC_PE_CONTROL_QFACTOR      0x03
#define ADC_PE_CONTROL_GAIN         0x04
#define ADC_PE_CONTROL_UNDERFLOW    0x05
#define ADC_PE_CONTROL_OVERFLOW     0x06
#define ADC_PE_CONTROL_LATENCY      0x07

/* Reverberation Effect Unit Control Selectors */

#define ADC_RV_CONTROL_UNDEF        0x00
#define ADC_RV_CONTROL_ENABLE       0x01
#define ADC_RV_CONTROL_TYPE         0x02
#define ADC_RV_CONTROL_LEVEL        0x03
#define ADC_RV_CONTROL_TIME         0x04
#define ADC_RV_CONTROL_FEEDBACK     0x05
#define ADC_RV_CONTROL_PREDELAY     0x06
#define ADC_RV_CONTROL_DENSITY      0x07
#define ADC_RV_CONTROL_HF_ROLLOFF   0x08
#define ADC_RV_CONTROL_UNDERFLOW    0x09
#define ADC_RV_CONTROL_OVERFLOW     0x0a
#define ADC_RV_CONTROL_LATENCY      0x0b

/* Modulation Delay Effect Unit Control Selectors */

#define ADC_MD_CONTROL_UNDEF        0x00
#define ADC_MD_CONTROL_ENABLE       0x01
#define ADC_MD_CONTROL_BALANCE      0x02
#define ADC_MD_CONTROL_RATE         0x03
#define ADC_MD_CONTROL_DEPTH        0x04
#define ADC_MD_CONTROL_TIME         0x05
#define ADC_MD_CONTROL_FEEDBACK     0x06
#define ADC_MD_CONTROL_UNDERFLOW    0x07
#define ADC_MD_CONTROL_OVERFLOW     0x08
#define ADC_MD_CONTROL_LATENCY      0x09

/* Dynamic Range Compressor Effect Unit Control Selectors */

#define ADC_DR_CONTROL_UNDEF        0x00
#define ADC_DR_CONTROL_ENABLE       0x01
#define ADC_DR_CONTROL_COMP_RATE    0x02
#define ADC_DR_CONTROL_MAXAMPL      0x03
#define ADC_DR_CONTROL_THRESHOLD    0x04
#define ADC_DR_CONTROL_ATTACK_TIME  0x05
#define ADC_DR_CONTROL_RELEASE_TIME 0x06
#define ADC_DR_CONTROL_UNDERFLOW    0x07
#define ADC_DR_CONTROL_OVERFLOW     0x08
#define ADC_DR_CONTROL_LATENCY      0x09

/* Up/Down-mix Processing Unit Control Selectors */

#define ADC_UD_CONTROL_UNDEF        0x00
#define ADC_UD_CONTROL_ENABLE       0x01
#define ADC_UD_CONTROL_MODE_SELECT  0x02
#define ADC_UD_CONTROL_CLUSTER      0x03
#define ADC_UD_CONTROL_UNDERFLOW    0x04
#define ADC_UD_CONTROL_OVERFLOW     0x05
#define ADC_UD_CONTROL_LATENCY      0x06

/* Dolby Prologic™ Processing Unit Control Selectors */

#define ADC_DP_CONTROL_UNDEF        0x00
#define ADC_DP_CONTROL_ENABLE       0x01
#define ADC_DP_CONTROL_MODE_SELECT  0x02
#define ADC_DP_CONTROL_CLUSTER      0x03
#define ADC_DP_CONTROL_UNDERFLOW    0x04
#define ADC_DP_CONTROL_OVERFLOW     0x05
#define ADC_DP_CONTROL_LATENCY      0x06

/* Stereo Extender Processing Unit Control Selectors */

#define ADC_STEXT_CONTROL_UNDEF     0x00
#define ADC_STEXT_CONTROL_ENABLE    0x01
#define ADC_STEXT_CONTROL_WIDTH     0x02
#define ADC_STEXT_CONTROL_UNDERFLOW 0x03
#define ADC_STEXT_CONTROL_OVERFLOW  0x04
#define ADC_STEXT_CONTROL_LATENCY   0x05

/* Extension Unit Control Selectors */

#define ADC_XU_CONTROL_UNDEF        0x00
#define ADC_XU_CONTROL_ENABLE       0x01
#define ADC_XU_CONTROL_CLUSTER      0x02
#define ADC_XU_CONTROL_UNDERFLOW    0x03
#define ADC_XU_CONTROL_OVERFLOW     0x04
#define ADC_XU_CONTROL_LATENCY      0x05

/* AudioStreaming Interface Control Selectors */

#define ADC_AS_CONTROL_UNDEF        0x00
#define ADC_AS_CONTROL_ACT_ALT      0x01
#define ADC_AS_CONTROL_VAL_ALT      0x02
#define ADC_AS_CONTROL_AUDIO_FORMAT 0x03

/* Encoder Control Selectors */

#define ADC_EN_CONTROL_UNDEF        0x00
#define ADC_EN_CONTROL_BIT_RATE     0x01
#define ADC_EN_CONTROL_QUALITY      0x02
#define ADC_EN_CONTROL_VBR          0x03
#define ADC_EN_CONTROL_TYPE         0x04
#define ADC_EN_CONTROL_UNDERFLOW    0x05
#define ADC_EN_CONTROL_OVERFLOW     0x06
#define ADC_EN_CONTROL_ENCODER_ERR  0x07
#define ADC_EN_CONTROL_PARAM1       0x08
#define ADC_EN_CONTROL_PARAM2       0x09
#define ADC_EN_CONTROL_PARAM3       0x0a
#define ADC_EN_CONTROL_PARAM4       0x0b
#define ADC_EN_CONTROL_PARAM5       0x0c
#define ADC_EN_CONTROL_PARAM6       0x0d
#define ADC_EN_CONTROL_PARAM7       0x0e
#define ADC_EN_CONTROL_PARAM8       0x0f

/* MPEG Decoder Control Selectors */

#define ADC_MPGD_CONTROL_UNDEF      0x00
#define ADC_MPGD_CONTROL_DUAL_CHAN  0x01
#define ADC_MPGD_CONTROL_2ND_STEREO 0x02
#define ADC_MPGD_CONTROL_MULTILING  0x03
#define ADC_MPGD_CONTROL_DYN_RANGE  0x04
#define ADC_MPGD_CONTROL_SCALING    0x05
#define ADC_MPGD_CONTROL_HILO_SCALE 0x06
#define ADC_MPGD_CONTROL_UNDERFLOW  0x07
#define ADC_MPGD_CONTROL_OVERFLOW   0x08
#define ADC_MPGD_CONTROL_DECODE_ERR 0x09

/* AC-3 Decoder Control Selectors */

#define ADC_AC3D_CONTROL_UNDEF      0x00
#define ADC_AC3D_CONTROL_MODE       0x01
#define ADC_AC3D_CONTROL_DYN_RANGE  0x02
#define ADC_AC3D_CONTROL_SCALING    0x03
#define ADC_AC3D_CONTROL_HILO_SCALE 0x04
#define ADC_AC3D_CONTROL_UNDERFLOW  0x05
#define ADC_AC3D_CONTROL_OVERFLOW   0x06
#define ADC_AC3D_CONTROL_DECODE_ERR 0x07

/* WMA Decoder Control Selectors */

#define ADC_WMAD_CONTROL_UNDEF      0x00
#define ADC_WMAD_CONTROL_UNDERFLOW  0x01
#define ADC_WMAD_CONTROL_OVERFLOW   0x02
#define ADC_WMAD_CONTROL_DECODE_ERR 0x03

/* DTS Decoder Control Selectors */

#define ADC_DTSD_CONTROL_UNDEF      0x00
#define ADC_DTSD_CONTROL_UNDERFLOW  0x01
#define ADC_DTSD_CONTROL_OVERFLOW   0x02
#define ADC_DTSD_CONTROL_DECODE_ERR 0x03

/* Endpoint Control Selectors */

#define ADC_EP_CONTROL_UNDEF        0x00
#define ADC_EP_CONTROL_PITCH        0x01
#define ADC_EP_CONTROL_OVERRUN      0x02
#define ADC_EP_CONTROL_UNDERRUN     0x03

/* Encoder Error Codes */

                                               /* <0: Reserved for vendor extensions */
#define ADC_ENCODER_SUCCESS         0          /* No Error */
#define ADC_ENCODER_ERROR_NOMEM     1          /* Out of Memory */
#define ADC_ENCODER_ERROR_BW        2          /* Out of Bandwidth */
#define ADC_ENCODER_ERROR_CYCLE     3          /* Out of Processing Cycles */
#define ADC_ENCODER_ERROR_FRAME     4          /* General Format Frame Error */
#define ADC_ENCODER_ERROR_TOOSMALL  5          /* Format Frame Too Small */
#define ADC_ENCODER_ERROR_TOOBIG    6          /* Format Frame Too Large */
#define ADC_ENCODER_ERROR_BADFORMAT 7          /* Bad Data Format */
#define ADC_ENCODER_ERROR_NCHAN     8          /* Incorrect Number of Channels */
#define ADC_ENCODER_ERROR_RATE      9          /* Incorrect Sampling Rate */
#define ADC_ENCODER_ERROR_BITRATE   10         /* Unable to Meet Target Bitrate */
#define ADC_ENCODER_ERROR_PARMS     11         /* Inconsistent Set of Parameters */
#define ADC_ENCODER_ERROR_NOTREADY  12         /* Not Ready */
#define ADC_ENCODER_ERROR_BUSY      13         /* Busy */
                                               /* >13: Reserved */

/* Format Type Codes */

#define ADC_FORMAT_TYPE_UNDEF       0x00
#define ADC_FORMAT_TYPEI            0x01
#define ADC_FORMAT_TYPEII           0x02
#define ADC_FORMAT_TYPEIII          0x03
#define ADC_FORMAT_TYPEIV           0x04
#define ADC_FORMAT_EXT_TYPEI        0x81
#define ADC_FORMAT_EXT_TYPEII       0x82
#define ADC_FORMAT_EXT_TYPEIII      0x83

/* Audio Data Format Type I Bit Allocations */

#define ADC_FORMAT_TYPEI_PCM        (1 << 0)
#define ADC_FORMAT_TYPEI_PCM8       (1 << 1)
#define ADC_FORMAT_TYPEI_IEEEFLOAT  (1 << 2)
#define ADC_FORMAT_TYPEI_ALAW       (1 << 3)
#define ADC_FORMAT_TYPEI_MULAW      (1 << 4)
#define ADC_FORMAT_TYPEI_RAWDATA    (1 << 31)

/* Audio Data Format Type II Bit Allocations */

#define ADC_FORMAT_TYPEII_MPEG      (1 << 0)
#define ADC_FORMAT_TYPEII_AC3       (1 << 1)
#define ADC_FORMAT_TYPEII_WMA       (1 << 2)
#define ADC_FORMAT_TYPEII_DTS       (1 << 3)
#define ADC_FORMAT_TYPEII_RAWDATA   (1 << 31)

/* Audio Data Format Type III Bit Allocations */


#define ADC_FORMAT_TYPEIII_IEC61937_AC3            (1 << 0)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG1_L1       (1 << 1)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG1_L2_3     (1 << 1)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG2_NOEXT    (1 << 2)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG2_EXT      (1 << 3)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG2_AAC_ADTS (1 << 4)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG2_L1_LS    (1 << 5)
#define ADC_FORMAT_TYPEIII_IEC61937_MPEG2_L2_3_LS  (1 << 6)
#define ADC_FORMAT_TYPEIII_IEC61937_DTS-I          (1 << 7)
#define ADC_FORMAT_TYPEIII_IEC61937_DTS-II         (1 << 8)
#define ADC_FORMAT_TYPEIII_IEC61937_DTS-III        (1 << 9)
#define ADC_FORMAT_TYPEIII_IEC61937_ATRAC          (1 << 10)
#define ADC_FORMAT_TYPEIII_IEC61937_ATRAC2_3       (1 << 11)
#define ADC_FORMAT_TYPEIII_WMA                     (1 << 12)

/* Audio Data Format Type IV Bit Allocations */

#define ADC_FORMAT_TYPEIV_PCM                      (1 << 0)
#define ADC_FORMAT_TYPEIV_PCM8                     (1 << 1)
#define ADC_FORMAT_TYPEIV_IEEE_FLOAT               (1 << 2)
#define ADC_FORMAT_TYPEIV_ALAW                     (1 << 3)
#define ADC_FORMAT_TYPEIV_MULAW                    (1 << 4)
#define ADC_FORMAT_TYPEIV_MPEG                     (1 << 5)
#define ADC_FORMAT_TYPEIV_AC3                      (1 << 6)
#define ADC_FORMAT_TYPEIV_WMA                      (1 << 7)
#define ADC_FORMAT_TYPEIV_IEC61937_AC3             (1 << 8)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG1_L1        (1 << 9)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG1_L2_3      (1 << 10)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG2_NOEXT     (1 << 10)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG2_EXT       (1 << 11)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG2_AAC_ADTS  (1 << 12)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG2_L1_LS     (1 << 13)
#define ADC_FORMAT_TYPEIV_IEC61937_MPEG2_L2_3_LS   (1 << 14)
#define ADC_FORMAT_TYPEIV_IEC61937_DTS-I           (1 << 15)
#define ADC_FORMAT_TYPEIV_IEC61937_DTS-II          (1 << 16)
#define ADC_FORMAT_TYPEIV_IEC61937_DTS-III         (1 << 17)
#define ADC_FORMAT_TYPEIV_IEC61937_ATRAC           (1 << 18)
#define ADC_FORMAT_TYPEIV_IEC61937_ATRAC2_3        (1 << 19)
#define ADC_FORMAT_TYPEIV_TYPE_III_WMA             (1 << 20)
#define ADC_FORMAT_TYPEIV_IEC60958_PCM             (1 << 21)

/* Side Band Protocol Codes */

#define ADC_SIDEBAND_PROTOCOL_UNDEF 0x00
#define ADC_PRES_TIMESTAMP_PROTOCOL 0x01

/* USB Terminal Types */

#define ADC_TERMINAL_UNDEF          0x0100
#define ADC_TERMINAL_STREAMING      0x0101
#define ADC_TERMINAL_VENDOR         0x01ff

/* Input Terminal Types */

#define ADC_INTERM_UNDEF            0x0200     /* Undefined Type */
#define ADC_INTERM_MIC              0x0201     /* A generic microhpone */
#define ADC_INTERM_DESKTOP_MIC      0x0202     /* A desktop microphone */
#define ADC_INTERM_PERSONAL_MIC     0x0203     /* Head-mounted or clip-on microphone */
#define ADC_INTERM_OMNI_MIC         0x0204     /* Omni-directional microphone */
#define ADC_INTERM_MIC_ARRAY        0x0205     /* Microphone array */
#define ADC_INTERM_PROC_MIC_ARRAY   0x0206     /* Microphone array with signal processor */

/* Output Terminal Types */

#define ADC_OUTTERM_UNDEF           0x0300     /* Undefined Type */
#define ADC_OUTTERM_SPEAKER         0x0301     /* Generic speakers */
#define ADC_OUTTERM_HEADPHONES      0x0302     /* A head-mounted audio output device */
#define ADC_OUTTERM_HEADDISPLAY     0x0303     /* Head Mounted Display Audio */
#define ADC_OUTTERM_DESKTOP         0x0304     /* Desktop speaker */
#define ADC_OUTTERM_ROOM            0x0305     /* Room speaker */
#define ADC_OUTTERM_COMMS           0x0306     /* Communication speaker */
#define ADC_OUTTERM_LOFREQ          0x0307     /* Low frequency effects speaker */

/* Bi-directional Terminal Types */

#define ADC_BIDITERM_UNDEF          0x0400     /* Undefined Type */
#define ADC_BIDITERM_HANDSET        0x0401     /* Hand-held bi-directional audio device */
#define ADC_BIDITERM_HEADSET        0x0402     /* Head-mounted bi-directional audio device */
#define ADC_BIDITERM_SPEAKERPHONE   0x0403     /* Speakerphone, no echo reduction */
#define ADC_BIDITERM_ECHOSUPPRESS   0x0404     /* Echo-suppressing speakerphone */
#define ADC_BIDITERM_ECHOCANCEL     0x0405     /* Echo-canceling speakerphone */

/* Telephony Terminal Types */

#define ADC_TELETERM_UNDEF          0x0500     /* Undefined Type */
#define ADC_TELETERM_PHONELINE      0x0501     /* Analog telephone line jack, an ISDN line,
                                                * a proprietary PBX interface, or a wireless link */
#define ADC_TELETERM_TELEPHONE      0x0502     /* Device can be used as a telephone */
#define ADC_TELETERM_DOWNLINE       0x0503     /* Down Line Phone */

/* External Terminal Types */

#define ADC_EXTTERM_UNDEF           0x0600     /* Undefined Type */
#define ADC_EXTTERM_ANALOG          0x0601     /* Generic analog connector */
#define ADC_EXTTERM_DIGITAL         0x0602     /* Generic digital audio interface */
#define ADC_EXTTERM_LINE            0x0603     /* Analog connector at standard line levels */
#define ADC_EXTTERM_LEGACY          0x0604     /* Legacy audio line out connector */
#define ADC_EXTTERM_SPDIF           0x0605     /* SPDIF interface */
#define ADC_EXTTERM_1394DA          0x0606     /* 1394 DA stream */
#define ADC_EXTTERM_1394DV          0x0607     /* 1394 DV stream soundtrack */
#define ADC_EXTTERM_ADAT            0x0608     /* ADAT Lightpipe */
#define ADC_EXTTERM_TDIF            0x0609     /* TDIF  - Tascam Digital Interface */
#define ADC_EXTTERM_MADI            0x060a     /* MADI - Multi-channel Audio Digital Interface (AES) */

/* Embedded Function Terminal Types */

#define ADC_EMBEDTERM_UNDEF         0x0700     /* Undefined Type */
#define ADC_EMBEDTERM_CALIBRATION   0x0701     /* Level Calibration Noise Source */
#define ADC_EMBEDTERM_EQUALIZATION  0x0702     /* Equalization Noise */
#define ADC_EMBEDTERM_CD            0x0703     /* CD player */
#define ADC_EMBEDTERM_DAT           0x0704     /* Digital Audio Tape */
#define ADC_EMBEDTERM_DCC           0x0705     /* Digital Compact Cassette */
#define ADC_EMBEDTERM_COMPRESSED    0x0706     /* Compressed Audio Player */
#define ADC_EMBEDTERM_TAPE          0x0707     /* Analog Audio Tape */
#define ADC_EMBEDTERM_PHONOGRAPH    0x0708     /* Analog vinyl record player */
#define ADC_EMBEDTERM_VCR           0x0709     /* Audio track of VCR */
#define ADC_EMBEDTERM_VIDDISC       0x070a     /* Audio track of VideoDisc player */
#define ADC_EMBEDTERM_DVD           0x070b     /* Audio track of DVD player */
#define ADC_EMBEDTERM_TVTUNER       0x070c     /* Audio track of TV tuner */
#define ADC_EMBEDTERM_SATELLITE     0x070d     /* Audio track of satellite receiver */
#define ADC_EMBEDTERM_CABLETUNER    0x070e     /* Audio track of cable tuner */
#define ADC_EMBEDTERM_DSS           0x070f     /* Audio track of DSS receiver */
#define ADC_EMBEDTERM_RADIO         0x0710     /* AM/FM radio receiver */
#define ADC_EMBEDTERM_TRANSMITTER   0x0711     /* AM/FM radio transmitter */
#define ADC_EMBEDTERM_MULTITRACK    0x0712     /* A multi-track recording system */
#define ADC_EMBEDTERM_SYNTHESIZER   0x0713     /* Synthesizer */
#define ADC_EMBEDTERM_PIANO         0x0714     /* Piano */
#define ADC_EMBEDTERM_GUITAR        0x0715     /* Guitar */
#define ADC_EMBEDTERM_PERCUSSON     0x0716     /* Percussion Instrument */
#define ADC_EMBEDTERM_INSTRUMENT    0x0717     /* Other Musical Instrument */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
/* Audio Channel Cluster Descriptor */

struct adc_cluster_desc_s
{
  uint8_t cl_nchan;             /* 0: Number of logical channels in the cluster */
  uint8_t cl_config[4];         /* 1: The spatial location of the channels */
  uint8_t cl_names;             /* 5: Index of name string of first channel */
};
#define USB_SIZEOF_ADC_CLUSTER_DESC 6

/* Class-specific AC Interface Descriptor */

struct adc_ac_ifdesc_s
{
  uint8_t ac_len;               /* 0: Descriptor length (9)*/
  uint8_t ac_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ac_subtype;           /* 2: Descriptor sub-type (ADC_AC_HEADER) */
  uint8_t ac_adc[2];            /* 3: ADC spec version in BCD */
  uint8_t ac_category;          /* 5: Category of audio function */
  uint8_t ac_totallen[2];       /* 6: Total length */
  uint8_t ac_controls;          /* 8: Bits 0-1: Latency control; Bits 2-7 reserved */
};
#define USB_SIZEOF_ADC_AC_IFDESC 9

/* Clock Source Descriptor */

struct adc_clksrc_desc_s
{
  uint8_t cs_len;               /* 0: Descriptor length (8)*/
  uint8_t cs_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t cs_subtype;           /* 2: Descriptor sub-type (ADC_AC_CLOCK_SOURCE) */
  uint8_t cs_clockid;           /* 3: Identifies clock source entity */
  uint8_t cs_attr;              /* 4: Bits 0-1: CLKSRC type, D2: Clock synch'ed top SOF */
  uint8_t cs_controls;          /* 5: Bits 0-1: Clock freq control, Bits 2-3: Clock valid control */
  uint8_t cs_termid;            /* 6: ID of the terminal associated with the clock source */
  uint8_t cs_clksrc;            /* 7: Clock source string index */
};
#define USB_SIZEOF_ADC_CLKSRC_DESC 8

/* Clock Selector Descriptor */

struct adc_clksel_desc_s
{
  uint8_t cl_len;               /* 0: Descriptor length (7+npins)*/
  uint8_t cl_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t cl_subtype;           /* 2: Descriptor sub-type (ADC_AC_CLOCK_SELECTOR) */
  uint8_t cl_clockid;           /* 3: Identifies clock source entity */
  uint8_t cl_npins;             /* 4: Number of input pins */
  uint8_t cl_variable[1];       /* 5-(5+npins-1): cl_csrcid, ID of clock input to pin n, n=1-npins */
                                /* 5+npins: cl_controls:
                                 *    Bits 0-1: Clock selector controls, Bits 2-7: Reserved */
                                /* 6+npins: cl_clksel
                                 *    Clock selector string index */
};

#define cl_csrcid(n)   cl_variable[(n)-1]
#define cl_controls(p) cl_variable[(p)->cl_npins]
#define cl_clksel(p)   cl_variable[(p)->cl_npins+1]

#define USB_SIZEOF_ADC_CLKSEL_DESC(npins) (7+(npins))

/* Clock Multiplier Descriptor */

struct adc_clkmult_desc_s
{
  uint8_t cm_len;               /* 0: Descriptor length (7) */
  uint8_t cm_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t cm_subtype;           /* 2: Descriptor sub-type (ADC_AC_CLOCK_MULTIPLIER) */
  uint8_t cm_clockid;           /* 3: Identifies clock source entity */
  uint8_t cm_csrcid;            /* 4: ID of clock input to list pin n */
  uint8_t cm_controls;          /* 5: Bits 0-1: Clock numerator control,
                              *    Bits 2-3: Clock denominator control */
  uint8_t cm_clkmult;           /* 6: Index of clock multiplier name string */
};
#define USB_SIZEOF_ADC_CLKMULT_DESC 7

/* Input Terminal Descriptor */

struct adc_interm_desc_s
{
  uint8_t it_len;               /* 0: Descriptor length (17) */
  uint8_t it_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t it_subtype;           /* 2: Descriptor sub-type (ADC_AC_INPUT_TERMINAL) */
  uint8_t it_termid;            /* 3: Identifies terminal in audio function */
  uint8_t it_termtype[2];       /* 4: Terminal type */
  uint8_t it_outterm;           /* 6: ID of the associated output terminal */
  uint8_t it_csrcid;            /* 7: ID of clock entity to which terminal is connected */
  uint8_t it_nchan;             /* 8: Number of logical output channels */
  uint8_t it_config[4];         /* 9: The spatial location of the logical channels */
  uint8_t it_names;             /* 13: Index of name string of first logical channel */
  uint8_t it_controls{2];       /* 14: Bits 0-1: Copy protect control,
                                 *     Bits 2-3: Converter control
                                 *     Bits 4-5: Overload control
                                 *     Bits 6-7: Cluster control
                                 *     Bits 8-9: Underflow control
                                 *     Bits 10-11: Overflow control
                                 *     Bits 12-15: Reserved */
  uint8_t it_interm;            /* 16: Input terminal string index */
};
#define USB_SIZEOF_ADC_INTERM_DESC 17

/* Output Terminal Descriptor */

struct adc_outterm_desc_s
{
  uint8_t ot_len;               /* 0: Descriptor length (12) */
  uint8_t ot_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ot_subtype;           /* 2: Descriptor sub-type (ADC_AC_OUTPUT_TERMINAL) */
  uint8_t ot_termid;            /* 3: Identifies terminal in audio function */
  uint8_t ot_termtype[2];       /* 4: Terminal type */
  uint8_t ot_interm;            /* 6: ID of the associated input terminal */
  uint8_t ot_srcid;             /* 7: ID of unit/terminal to which terminal is connnected */
  uint8_t ot_csrcid;            /* 8: ID of clock entity to whcih terminal is connected */
  uint8_t ot_controls{2];       /* 9: Bits 0-1: Copy protect control,
                                 *    Bits 2-3: Connector control
                                 *    Bits 4-5: Overload control
                                 *    Bits 6-7: Underflow control
                                 *    Bits 8-9: Overflow control
                                 *    Bits 10-15: Reserved */
  uint8_t ot_outterm;           /* 11: Output terminal string index */
};
#define USB_SIZEOF_ADC_OUTTERM_DESC 12

/* Mixer Unit Descriptor */

struct adc_mixerunit_desc_s
{
  uint8_t mu_len;               /* 0: Descriptor length (13+npins+nchan)*/
  uint8_t mu_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t mu_subtype;           /* 2: Descriptor sub-type (ADC_AC_MIXER_UNIT) */
  uint8_t mu_unitid;            /* 3: Identifies unit in audio function */
  uint8_t mu_npins;             /* 4: Number of input pins of this unit */
  uint8_t mu_variable[1];       /* 5-(5+npins-1): mu_srcid[n]
                                 *   ID of clock input connected to pin n, n=1-npins */
                                /* 5+npins: nchan=Number of logic output channels */
                                /* 6+npins: config[4]=spatial location of channels */
                                /* 10+npins: name=String index of first channel name */
                                /* 11+npins+nchan: controls
                                 *   Bits 0-1: Cluster control
                                 *   Bits 2-3: Underflow control
                                 *   Bits 4-5: Overflow control
                                 *   Bits 6-7: Reserved */
                                /* 12+npins+nchan: mixer=String index of mixer unit name */
};

#define mu_srcid[n]    mu_variable[(n)-1]
#define mu_nchan(p)    mu_variable[(p)->mu_npins]
#define mu_controls(p) mu_variable[(p)->mu_npins+1]
#define mu_mixer(p)    mu_variable[(p)->mu_npins+2]

#define USB_SIZEOF_ADC_CLKSEL_DESC(npins,nchan) (13+(npins)+(nchan))

/* Selector Unit Descriptor */

struct adc_selunit_desc_s
{
  uint8_t su_len;               /* 0: Descriptor length (7+npins) */
  uint8_t su_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t su_subtype;           /* 2: Descriptor sub-type (ADC_AC_SELECTOR_UNIT) */
  uint8_t su_unitid;            /* 3: Identifies unit in audio function */
  uint8_t su_npins;             /* 4: Number of input pins of this unit */
  uint8_t su_vairable[1];       /* 5-(5+npins-1): su_srcid[n]=ID of unit/terminal input connected to
                                 *   pin n, n=1-npins */
                                /* 5+npins: su_controls
                                 *   Bits 0-1: Selector control
                                 *   Bits 2-7: Reserved */
                                /* 6+npins: su_selector=String index of selector unit name */
};

#define su_srcid(n)    su_srcid[(n)-1]
#define su_controls(p) su_srcid[(p)->su_npins+1]
#define su_selector(p) su_srcid[(p)->su_npins+2]

#define USB_SIZEOF_ADC_SELUNIT_DESC(npins,nchan) (7+(npins))

/* Feature Unit Descriptor */

struct adc_featunit_desc_s
{
  uint8_t fu_len;               /* 0: Descriptor length (6+4*(nchan+1)) */
  uint8_t fu_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t fu_subtype;           /* 2: Descriptor sub-type (ADC_AC_FEATURE_UNIT) */
  uint8_t fu_unitid;            /* 3: Identifies unit in audio function */
  uint8_t fu_srcid;             /* 4: ID of unit/terminal to which unit is connected */
  uint8_t fu_variable[1];       /* 5-(5+4*nchan): fu_controls
                                 * Controls for master channel n, n=0,..nchan,
                                 *   Bits 0-1: Mute control
                                 *   Bits 2-3: Volume control
                                 *   Bits 4-5: Bass control
                                 *   Bits 6-7: Mid control
                                 *   Bits 8-9: Treble control
                                 *   Bits 10-11: Graphic equalizer control
                                 *   Bits 12-13: Automatic gain control
                                 *   Bits 14-15: Delay control
                                 *   Bits 16-17: Bass boos control
                                 *   Bits 18-19: Loudness control
                                 *   Bits 20-21: Input gain control
                                 *   Bits 22-23: Input gain pad control
                                 *   Bits 24-25: Phase inverter control
                                 *   Bits 26-27: Underflow control
                                 *   Bits 28-29: Overflow control
                                 *   Bits 30-31: Reserved */
                                /* 5+4*(nchan+1): feature=Strings index to feature unit name */
};

#define fu_controls(n)    fu_variable[4*(n)]
#define fu_feature(nchan) fu_variable[4*((nchan)+1]

#define USB_SIZEOF_ADC_FEATUNIT_DESC(nchan) (6+4*((nchan)+1)))

/* Sampling Rate Converter Descriptor */

struct adc_srconverter_desc_s
{
  uint8_t sr_len;               /* 0: Descriptor length (8) */
  uint8_t sr_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t sr_subtype;           /* 2: Descriptor sub-type (ADC_AC_SAMPLERATE_CONVERTER) */
  uint8_t sr_unitid;            /* 3: Identifies unit in audio function */
  uint8_t sr_srcid;             /* 4: ID of unit/terminal to which unit is connected */
  uint8_t sr_csrcinid;          /* 5: ID of clock entity to which unit input is connected */
  uint8_t sr_csrcoutid;         /* 6: ID of clock entity to which unit output is connected */
  uint8_t sr_converter;         /* 7: String index to the name fo the SRC unit */
};
#define USB_SIZEOF_ADC_SRCCONVERTER_DESC (8)

/* Common Effect Unit Descriptor */

struct adc_effectunit_desc_s
{
  uint8_t ef_len;               /* 0: Descriptor length (16+4*nchan) */
  uint8_t ef_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ef_subtype;           /* 2: Descriptor sub-type (ADC_AC_EFFECT_UNIT) */
  uint8_t ef_unitid;            /* 3: Identifies unit in audio function */
  uint8_t ef_eftype[2];         /* 4: Effect type */
  uint8_t ef_srcid;             /* 6: ID of unit/terminal to which unit is connected */
  unit8_t ef_variable[1];       /* 7-(7+4*(nchan+1)): ef_controls[n]
                                 * Controls for channel n, n=0,..,nchan
                                 *   Bits 0-31: Effect-specific allocation */
                                /* 15+4*nchan: ef_effects=String index to the effect unit name */
};

#define ef_controls(n)    ef_controls[4*(n)]
#define ef_effects(nchan) ef_controls[4*(nchan)]

#define USB_SIZEOF_ADC_EFFECTUNIT_DESC(nchan) (16+4*(nchsn))

/* Parametric Equalizer Section Effect Unit Descriptor
 *
 *   ef_eftype = ADC_EFFECT_PARAM_EQ_SECTION
 *   ef_controls:
 *     Bits 0-1: Enable control
 *     Bits 2-3: Center frequency control
 *     Bits 4-5: Q factor control
 *     Bits 6-7: Gain control
 *     Bits 8-9: Underflow control
 *     Bits 10-11: Overflow control
 *     Bits 12-31: Reserved
 *
 * Reverberation Effect Unit Descriptor
 *
 *   ef_eftype = ADC_EFFECT_REVERBERATION
 *   ef_controls:
 *     Bits 0-1: Enable control
 *     Bits 2-3: Type control
 *     Bits 4-5: Level control
 *     Bits 6-7: Time control
 *     Bits 8-9: Delay feedback control
 *     Bits 10-11: Pre-delay control
 *     Bits 12-13: Density control
 *     Bits 13-15: Hi-freq roll-off control
 *     Bits 16-17: Underflow control
 *     Bits 18-19: Overflow control
 *     Bits 20-31: Reserved
 *
 * Modulation Delay Effect Unit Descriptor
 *
 *   ef_eftype = ADC_EFFECT_MOD_DELAY
 *   ef_controls:
 *     Bits 0-1: Enable control
 *     Bits 2-3: Balance control
 *     Bits 4-5: Rate control
 *     Bits 6-7: Depth control
 *     Bits 8-9: Time control
 *     Bits 10-11: Feedback level control
 *     Bits 12-13: Underflow control
 *     Bits 14-15: Overflow control
 *     Bits 16-31: Reserved
 *
 * Dynamic Range Compressor Effect Unit Descriptor
 *
 *   ef_eftype = ADC_EFFECT_DYN_RANGE_COMP
 *   ef_controls:
 *     Bits 0-1: Enable control
 *     Bits 2-3: Compression control
 *     Bits 4-5: MaxAmpl control
 *     Bits 6-7: Threshold control
 *     Bits 8-9: Attack time control
 *     Bits 10-11: Release time control
 *     Bits 12-13: Underflow control
 *     Bits 14-15: Overflow control
 *     Bits 16-31: Reserved
 */

/* Common Processing Unit Descriptor */

struct adc_procunit_desc_s
{
  uint8_t pu_len;               /* 0: Descriptor length (17+npins+x) */
  uint8_t pu_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t pu_subtype;           /* 2: Descriptor sub-type (ADC_AC_PROCESSING_UNIT) */
  uint8_t pu_unitid;            /* 3: Identifies unit in audio function */
  uint8_t pu_putype[2];         /* 4: Processing unit type */
  uint8_t pu_npins;             /* 6: Number of input pins of this unit */
  unit8_t pu_variable[1];       /* 7-(7+(npins11)): pu_srcid[n]
                                 *   ID of unit/terminal input is connected to, n=1,..,npins */
                                /* 7+npins: pu_nchan: Number of logic output channels */
                                /* 8+npins: pu_config: Spatial location of channels */
                                /* 12+npins: pu_names: String index to first channel name */
                                /* 13+npins: pu_controls
                                 *   Bits 0-1: Enable control
                                 *   Bits 2-15: Process-specific controls */
                                /* 15+npins: pu_processing: String index to name of processing unit */
                                /* 16+npins: pu_specific: Beginning of process-specific descriptor */
};

#define pu_srcid(n)       pu_variable[n]
#define pu_nchan(p)       pu_variable[(Ip)->npins]
#define pu_config(p)      pu_variable[(Ip)->npins+1]
#define pu_names(p)       pu_variable[(Ip)->npins+5]
#define pu_controls(p)    pu_variable[(Ip)->npins+6]
#define pu_processing(p)  pu_variable[(Ip)->npins+8]
#define pu_specific(p)   &pu_variable[(Ip)->npins+9]

#define USB_SIZEOF_ADC_PROCUNIT_DESC(npins) (16+(npins))

/* Up/Down-mix Processing Unit Descriptor */

struct adc_updownunit_desc_s
{
  uint8_t ud_len;               /* 0: Descriptor length (18+4*nmodes) */
  uint8_t ud_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ud_subtype;           /* 2: Descriptor sub-type (ADC_AC_PROCESSING_UNIT) */
  uint8_t ud_unitid;            /* 3: Identifies unit in audio function */
  uint8_t ud_putype[2];         /* 4: Processing unit type (ADC_PROCESS_UPDOWNMIX) */
  uint8_t ud_npins;             /* 6: Number of input pins of this unit (1) */
  unit8_t ud_srcid;             /* 7: ID of unit/terminal input is connected to */
  uint8_t ud_nchan;             /* 8: Number of logic output channels */
  uint8_t ud_config[4];         /* 9: Spatial location of channels */
  uint8_t ud_names;             /* 13: String index to first channel name */
  uint8_t ud_controls[2];       /* 14: controls
                                 *   Bits 0-1: Enable control
                                 *   Bits 2-3; Mode select control
                                 *   Bits 4-5: Cluster control
                                 *   Bits 6-7: Underflow control
                                 *   Bits 8-9: Overflow control
                                 *   Bits 10-15 Reserved */
  uint8_t ud_processing;        /* 16: String index to name of processing unit */
  uint8_t ud_nmodes;            /* 17: Number of modes supported */
  uint8_t ud_modes[1];          /* 18-(18+4*(nmodes-1)): Active logical channels in mode n */
};

#define USB_SIZEOF_ADC_UPDOWNUNIT_DESC(nmodes) (18+4(nmodes))

/* Dolby Prologic Processing Unit Descriptor */

struct adc_dolbyunit_desc_s
{
  uint8_t dp_len;               /* 0: Descriptor length (18+4*nmodes) */
  uint8_t dp_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t dp_subtype;           /* 2: Descriptor sub-type (ADC_AC_PROCESSING_UNIT) */
  uint8_t dp_unitid;            /* 3: Identifies unit in audio function */
  uint8_t dp_putype[2];         /* 4: Processing unit type (ADC_PROCESS_DOLBY_PROLOGIC) */
  uint8_t dp_npins;             /* 6: Number of input pins of this unit (1) */
  unit8_t dp_srcid;             /* 7: ID of unit/terminal input is connected to */
  uint8_t dp_nchan;             /* 8: Number of logic output channels */
  uint8_t dp_config[4];         /* 9: Spatial location of channels */
  uint8_t dp_names;             /* 13: String index to first channel name */
  uint8_t dp_controls[2];       /* 14: controls
                                 *   Bits 0-1: Enable control
                                 *   Bits 2-3; Mode select control
                                 *   Bits 4-5: Cluster control
                                 *   Bits 6-7: Underflow control
                                 *   Bits 8-9: Overflow control
                                 *   Bits 10-15 Reserved */
  uint8_t dp_processing;        /* 16: String index to name of processing unit */
  uint8_t dp_nmodes;            /* 17: Number of modes supported */
  uint8_t dp_modes[1];          /* 18-(18+4*(nmodes-1)): Active logical channels in mode n */
};

#define USB_SIZEOF_ADC_DOLBYUNIT_DESC(nmodes) (18+4(nmodes))

/* Stereo Extender Processing Unit Descriptor */

struct adc_stextunit_desc_s
{
  uint8_t st_len;               /* 0: Descriptor length (17) */
  uint8_t st_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t st_subtype;           /* 2: Descriptor sub-type (ADC_AC_PROCESSING_UNIT) */
  uint8_t st_unitid;            /* 3: Identifies unit in audio function */
  uint8_t st_putype[2];         /* 4: Processing unit type (ADC_PROCESS_STEREO_EXTENDER) */
  uint8_t st_npins;             /* 6: Number of input pins of this unit (1) */
  unit8_t st_srcid;             /* 7: ID of unit/terminal input is connected to */
  uint8_t st_nchan;             /* 8: Number of logic output channels */
  uint8_t st_config[4];         /* 9: Spatial location of channels */
  uint8_t st_names;             /* 13: String index to first channel name */
  uint8_t st_controls[2];       /* 14: controls
                                 *   Bits 0-1: Enable control
                                 *   Bits 2-3; Width control
                                 *   Bits 4-5: Cluster control
                                 *   Bits 6-7: Underflow control
                                 *   Bits 8-9: Overflow control
                                 *   Bits 10-15 Reserved */
  uint8_t st_processing;        /* 16: String index to name of processing unit */
};

#define USB_SIZEOF_ADC_DOLBYUNIT_DESC(nmodes) (17)

/* Extension Unit Descriptor */

struct adc_extunit_desc_s
{
  uint8_t xu_len;               /* 0: Descriptor length (16+p) */
  uint8_t xu_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t xu_subtype;           /* 2: Descriptor sub-type (ADC_AC_EXTENSION_UNIT) */
  uint8_t xu_unitid;            /* 3: Identifies unit in audio function */
  uint8_t xu_extcode[2];        /* 4: Vendor-specific code identifying the extension unit */
  uint8_t xu_npins;             /* 6: Number of input pins of this unit */
  uint8_t xu_variable[1];       /* 7-(7+(npins-1)): xu_srcid: ID of unit/terminal to which
                                 *   input pin n is connect, n=1,..,npins */
                                /* 8+npins: xu_nchan: Number of logic output channels */
                                /* 9+npins: xu_config: Spatial location of logical channels */
                                /* 13+npins: xu_names: String index to first channel name */
                                /* 14+npins: xu_controls:
                                 *   Bits 0-1: Enable control
                                 *   Bits 2-3: Cluster control
                                 *   Bits 4-5: Underflow control
                                 *   Bits 6-7: Overflow control */
                                /* 15+np;ins: xu_extunit: String index to unit name */
};

#define xu_srcid(n)    xu_variable[n]
#define xu_nchan(p)    xu_variable[(p)->xu_npins+1]
#define xu_config(p)   xu_variable[(p)->xu_npins+2]
#define xu_names(p)    xu_variable[(p)->xu_npins+6]
#define xu_controls(p) xu_variable[(p)->xu_npins+7]
#define xu_extunit(p)  xu_variable[(p)->xu_npins+8]

#define USB_SIZEOF_ADC_EXTUNIT_DESC(npins) (16+(npins))

/* Class-Specific AS Interface Descriptor */

struct adc_as_ifdesc_s
{
  uint8_t as_len;               /* 0: Descriptor length (9)*/
  uint8_t as_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t as_subtype;           /* 2: Descriptor sub-type (ADC_AS_GENERAL) */
  uint8_t as_terminal;          /* 3: ID of connected terminal */
  uint8_t as_controls;          /* 4: controls
                                 *   Bits 0-1: Active alternate setting control
                                 *   Bits 2-3: Valid alternate setting control
                                 *   Bits 4-7: Reserved */
  uint8_t as_format;            /* 5: Format type of audio streaming interface */
  uint8_t as_formats[4];        /* 6: Supported audio datat formats */
  uint8_t as_nchan;             /* 10: Number of physical channels in audo channel cluster */
  uint8_t as_config[4];         /* 11: Spatial location of channels */
  uint8_t as_names;             /* 15: String index to name of first channel */
};

#define USB_SIZEOF_ADC_AS_IFDESC 9

/* Encoder Descriptor */

struct adc_encoder_desc_s
{
  uint8_t as_len;               /* 0: Descriptor length (21) */
  uint8_t as_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t as_subtype;           /* 2: Descriptor sub-type (ADC_AS_ENCODER) */
  uint8_t as_encoderid;         /* 3: Identifies the encoder within the interface */
  uint8_t as_encoder;           /* 4: Identifies the encoder */
  uint8_t as_pad[3];            /*    (there is an apparent error in the spec) */
  uint8_t as_controls[4];       /* 8: Controls
                                 *    Bits 2-3: Quality Control
                                 *    Bits 4-5: VBR Control
                                 *    Bits 6-7: Type Control
                                 *    Bits 8-9: Underflow Control
                                 *    Bits 10-11: Overflow Control
                                 *    Bits 12-13: Encoder Error Control
                                 *    Bits 14-15: Param1 Control
                                 *    Bits 16-17: Param2 Control
                                 *    Bits 18-19: Param3 Control
                                 *    Bits 20-21: Param4 Control
                                 *    Bits 22-23: Param5 Control
                                 *    Bits 24-25: Param6 Control
                                 *    Bits 26-27: Param7 Control
                                 *    Bits 28-20: Param8 Control
                                 *    Bits 30-31: Reserved */
  uint8_t as_param[8];          /* 12: String index of purpose of parameter n-1, n=1-8 */
  uint8_t as_encoder:           /* 20: String index to the name of the encoder */
};

#define USB_SIZEOF_ADC_ENCODER_DESC 21

/* MPEG Decoder Descriptor */

struct adc_mpeg_decoder_desc_s
{
  uint8_t md_len;               /* 0: Descriptor length (10) */
  uint8_t md_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t md_subtype;           /* 2: Descriptor sub-type (ADC_AS_DECODER) */
  uint8_t md_decoderid;         /* 3: Identifies the decoder within the interface */
  uint8_t md_decoder;           /* 4: Identifies the decoder (ADC_DECODER_MPEG) */
  uint8_t md_capabilities[2];   /* 5: MPEG capabilities
                                 *    Bits 0-2: Layer support
                                 *      Bit 0:  Layer I
                                 *      Bit 1:  Layer II
                                 *      Bit 2:  Layer III
                                 *    Bit 3: MPEG-1 only.
                                 *    Bit 4: MPEG-1 dual-channel
                                 *    Bit 5: MPEG-2 second stereo
                                 *    Bit 6: MPEG-2 7.1 channel augmentation
                                 *    Bit 7: Adaptive multi-channel prediction
                                 *    Bits 8-9: MPEG-2 multilingual support
                                 *      00 = Not supported
                                 *      01 = Supported at Fs
                                 *      10 = Reserved
                                 *      11 = Supported at Fs and ½Fs.
                                 *    Bit 10: 
                                 *    Bit 11-15: Reserved */
  uint8_t md_features;          /* 7: MPEG features
                                 *    Bits 0-3: Reserved
                                 *    Bits 4-5 Internal dynamic range control
                                 *      00 = not supported
                                 *      01 = supported but not scalable
                                 *      10 = scalable, common boost and cut scaling value
                                 *      11 = scalable, separate boost and cut scaling value.
                                 *    Bits 6-7: Reserved */
  uint8_t md_controls;          /* 8: Controls:
                                 *    Bits 0-1: Underflow control
                                 *    Bits 2-3: Overflow control
                                 *    Bits 4-5: Decoder error control
                                 *    Bits 6-7: Reserved */
  uint8_t md_decoder;           /* 9: String index to the name of the decoder */
};

#define USB_SIZEOF_ADC_MPEG_DECODER_DESC 10

/* AC-3 Decoder Descriptor */

struct adc_ac3_decoder_desc_s
{
  uint8_t ad_len;               /* 0: Descriptor length (12) */
  uint8_t ad_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ad_subtype;           /* 2: Descriptor sub-type (ADC_AS_DECODER) */
  uint8_t ad_decoderid;         /* 3: Identifies the decoder within the interface */
  uint8_t ad_decoder;           /* 4: Identifies the decoder (ADC_DECODER_AC3) */
  uint8_t ad_id[4];             /* 5: Bitmap, 1=corresponding BSID mode supported */
  uint8_t ad_features;          /* 7: MPEG features
                                 *    Bit 0: RF mode
                                 *    Bit 1: Line mode
                                 *    Bit 2: Custom0 mode
                                 *    Bit 3: Custom1 mode
                                 *    Bits 4-5 Internal dynamic range control
                                 *      00 = not supported
                                 *      01 = supported but not scalable
                                 *      10 = scalable, common boost and cut scaling value
                                 *      11 = scalable, separate boost and cut scaling value.
                                 *    Bits 6-7: Reserved */
  uint8_t ad_controls;          /* 8: Controls:
                                 *    Bits 0-1: Underflow control
                                 *    Bits 2-3: Overflow control
                                 *    Bits 4-5: Decoder error control
                                 *    Bits 6-7: Reserved */
  uint8_t ad_decoder;           /* 9: String index to the name of the decoder */
};

#define USB_SIZEOF_ADC_AC3_DECODER_DESC 12

/* WMA Decoder Descriptor */

struct adc_wma_decoder_desc_s
{
  uint8_t wd_len;               /* 0: Descriptor length (9) */
  uint8_t wd_type;              /* 1: Descriptor type (ADC_CS_ENDPOINT) */
  uint8_t wd_subtype;           /* 2: Descriptor sub-type (ADC_AS_DECODER) */
  uint8_t wd_decoderid;         /* 3: Identifies the decoder within the interface */
  uint8_t wd_decoder;           /* 4: Identifies the decoder (ADC_DECODER_WMA) */
  uint8_t wd_profile[2];        /* 5: WMA profile
                                 *    Bit 0: WMA profile 1, L1
                                 *    Bit 1: WMA profile 2, L2
                                 *    Bit 2: WMA profile 3, L3
                                 *    Bit 3: WMA profile other, L
                                 *    Bit 4: WMA speech 1, S1
                                 *    Bit 5: WMA speech 2, S2
                                 *    Bit 6: WMAPro profile 1, M1
                                 *    Bit 7: WMAPro profile 2, M2
                                 *    Bit 8: WMAPro profile 3, M3
                                 *    Bit 9: WMAPro profile other, M
                                 *    Bit 10: WMA lossless decoding is supported
                                 *    Bits 11-15: Reserved */
  uint8_t wd_controls;          /* 7: Controls:
                                 *    Bits 0-1: Underflow control
                                 *    Bits 2-3: Overflow control
                                 *    Bits 4-5: Decoder error control
                                 *    Bits 6-7: Reserved */
  uint8_t wd_decoder;           /* 9: String index to the name of the decoder */
};

#define USB_SIZEOF_ADC_WMA_DECODER_DESC 9

/* DTS Decoder Descriptor */

struct adc_dts_decoder_desc_s
{
  uint8_t dd_len;               /* 0: Descriptor length (8) */
  uint8_t dd_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t dd_subtype;           /* 2: Descriptor sub-type (ADC_AS_DECODER) */
  uint8_t dd_decoderid;         /* 3: Identifies the decoder within the interface */
  uint8_t dd_decoder;           /* 4: Identifies the decoder (ADC_DECODER_DTS) */
  uint8_t dd_capabilities;      /* 5: DTS capabilities
                                 *    Bit 0: Core
                                 *    Bit 1: Lossless
                                 *    Bit 2: LBR
                                 *    Bit 3: MultipleStreamMixing
                                 *    Bit 4: DualDecode
                                 *    Bits 5-7: Reserved */
  uint8_t dd_controls;          /* 7: Controls:
                                 *    Bits 0-1: Underflow control
                                 *    Bits 2-3: Overflow control
                                 *    Bits 4-5: Decoder error control
                                 *    Bits 6-7: Reserved */
  uint8_t dd_decoder;           /* 9: String index to the name of the decoder */
};

#define USB_SIZEOF_ADC_DTS_DECODER_DESC 8

/* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */

struct adc_audio_epdesc_s
{
  uint8_t ae_len;               /* 0: Descriptor length (8) */
  uint8_t ae_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t ae_subtype;           /* 2: Descriptor sub-type (ADC_EPTYPE_GENERAL) */
  uint8_t ae_attr;              /* 3: Attributes: Bit 7: MaxPacketsOnly */
  uint8_t ae_controls;          /* 4 Controls
                                 *    Bits 0-1: Pitch control
                                 *    Bits 2-3: Data overrun control
                                 *    Bits 4-5: Data underrun control
                                 *    Bits 6-7: Reserved */
  uint8_t ae_units;             /* 5: Lock delay units
                                 *    0=undefined
                                 *    1=milliseconds
                                 *    2=Decoded PCM samples
                                 *    2-255=Reserved */
  uint8_t ae_delay[2];          /* 6: Lock delay */
};

#define USB_SIZEOF_ADC_AUDIO_EPDESC 8

/* Layout 1, Control CUR Parameter Block */

struct adc_l1_curparm_s
{
   uint8_t l1_cur;              /* 0: Setting of the CUR attribute of the addressed control */
};

#define USB_SIZEOF_ADC_LI_CURPARM 1

/* Layout 1, Control RANGE Parameter Block */

struct adc_l1_subrange_s packed_struct
{
   uint8_t l1_min;              /* 0: MIN attribute */
   uint8_t l1_max;              /* 1: MAX attribute */
   uint8_t l1_res;              /* 2: RES attribute */
};

struct adc_l1_rangeparm_s packed_struct
{
   uint8_t l1_nranges;          /* 0: Number of sub-ranges */
   struct adc_l1_subrange_s l1_subrange[1];
};

#define USB_SIZEOF_ADC_LI_RANGEPARM(nranges) (1+3*(nranges))

/* Layout 2, Control CUR Parameter Block */

struct adc_l2_curparm_s
{
   uint8_t l2_cur[2];           /* 0: Setting of the CUR attribute of the addressed control */
};

#define USB_SIZEOF_ADC_L2_CURPARM 2

/* Layout 2, Control RANGE Parameter Block */

struct adc_l2_subrange_s
{
   uint8_t l2_min[2];           /* 0: MIN attribute */
   uint8_t l2_max[2];           /* 2: MAX attribute */
   uint8_t l2_res[2];           /* 4: RES attribute */
};

struct adc_l2_rangeparm_s
{
   uint8_t l2_nranges[2];       /* 0: Number of sub-ranges */
   struct adc_l2_subrange_s l2_subrange[1];
};

#define USB_SIZEOF_ADC_L2_RANGEPARM(nranges) (2+6*(nranges))

/* Layout 2, Control CUR Parameter Block */

struct adc_l3_curparm_s
{
   uint8_t l3_cur[4];           /* 0: Setting of the CUR attribute of the addressed control */
};

#define USB_SIZEOF_ADC_L3_CURPARM 4

/* Layout 2, Control RANGE Parameter Block */

struct adc_l3_subrange_s
{
   uint8_t l3_min[4];           /* 0: MIN attribute */
   uint8_t l3_max[4];           /* 2: MAX attribute */
   uint8_t l3_res[4];           /* 4: RES attribute */
};

struct adc_l3_rangeparm_s
{
   uint8_t l3_nranges[2];       /* 0: Number of sub-ranges */
   struct adc_l3_subrange_s l3_subrange[1];
};

#define USB_SIZEOF_ADC_L3_RANGEPARM(nranges) (2+12*(nranges))

/* Cluster Control CUR Parameter Block */

struct adc_clustctrl_curparm_s
{
  uint8_t cc_nchan;             /* 0: Number of logical channels */
  uint8_t cc_config[4];         /* 1: Spatial location of channels */
  uint8_t cc_names;             /* 5: String index of first channel name */
}; 

#define USB_SIZEOF_ADC_CLUSTCTRL_CURPARM 6

/* Cluster Control CUR Parameter Block */

struct adc_connctrl_curparm_s
{
  uint8_t cc_nchan;             /* 0: Number of logical channels */
  uint8_t cc_config[4];         /* 1: Spatial location of channels */
  uint8_t cc_names;             /* 5: String index of first channel name */
}; 

#define USB_SIZEOF_ADC_CONNCTRL_CURPARM 6

/* Graphic Equalizer Control CUR Parameter Block */

struct adc_equalizer_curparm_s
{
  uint8_t eq_bands[4];          /* 0: A set bit indicates that the band is present */
  uint8_t eq_cur[[1];           /* 4: Setting for the band in bands bitset */
}; 

#define USB_SIZEOF_ADC_CONNCTRL_CURPARM(nbands) (4+(nbands))

/* Graphic Equalizer Control RANGE Parameter Block */

struct adc_eq_subrange_s packed_struct
{
   uint8_t eq_min;              /* 0: MIN attribute */
   uint8_t eq_max;              /* 1: MAX attribute */
   uint8_t eq_res;              /* 2: RES attribute */
};

struct adc_equalizer_rangeparm_s packed_struct
{
   uint8_t eq_nranges;          /* 0: Number of sub-ranges */
   struct adc_eq_subrange_s eq_subrange[1];
};

#define USB_SIZEOF_ADC_EQUALIZER_RANGEPARM(nranges) (1+3*(nranges))

/* Valid Alternate Settings Control CUR Parameter Block */

struct adc_altsettings_curparm_s
{
  uint8_t as_nsettings;         /* 0: Number of alternate settings */
  uint8_t as_settings[1];       /* 1-: Altnating setting n, n-1,..., nsettings */
};

#define USB_SIZEOF_ADC_ALTSETTINGS_CURPARM(nsettings) (1+(nsettings))

/* High/Low Scaling Control CUR Parameter Block */

struct adc_hilo_curparm_s
{
   uint8_t hl_lo;               /* 0: CUR value of the low level scaling control */
   uint8_t hl_hi;               /* 0: CUR value of the high level scaling control */
};

/* High/Low Scaling Control RANGE Parameter Block */

struct adc_hl_subrange_s packed_struct
{
   uint8_t hl_min;              /* 0: MIN attribute */
   uint8_t hl_max;              /* 1: MAX attribute */
   uint8_t hl_res;              /* 2: RES attribute */
};

struct adc_hilo_rangeparm_s packed_struct
{
   uint8_t hl_nranges[2];       /* 0: Number of sub-ranges */
   struct adc_hl_subrange_s hl_subrange[1];
};

#define USB_SIZEOF_ADC_HILO_RANGEPARM(nranges) (2+3*(nranges))

/* Interrupt Data Message Format */

struct adc_int_message_s
{
  uint8_t im_info;              /* 0: Bitmap
                                 *    Bit 0: Vender specific, 
                                 *    Bit 1: Interface or endpoint
                                 *    Bits 2-7: Reserved */
  uint8_t im_attr;              /* 1: The attribute that cause the interrupt */
  uint8_t im_value[2];          /* 2: CS is MS byte; CN or MCN in LS byte */
  uint8_t im_index[2];          /* 4: ID or zero in MS bytes; Interface or endpoint is LS byte */
};

#define USB_SIZEOF_ADC_INT_MESSAGE 6

/* Type I Format Type Descriptor */

struct adc_t1_format_desc_s
{
  uint8_t t1_len;               /* 0: Descriptor length (6) */
  uint8_t t1_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t t1_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t t1_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEI) */
  uint8_t t1_size;              /* 4: Number of bytes in one audio subslot, 1,2,3, or 4 */
  uint8_t fl_resolution;        /* 5: Number of bits used from audio subslot */
};

#define USB_SIZEOF_ADC_T1_FORMAT_DESC  6

/* Type II Format Type Descriptor */

struct adc_t2_format_desc_s
{
  uint8_t t2_len;               /* 0: Descriptor length (8) */
  uint8_t t2_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t t2_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t t2_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEII) */
  uint8_t t2_bitrate[2];        /* 4 Maximum number of bits per second */
  uint8_t t2_slotsperframe[2];  /* 6: Number of PCM audio slots in one encoded audio frame*/
};

#define USB_SIZEOF_ADC_T2_FORMAT_DESC  8

/* Type III Format Type Descriptor */

struct adc_t3_format_desc_s
{
  uint8_t t3_len;               /* 0: Descriptor length (6) */
  uint8_t t3_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t t3_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t t3_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEIII) */
  uint8_t t3_size;              /* 4: Number of bytes in one audio subslot (2) */
  uint8_t t3_resolution;        /* 5: Number of bits used from audio subslot */
};

#define USB_SIZEOF_ADC_T3_FORMAT_DESC  6

/* Type IV Format Type Descriptor */

struct adc_t4_format_desc_s
{
  uint8_t t4_len;               /* 0: Descriptor length (4) */
  uint8_t t4_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t t4_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t t4_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEIV) */
};

#define USB_SIZEOF_ADC_T4_FORMAT_DESC  6

/* Extended Type I Format Type Descriptor */

struct adc_x1_format_desc_s
{
  uint8_t x1_len;               /* 0: Descriptor length (8) */
  uint8_t x1_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t x1_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t x1_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_EXT_TYPEI) */
  uint8_t x1_size;              /* 4: Number of bytes in one audio subslo, 1,2,3, or 4 */
  uint8_t xl_resolution;        /* 5: Number of bits used from audio subslot */
  uint8_t x1_hdrlen;            /* 6: Size of packet header (in bytes) */
  uint8_t x1_ctrlsize;          /* 7: Size of control channel words (in bytes) */
  uint8_t x1_sbproto;           /* 8: Sideband protocol used in packet header and ctrl channel */
};

#define USB_SIZEOF_ADC_X1_FORMAT_DESC  8

/* Extended Type II Format Type Descriptor */

struct adc_x2_format_desc_s
{
  uint8_t x2_len;               /* 0: Descriptor length (10) */
  uint8_t x2_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t x2_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t x2_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEII) */
  uint8_t x2_bitrate[2];        /* 4 Maximum number of bits per second */
  uint8_t x2_samperframe[2];    /* 6: Number of PCM audio samples in one encoded audio frame*/
  uint8_t x2_hdrlen;            /* 8: Size of packet header (in bytes) */
  uint8_t x2_sbproto;           /* 9: Sideband protocol used in packet header and ctrl channel */
};

#define USB_SIZEOF_ADC_x2_FORMAT_DESC  10

/* Extended Type III Format Type Descriptor */

struct adc_x3_format_desc_s
{
  uint8_t x3_len;               /* 0: Descriptor length (8) */
  uint8_t x3_type;              /* 1: Descriptor type (ADC_CS_INTERFACE) */
  uint8_t x3_subtype;           /* 2: Descriptor sub-type (ADC_AS_FORMAT_TYPE) */
  uint8_t x3_fmttype;           /* 3: Identifies the format type (ADC_FORMAT_TYPEIII) */
  uint8_t x3_size;              /* 4: Number of bytes in one audio subslot (2) */
  uint8_t x3_resolution;        /* 5: Number of bits used from audio subslot */
  uint8_t x3_hdrlen;            /* 6: Size of packet header (in bytes) */
  uint8_t x3_sbproto;           /* 7: Sideband protocol used in packet header and ctrl channel */
};

#define USB_SIZEOF_ADC_X3_FORMAT_DESC  8

/* Hi-Res Presentation TimeStamp Layout*/

struct adc_hires_timestamp_s
{
  uint8_t hr_flags;             /* Bit32=valid */
  uint8_t hr_nsec[8];           /* Offset in nanoseconds from the beginning of the stream */
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_AUDIO_H */
