//
// Copyright (c) 2016 Airlango Ltd. All rights reserved.
//
// @file lanbao_isl_v2.h
//
// Lanbao TOF, hw version 2, with CRC-16 support
//
#ifndef EAGLE_TOF_LANBAO_ISL_V2_H_
#define EAGLE_TOF_LANBAO_ISL_V2_H_

#include "lanbao_isl.h"

class LanbaoIslV2 : public LanbaoIsl {
 public:
  LanbaoIslV2();
  virtual ~LanbaoIslV2();
  virtual TofModel model() const { return LANBAO_ISL_V2; }

 protected:
  virtual int Parse(const uint8_t* buffer, int length, bool* full_frame);
};

#endif
