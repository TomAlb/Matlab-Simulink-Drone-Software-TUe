/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
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

/* @file ASHTECH protocol definitions */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

#define OPTITRACK_RECV_BUFFER_SIZE 128

class GPSDriverOptitrack : public GPSHelper
{
public:
    GPSDriverOptitrack(GPSCallbackPtr callback, void *callback_user,
                        struct vehicle_gps_position_s *gps_position,
                        struct vehicle_gps_optitrack_s *gps_optitrack,
                        struct satellite_info_s *satellite_info);
    virtual ~GPSDriverOptitrack() = default;

    int receive(unsigned timeout);
    int configure(unsigned &baud, OutputMode output_mode);
    int parseChar(uint8_t b);


private:
    void decodeInit(void);
    int handleMessage(int len);
    int receive_config(unsigned timeout);
    int parseChar_config(uint8_t b);


    /** Read int OPTITRACK parameter */
    int32_t read_int();
    /** Read float OPTITRACK parameter */
    double read_float();
    /** Read char OPTITRACK parameter */
    char read_char();

    enum optitrack_decode_state_t {
        NME_DECODE_UNINIT,
        NME_DECODE_GOT_SYNC1,
        NME_DECODE_GOT_ASTRIKS,
        NME_DECODE_GOT_FIRST_CS_BYTE
    };

    struct satellite_info_s *_satellite_info {nullptr};
    struct vehicle_gps_position_s *_gps_position {nullptr};
    struct vehicle_gps_optitrack_s *_gps_optitrack {nullptr};
    uint64_t _last_timestamp_time{0};
    int _optitracklog_fd{-1};

    optitrack_decode_state_t _decode_state{NME_DECODE_UNINIT};
    optitrack_decode_state_t _decode_state_config{NME_DECODE_UNINIT};
    uint8_t  _rx_buffer[OPTITRACK_RECV_BUFFER_SIZE] {};
    uint16_t _rx_buffer_bytes{};
    bool _got_pashr_pos_message{false}; /**< If we got a PASHR,POS message we will ignore GGA messages */
    bool _parse_error{}; /**< parse error flag */
    char *_parse_pos{}; /**< parse position */
};
