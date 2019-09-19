/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <px4_log.h>
#include "optitrack.h"

GPSDriverOptitrack::GPSDriverOptitrack(GPSCallbackPtr callback, void *callback_user,
                   struct vehicle_gps_position_s *gps_position,
                   struct vehicle_gps_optitrack_s *gps_optitrack,
                   struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _satellite_info(satellite_info),
    _gps_position(gps_position),
    _gps_optitrack(gps_optitrack)
{
    decodeInit();
    _decode_state = NME_DECODE_UNINIT;
    _decode_state_config = NME_DECODE_UNINIT;
    _rx_buffer_bytes = 0;
    #define TIMEOUT_5HZ 500
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int GPSDriverOptitrack::handleMessage(int len)
{
    char *endp;

    if(len < 7){
        return 0;
    }

    int uiCalcComma = 0;    // Number of commas in message

    for (int i = 0; i < len; i++){
        if (_rx_buffer[i] == ',') {uiCalcComma++;}
    }

    char *bufptr = (char *)(_rx_buffer + 6);
    int ret = 0;

    if ((memcmp(_rx_buffer + 0, "$OPTI,", 6) == 0) && (uiCalcComma == 10)){

        /*
        Optitrack data information based on this specific object
        An example of the OPTI message string is:

        $OPTI,684154,1,-0.087,-0.352,0.0410.544,-0.721,-0.367,0.004*

        ZDA message fields
        Field   Meaning
        0       Message ID $OPTI
        1       Frame number
        2       Rigid body ID
        3       X-position
        4       Y-position
        5       Z-position
        6       qx-orientation
        7       qy-orientation
        8       qz-orientation
        9       qw-orientation
        10      mean error
        11       The checksum data, always begins with *
        */

        double optitrack_time __attribute__((unused)) = 0.0, x_pos = 0.0, y_pos = 0.0, z_pos = 0.0, qx_orient = 0.0, qy_orient = 0.0, qz_orient = 0.0, qw_orient = 0.0, mean_error = 0.0;
        int frame_number __attribute__((unused)) = 0;
        int rigidbody_id __attribute__((unused)) = 0;
        int fix_quality __attribute__((unused)) = 0;

        if (bufptr && *(++bufptr) != ',') { frame_number = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { rigidbody_id = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { x_pos = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { y_pos = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { z_pos = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { qx_orient = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { qy_orient = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { qz_orient = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { qw_orient = strtod(bufptr, &endp); bufptr = endp; }
        if (bufptr && *(++bufptr) != ',') { mean_error = strtod(bufptr, &endp); bufptr = endp; }

        /* convert from degrees, minutes and seconds, to degrees * 1e7 */
        _gps_position->lat = x_pos*1000;
        _gps_position->lon = y_pos*1000;
        _gps_position->alt = z_pos*1000;
        _rate_count_lat_lon++;
        PX4_INFO("Position: %0.3f | %0.3f | %0.3f", x_pos, y_pos, z_pos);

        if (fix_quality <= 0.1) {
            _gps_position->fix_type = 0;
        } else{
            /*
             * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
             * In case op Optitrack, if accuracy is below 10 cm, 3D fix, otherwise, no fix
             */
            _gps_position->fix_type = 3;
        }

        _gps_position->eph = mean_error;
        _gps_position->epv = mean_error;
        _gps_position->satellites_used = 8;
        //_gps_position->alt_ellipsoid = alt_ellipsoid;
        _gps_position->vel_m_s = 0;             /**< GPS ground speed (m/s) */
        _gps_position->vel_n_m_s = 0;           /**< GPS ground speed in m/s */
        _gps_position->vel_e_m_s = 0;           /**< GPS ground speed in m/s */
        _gps_position->vel_d_m_s = 0;           /**< GPS ground speed in m/s */
        _gps_position->cog_rad = 0;             /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
        _gps_position->vel_ned_valid = false;    /**< Flag to indicate if NED speed is valid */
        _gps_position->c_variance_rad = 0.1f;
        // Do something smart with frame number
        _gps_position->timestamp = gps_absolute_time();

        // Set optitrack Uorb message
        //_gps_optitrack->frame_number = static_cast<uint32>(frame_number);
        //_gps_optitrack->pos_x = 0;
        mean_error = qx_orient*qy_orient*qz_orient*qw_orient;


        _gps_optitrack->timestamp = hrt_absolute_time();
        _gps_optitrack->frame_number = 10;
        _gps_optitrack->rigid_body_ID = 10;
        _gps_optitrack->pos_x = x_pos*1000;
        _gps_optitrack->pos_y = y_pos*1000;
        _gps_optitrack->pos_z = z_pos*1000;
        _gps_optitrack->orientation_qx = 10.10;
        _gps_optitrack->orientation_qy = 20.20;
        _gps_optitrack->orientation_qz = 30.30;
        _gps_optitrack->orientation_qw = 40.40;
        _gps_optitrack->mean_error = 0.0110;

        ret = 1;

    }

    if (ret > 0) {
        _gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
    }
    return ret;

}

int GPSDriverOptitrack::receive_config(unsigned timeout)
{
    {
        PX4_INFO("RECEIVE_CONFIG RUNNING");
        uint8_t buf[GPS_READ_BUFFER_SIZE];

        /* timeout additional to poll */
        uint64_t time_started = gps_absolute_time();

        int j = 0;
        ssize_t bytes_count = 0;

        while (true) {

            /* pass received bytes to the packet decoder */
            while (j < bytes_count) {
                int l = 0;
                //parseChar_config(buf[j]);
                if ((l = parseChar_config(buf[j])) > 0) {
                    /* return to configure during configuration or to the gps driver during normal work
                     * if a packer has arrived */
                    return 1;
                }

                j++;
            }

            /* everything is read */
            j = bytes_count = 0;

            /* then poll or read for new data */
            int ret = read(buf, sizeof(buf), timeout * 2);

            if (ret < 0) {
                //PX4_INFO("ret < 0");
                /* something went wrong when polling */
                return -1;
            } else if (ret == 0) {
                //PX4_INFO("ret == 0");
                /* Timeout while polling or just nothing read if reading, let's
                 * stay here, and use timeout below. */
            } else if (ret > 0) {
                //PX4_INFO("ret > 0");
                /* if we have new data from GPS, go handle it */
                bytes_count = ret;
                //return ret;
            }

            /* in case we get crap from GPS or time out */
            if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
                //PX4_INFO("TIMEOUT");
                return -1;
            }

        }
    }
}

int GPSDriverOptitrack::receive(unsigned timeout)
{
    {
        uint8_t buf[GPS_READ_BUFFER_SIZE];

        /* timeout additional to poll */
        uint64_t time_started = gps_absolute_time();

        int j = 0;
        ssize_t bytes_count = 0;

        while (true) {
//            uint64_t time_started = gps_absolute_time();
            /* pass received bytes to the packet decoder */
            while (j < bytes_count) {
                int l = 0;

                if ((l = parseChar(buf[j])) > 0) {
                    /* return to configure during configuration or to the gps driver during normal work
                     * if a packer has arrived */
                    if (handleMessage(l) > 0) {
                        return 1;
                    }
                }

                j++;
            }

            /* everything is read */
            j = bytes_count = 0;

            /* then poll or read for new data */
            int ret = read(buf, sizeof(buf), timeout * 2);

            if (ret < 0) {
                /* something went wrong when polling */
                return -1;
            } else if (ret == 0) {
                /* Timeout while polling or just nothing read if reading, let's
                 * stay here, and use timeout below. */
            } else if (ret > 0) {
                /* if we have new data from GPS, go handle it */
                bytes_count = ret;
            }

            /* in case we get crap from GPS or time out */
            if (time_started + timeout * 1000 < gps_absolute_time() && bytes_count != 0) {
                return -1;
            }

        }
    }
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverOptitrack::parseChar(uint8_t b)
{
    int iRet = 0;

    switch (_decode_state) {
    /* First, look for sync1 */
    case NME_DECODE_UNINIT:
        if (b == '$') {
            _decode_state = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
            _rx_buffer[_rx_buffer_bytes++] = b;
        }
        break;

    case NME_DECODE_GOT_SYNC1:
        if (b == '$') {
            _decode_state = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
        } else if (b == '*') {
            _decode_state = NME_DECODE_GOT_ASTRIKS;
        }

        if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
            _decode_state = NME_DECODE_UNINIT;
            _rx_buffer_bytes = 0;
        } else {
            _rx_buffer[_rx_buffer_bytes++] = b;
        }
        break;

    case NME_DECODE_GOT_ASTRIKS:
        _rx_buffer[_rx_buffer_bytes++] = b;
        _decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
        break;

    case NME_DECODE_GOT_FIRST_CS_BYTE:
        _rx_buffer[_rx_buffer_bytes++] = b;
        uint8_t checksum = 0;
        uint8_t *buffer = _rx_buffer;
        uint8_t *buffend = _rx_buffer + _rx_buffer_bytes - 3;

        for (; buffer < buffend; buffer++) { checksum ^= *buffer; }

        if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
            (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
            iRet = _rx_buffer_bytes;
        }

        _decode_state = NME_DECODE_UNINIT;
        _rx_buffer_bytes = 0;
        break;


    }

    return iRet;
}

int GPSDriverOptitrack::parseChar_config(uint8_t b)
{
    int CS_Confirm = 0;
    switch (_decode_state_config) {
    /* First, look for sync1 */
    case NME_DECODE_UNINIT:
        //PX4_INFO("NME_DECODE_UNINIT");
        if (b == '$') {
            _decode_state_config = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
            _rx_buffer[_rx_buffer_bytes++] = b;
        }
        break;

    case NME_DECODE_GOT_SYNC1:
        //PX4_INFO("NME_DECODE_GOT_SYNC1");
        if (b == '$') {
            //PX4_INFO("GOT DOLLAR");
            _decode_state_config = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
        } else if (b == '*') {
            //PX4_INFO("GOT ASTRIKS");
            _decode_state_config = NME_DECODE_GOT_ASTRIKS;
        }

        if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
            _decode_state_config = NME_DECODE_UNINIT;
            _rx_buffer_bytes = 0;
        } else {
            _rx_buffer[_rx_buffer_bytes++] = b;
        }
        break;

    case NME_DECODE_GOT_ASTRIKS:
        //PX4_INFO("NME_DECODE_ASTRIKS");
        _rx_buffer[_rx_buffer_bytes++] = b;
        _decode_state_config = NME_DECODE_GOT_FIRST_CS_BYTE;
        break;

    case NME_DECODE_GOT_FIRST_CS_BYTE:
        //PX4_INFO("NME_DECODE_GOT_FIRST_CS_BYTE");
        _rx_buffer[_rx_buffer_bytes++] = b;
        uint8_t checksum = 0;
        uint8_t *buffer = _rx_buffer;
        uint8_t *buffend = _rx_buffer + _rx_buffer_bytes - 3;
        for (; buffer < buffend; buffer++) {checksum ^= *buffer;
            //PX4_INFO("%c%c - %c%c", HEXDIGIT_CHAR(checksum >> 4), HEXDIGIT_CHAR(checksum & 0x0F), *(_rx_buffer + _rx_buffer_bytes - 2),  *(_rx_buffer + _rx_buffer_bytes - 1));
            //printf("%c",  *buffer);
            //printf("%x|", checksum);
        }
        //printf("\n");
        //PX4_INFO("%d", buffend);
        //printf("%c",  *buffer);
        //buffer++; printf("%c",  *buffer);
        //buffer++; printf("%c",  *buffer);
        //printf("\n");

        //PX4_INFO("%x", checksum);
        //PX4_INFO("%c%c - %c%c", HEXDIGIT_CHAR(checksum & 0xF0), HEXDIGIT_CHAR(checksum & 0x0F), *(_rx_buffer + _rx_buffer_bytes - 2),  *(_rx_buffer + _rx_buffer_bytes - 1));

        if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
            (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
            //PX4_INFO("CS_CONFIRM");
            CS_Confirm = 1;
        }

        _decode_state_config = NME_DECODE_UNINIT;
        _rx_buffer_bytes = 0;
        break;


    }
    return CS_Confirm;
}


void GPSDriverOptitrack::decodeInit()
{

}

/*
 *  Marvelmind board configuration script
 */

int GPSDriverOptitrack::configure(unsigned &baudrate, OutputMode output_mode)
{
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("OPTITRACK: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }

    /* try different baudrates */
    //const unsigned baudrates_to_try[] = {9600, 19200, 38400, 57600, 115200};
    const unsigned baudrates_to_try[] = {115200};

    for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++){
        baudrate = baudrates_to_try[baud_i];
        PX4_INFO("TESTING %d", baudrate);

        setBaudrate(baudrate);

        if (GPSDriverOptitrack::receive_config(TIMEOUT_5HZ) == 1) {
            return 0;
        }

    }

    return -1;

}




