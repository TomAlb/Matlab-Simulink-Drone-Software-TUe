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

#include "marv.h"

GPSDriverMarvelmind::GPSDriverMarvelmind(GPSCallbackPtr callback, void *callback_user,
                   struct vehicle_gps_position_s *gps_position,
                   struct satellite_info_s *satellite_info) :
    GPSHelper(callback, callback_user),
    _satellite_info(satellite_info),
    _gps_position(gps_position)
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

int GPSDriverMarvelmind::handleMessage(int len)
{
    char *endp;

    if(len < 7){
        return 0;
    }

    int uiCalcComma = 0;

    for (int i = 0; i < len; i++){
        if (_rx_buffer[i] == ',') {uiCalcComma++;}
    }

    char *bufptr = (char *)(_rx_buffer + 6);
    int ret = 0;

    //PX4_INFO("Msg: %s", _rx_buffer);

    if ((memcmp(_rx_buffer + 3, "RMC,", 3) == 0)) {
           /*
            An example of the GSV message string is:

            $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

            GPRMC message field
            Field  Meaning
            0      Message ID $GPRMC
            1      UTC of position fix
            2      Status:
                       A: active
                       V: void
            3      Latitude
            4      Longitude
            5      Speed over the ground in knots
            6      Track angle in degrees (True)
            7      Date
            8      Magnetic variation in degrees
            The checksum data, always begins with *
            */

           double marvelmind_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, speed_n = 0.0, magvar = 0.0, track = 0.0;
           int date __attribute__((unused)) = 0;
           char ns = '?', ew = '?', ewmag = '?', messtat = '?';

           if (bufptr && *(++bufptr) != ',') { marvelmind_time = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { messtat = *(bufptr++); }

           if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

           if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

           if (bufptr && *(++bufptr) != ',') { speed_n = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { track = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { date = strtol(bufptr, &endp, 10); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { magvar = strtod(bufptr, &endp); bufptr = endp; }

           if (bufptr && *(++bufptr) != ',') { ewmag = *(bufptr++); }


           if (ns == 'S') {
               lat = -lat;
           }

           if (ew == 'W') {
               lon = -lon;
           }

           if (ewmag == 'W') {
               magvar = -magvar;
           }

           if (messtat == 'W') {
               magvar = magvar;
           }



           /* convert from degrees, minutes and seconds, to degrees * 1e7 */
           _gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
           _gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
           _rate_count_lat_lon++;
           _gps_position->vel_m_s = speed_n * 1.85;
           _gps_position->cog_rad = static_cast<int>(track * 2 * M_PI / 360.0 * 1e-5);
           //_gps_position->cog_rad = static_cast<int>(track * double(3.0*M_PI/360.0 * 1e-5f));


           /*
            * Do something smart with:
            *  track
            *  date
            *  magvar
            */
           _gps_position->timestamp = gps_absolute_time();
           ret = 1;


    } else if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (uiCalcComma == 14)){
    /*
    Time, position, and fix related data
    An example of the GBS message string is:

    $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M-25.669,M,2.0,0031*4F

    Note - The data string exceeds the Marvelmind standard length.
    GGA message fields
    Field   Meanings
    0       Message ID $GPGGA
    1       UTC of position fix
    2       Latitude
    3       Direction of Latitude:
                N: North
                S: South
    4       Longitude
    5       Direction of longitude:
                E: East
                W: West
    6       GPS Quality indicator:
                0: Fix not valid
                1: GPS fix
                2: Differential GPS fix, PmniSTAR VBS
                4: Real-Time Kinematic, fixed integers
                5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or localation RTK
    7       Number of SVs in use, range from 00 through to 24+
    8       HDOP
    9       Orthometric height (MSL reference)
    10      M: unit of measure for orthometric height is meters
    11      Geoid separation
    12      M: geoid separation measured in meters
    13      Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
    14      Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received.
    15
    The checksum data, always begins with *
    Note - If a user-defined geoid model, or an inclined
    */

    double marvelmind_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0, alt_ellipsoid = 0.0;
    int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
    double hdop __attribute__((unused)) = 99.9;
    char ns = '?', ew = '?', M = '?';

    if (bufptr && *(++bufptr) != ',') { marvelmind_time = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { M = *(bufptr++); }

    if (bufptr && *(++bufptr) != ',') { alt_ellipsoid = strtod(bufptr, &endp); bufptr = endp; }

    if (bufptr && *(++bufptr) != ',') { M = *(bufptr++); }

    if (ns == 'S') {
        lat = -lat;
    }

    if (ew == 'W') {
        lon = -lon;
    }

    if (M == 'M') {
        alt = alt;
    }

    /* convert from degrees, minutes and seconds, to degrees * 1e7 */
    _gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
    _gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
    _gps_position->alt = static_cast<int>(alt * 1000);
    _rate_count_lat_lon++;

    if (fix_quality <= 0) {
        _gps_position->fix_type = 0;
    } else{
        /*
         * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality
         * and since value 3 is not being used, I "moved" value 5 to 3 add it to _gps_position->fix_type
         */
        if (fix_quality == 5) { fix_quality = 3; }

        /*
         * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
         */
        _gps_position->fix_type = 3 + fix_quality - 1;
    }

    _gps_position->hdop = hdop;
    _gps_position->satellites_used = num_of_sv;
    _gps_position->alt_ellipsoid = alt_ellipsoid;
    _gps_position->vel_m_s = 0;             /**< GPS ground speed (m/s) */
    _gps_position->vel_n_m_s = 0;           /**< GPS ground speed in m/s */
    _gps_position->vel_e_m_s = 0;           /**< GPS ground speed in m/s */
    _gps_position->vel_d_m_s = 0;           /**< GPS ground speed in m/s */
    _gps_position->cog_rad = 0;             /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
    _gps_position->vel_ned_valid = true;    /**< Flag to indicate if NED speed is valid */
    _gps_position->c_variance_rad = 0.1f;

    _gps_position->timestamp = gps_absolute_time();
    ret = 1;

    } else if ((memcmp(_rx_buffer + 3, "VTG,", 3) == 0)){
        /*
         Track made good and ground speed. An example of the GSV message string is:

         $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*34

         VTG message fields
         Field  Meaning
         0      Message ID $GPVTG
         1      Track made good (degrees true)
         2      T: track made good is relative to true north
         3      Track made good (degrees magnetic)
         4      M: track made good is relative to magnetic north
         5      Speed in knots
         6      N: Speed is measured in knots
         7      Speed over ground in kilometers/hour (kph)
         8      K: speed over ground is measured in kph
         The checksum data, always begins with *
         */

        /*
        double speed_n __attribute__((unused)) = 0.0, speed_k = 0.0, track_made_good_t = 0.0, track_made_good_m = 0.0;
        char K = '?', N = '?', M = '?', T = '?';

        if (bufptr && *(++bufptr) != ',') { track_made_good_t = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { T = *(bufptr++); }

        if (bufptr && *(++bufptr) != ',') { track_made_good_m = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { M = *(bufptr++); }

        if (bufptr && *(++bufptr) != ',') { speed_n = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { N = *(bufptr++); }

        if (bufptr && *(++bufptr) != ',') { speed_k = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { K = *(bufptr++); }
        */

        /*
         * Do something smart with this data
         */
        _gps_position->timestamp = gps_absolute_time();
        ret = 1;

    }  else if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (uiCalcComma == 6)){

        /*
        UTC day, month, and year, and local time zone offset
        An example of the ZDA message string is:

        $GPZDA,172809.456,12,07,1996,00,00*45

        ZDA message fields
        Field   Meaning
        0       Message ID $GPZDA
        1       UTC
        2       Day, ranging between 01 and 31
        3       Month, ranging between 01 and 12
        4       Year
        5       Local time zone offset from GMT, ranging from 00 through 13 hours
        6       Local time zone offset from GMT, ranging from 00 through 59 minutes
        7       The checksum data, always begins with *
        Fields 5 and 6 together yield the total offset. For example, if field 5 is -5 and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
        */

        double marvelmind_time = 0.0;
        int day = 0, month = 0, year = 0, local_time_off_hour __attribute__((unused)) = 0,
                local_time_off_min __attribute__((unused)) = 0;

        if (bufptr && *(++bufptr) != ',') { marvelmind_time = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { local_time_off_hour = strtol(bufptr, &endp, 10); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { local_time_off_min = strtol(bufptr, &endp, 10); bufptr = endp; }

        int marvelmind_hour   = static_cast<int>(marvelmind_time / 10000);
        int marvelmind_minute = static_cast<int>((marvelmind_time - marvelmind_hour * 10000) / 100);
        double marvelmind_sec = static_cast<double>(marvelmind_time - marvelmind_hour * 10000 - marvelmind_minute * 100);

        /*
         *  Convert to unix timestamp
         */
        struct tm timeinfo = {};
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = marvelmind_hour;
        timeinfo.tm_min = marvelmind_minute;
        timeinfo.tm_sec = int(marvelmind_sec);
        timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME
        time_t epoch = mktime(&timeinfo);

        if (epoch > GPS_EPOCH_SECS) {
            uint64_t usecs = static_cast<uint64_t>((marvelmind_sec - static_cast<uint64_t>(marvelmind_sec))) * 1000000;

            // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
            // and control its drift. Since we rely on the HRT for our monotonic
            // clock, updating it from time to time is safe.

            timespec ts{};
            ts.tv_sec = epoch;
            ts.tv_nsec = usecs * 1000;

            setClock(ts);

            _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
            _gps_position->time_utc_usec += usecs;

        } else {
            _gps_position->time_utc_usec = 0;
        }
#else
        _gps_position->time_utc_usec = 0;
#endif

        _last_timestamp_time = gps_absolute_time();
        _gps_position->timestamp = gps_absolute_time();
        ret = 1;

    }

    if (ret > 0) {
        _gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
    }
    return ret;

}

int GPSDriverMarvelmind::receive_config(unsigned timeout)
{
    {
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
                /* something went wrong when polling */
                return -1;
            } else if (ret == 0) {
                /* Timeout while polling or just nothing read if reading, let's
                 * stay here, and use timeout below. */
            } else if (ret > 0) {
                /* if we have new data from GPS, go handle it */
                bytes_count = ret;
                //return ret;
            }

            /* in case we get crap from GPS or time out */
            if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
                return -1;
            }

        }
    }
}

int GPSDriverMarvelmind::receive(unsigned timeout)
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
            if (time_started + timeout * 1000 < gps_absolute_time()) {
                return -1;
            }

        }
    }
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverMarvelmind::parseChar(uint8_t b)
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
        uint8_t *buffer = _rx_buffer + 1;
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

int GPSDriverMarvelmind::parseChar_config(uint8_t b)
{
    int CS_Confirm = 0;

    switch (_decode_state_config) {
    /* First, look for sync1 */
    case NME_DECODE_UNINIT:
        if (b == '$') {
            _decode_state_config = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
            _rx_buffer[_rx_buffer_bytes++] = b;
        }
        break;

    case NME_DECODE_GOT_SYNC1:
        if (b == '$') {
            _decode_state_config = NME_DECODE_GOT_SYNC1;
            _rx_buffer_bytes = 0;
        } else if (b == '*') {
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
        _rx_buffer[_rx_buffer_bytes++] = b;
        _decode_state_config = NME_DECODE_GOT_FIRST_CS_BYTE;
        break;

    case NME_DECODE_GOT_FIRST_CS_BYTE:
        _rx_buffer[_rx_buffer_bytes++] = b;
        uint8_t checksum = 0;
        uint8_t *buffer = _rx_buffer + 1;
        uint8_t *buffend = _rx_buffer + _rx_buffer_bytes - 3;

        for (; buffer < buffend; buffer++) { checksum ^= *buffer; }

        if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
            (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
            CS_Confirm = 1;
        }

        _decode_state_config = NME_DECODE_UNINIT;
        _rx_buffer_bytes = 0;
        break;


    }
    return CS_Confirm;
}


void GPSDriverMarvelmind::decodeInit()
{

}

/*
 *  Marvelmind board configuration script
 */

int GPSDriverMarvelmind::configure(unsigned &baudrate, OutputMode output_mode)
{
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("MARVELMIND: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }

    /* try different baudrates */
    const unsigned baudrates_to_try[] = {4800, 9600, 19200, 38400, 57600, 115200, 500000};

    for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++){
        baudrate = baudrates_to_try[baud_i];
        setBaudrate(baudrate);
        if (GPSDriverMarvelmind::receive_config(TIMEOUT_5HZ) == 1) {
            return 0;
        }

    }

    return -1;

}




