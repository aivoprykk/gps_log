#ifndef GPY_H
#define GPY_H 1
/*
MIT License
First draft of C lib for creating the minimal OpenGNSS format out of the ubx nav pvt frame !
Copyright (c) 2022 RP6conrad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//char gpy_Tx[256]; 
//Struct definitions
struct GPY_Header{
	uint8_t Type_identifier;	//Frame identifier, header = 0xF0
    uint8_t Flags;
    uint16_t Length;    //length = 6 + 4 * STRING_IO_LENGTH + 2 = 72
	uint16_t DeviceType;//ublox = 2
	char deviceDescription[16];
	char deviceName[16];
	char serialNumber[16];
	char firmwareVersion[16];
	uint16_t Checksum;
} __attribute__((__packed__));

struct GPY_Frame {	
    uint8_t   Type_identifier;	//Frame identifier for full frame = 0xE0
    uint8_t   Flags;
	uint16_t  HDOP;	//HDOP
	int64_t   Unix_time;//ms 
	uint32_t  Speed; //mm/s
	uint32_t  Speed_error;//sAccCourse_Over_Ground;
	int32_t   Latitude;
	int32_t   Longitude;
	int32_t   COG;//Course over ground
	uint8_t   Sat; //number of sats
	uint8_t	  fix;
	uint16_t  Checksum;
} __attribute__((__packed__)); //total = 36 bytes

struct GPY_Frame_compressed {	
    uint8_t   Type_identifier;	//Frame identifier for compressed frame = 0xD0
    uint8_t   Flags;
	uint16_t  HDOP;	//HDOP
	int16_t   delta_time;//ms 
	int16_t   delta_Speed; //mm/
	int16_t   delta_Speed_error;//sAccCourse_Over_Ground;
	int16_t   delta_Latitude;
	int16_t   delta_Longitude;
	int16_t   delta_COG;//delta (course / 1000) !
	uint8_t   Sat; //number of sats
	uint8_t	  fix;
	uint16_t  Checksum;
  } __attribute__((__packed__)); //total = 20 bytes

//Functions definitions 
/*
 * https://en.wikipedia.org/wiki/Fletcher%27s_checksum
 * Length of the frame in bytes (last 2 bytes = checksum),
 * pointer to the frame bytes buffer.
 * 2byte checksum is returned
 */
uint16_t Fletcher16( uint8_t *data, int count );
//https://community.particle.io/t/make-epoch-timestamp-maketime-solved/10013
/*
time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss);
*/
struct gps_context_s;
void log_GPY_Header(const struct gps_context_s *context);
void log_GPY(struct gps_context_s *context);

#ifdef __cplusplus
}
#endif
#endif