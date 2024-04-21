#ifndef GPX_H
#define GPX_H 1
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
//See https://www.topografix.com/gpx_manual.asp#creator
//Doppler speed is not part of the gpx 1.1 frame, speed is then calculated as distance/time !!!
//gpx 1.0 is used here ! 
//https://logiqx.github.io/gps-wizard/gpx/
//Always 1Hz points

#ifdef __cplusplus
extern "C" {
#endif

#define GPX_HEADER 0
#define GPX_FRAME 1
#define GPX_END 2

struct gps_context_s;
void log_GPX(struct gps_context_s *context, int part);

#ifdef __cplusplus
}
#endif
#endif