NMEA library see: http://nmea.sourceforge.net/

Disclaimer

The National Marine Electronics Association (NMEA) has developed a specification that defines the interface between various pieces of marine electronic equipment. The standard permits marine electronics to send information to computers and to other marine equipment.
Most computer programs that provide real time position information understand and expect data to be in NMEA format. This data includes the complete PVT (position, velocity, time) solution computed by the GPS receiver. The idea of NMEA is to send a line of data called a sentence that is totally self contained and independent from other sentences. All NMEA sentences is sequences of ACSII symbols begins with a '$' and ends with a carriage return/line feed sequence and can be no longer than 80 characters of visible text (plus the line terminators).

Introduction

We present library in 'C' programming language for work with NMEA protocol. Small and easy to use. The library build on different compilers under different platforms (see below). The code was tested in real projects. Just download and try...

Features

- Analysis NMEA sentences and granting GPS data in C structures
- Generate NMEA sentences
- Supported sentences: GPGGA, GPGSA, GPGSV, GPRMC, GPVTG
- Multilevel architecture of algorithms
- Additional functions of geographical mathematics and work with navigation data

Supported (tested) platforms

- Microsoft Windows (MS Visual Studio 8.0, GCC)
- Windows Mobile, Windows CE (MS Visual Studio 8.0)
- UNIX (GCC)

Licence: LGPL
