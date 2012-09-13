/****************************************************************************
 * netutils/thttpd/mime_types.h
 * Provides mappings between filename extensions and MIME types and encodings.
 *
 * Based on mime_encodings.txt and mime_types.txt by Jef Poskanser which
 * contained no copyright information.
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __NETUTILS_THTTPD_MIME_TYPES_H
#define __NETUTILS_THTTPD_MIME_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mime_entry
{
  char   *ext;
  size_t  ext_len;
  char   *val;
  size_t  val_len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A list of file extensions followed by the corresponding MIME encoding.
 * Extensions not found in the table proceed to the mime_types table.
 * Must be ordered by extension so to support binary searches.
 */

static struct mime_entry enc_tab[] =
{
  { "Z",       0, "compress",   0 },
  { "gz",      0, "gzip",       0 },
  { "uu",      0, "x-uuencode", 0 },
};
static const int n_enc_tab = sizeof(enc_tab) / sizeof(*enc_tab);

/* A list of file extensions followed by the corresponding MIME type.
 * Extensions not found in the table are returned as text/plain.
 * Must be ordered by extension so to support binary searches.
 */

static struct mime_entry typ_tab[] =
{
  { "a",       0, "application/octet-stream", 0 },
  { "aab",     0, "application/x-authorware-bin", 0 },
  { "aam",     0, "application/x-authorware-map", 0 },
  { "aas",     0, "application/x-authorware-seg", 0 },
  { "ai",      0, "application/postscript", 0 },
  { "aif",     0, "audio/x-aiff", 0 },
  { "aifc",    0, "audio/x-aiff", 0 },
  { "aiff",    0, "audio/x-aiff", 0 },
  { "asc",     0, "text/plain", 0 },
  { "asf",     0, "video/x-ms-asf", 0 },
  { "asx",     0, "video/x-ms-asf", 0 },
  { "au",      0, "audio/basic", 0 },
  { "avi",     0, "video/x-msvideo", 0 },
  { "bcpio",   0, "application/x-bcpio", 0 },
  { "bin",     0, "application/octet-stream", 0 },
  { "bmp",     0, "image/bmp", 0 },
  { "cdf",     0, "application/x-netcdf", 0 },
  { "class",   0, "application/x-java-vm", 0 },
  { "cpio",    0, "application/x-cpio", 0 },
  { "cpt",     0, "application/mac-compactpro", 0 },
  { "crl",     0, "application/x-pkcs7-crl", 0 },
  { "crt",     0, "application/x-x509-ca-cert", 0 },
  { "csh",     0, "application/x-csh", 0 },
  { "css",     0, "text/css", 0 },
  { "dcr",     0, "application/x-director", 0 },
  { "dir",     0, "application/x-director", 0 },
  { "djv",     0, "image/vnd.djvu", 0 },
  { "djvu",    0, "image/vnd.djvu", 0 },
  { "dll",     0, "application/octet-stream", 0 },
  { "dms",     0, "application/octet-stream", 0 },
  { "doc",     0, "application/msword", 0 },
  { "dtd",     0, "text/xml", 0 },
  { "dump",    0, "application/octet-stream", 0 },
  { "dvi",     0, "application/x-dvi", 0 },
  { "dxr",     0, "application/x-director", 0 },
  { "eps",     0, "application/postscript", 0 },
  { "etx",     0, "text/x-setext", 0 },
  { "exe",     0, "application/octet-stream", 0 },
  { "ez",      0, "application/andrew-inset", 0 },
  { "fgd",     0, "application/x-director", 0 },
  { "fh",      0, "image/x-freehand", 0 },
  { "fh4",     0, "image/x-freehand", 0 },
  { "fh5",     0, "image/x-freehand", 0 },
  { "fh7",     0, "image/x-freehand", 0 },
  { "fhc",     0, "image/x-freehand", 0 },
  { "gif",     0, "image/gif", 0 },
  { "gtar",    0, "application/x-gtar", 0 },
  { "hdf",     0, "application/x-hdf", 0 },
  { "hqx",     0, "application/mac-binhex40", 0 },
  { "htm",     0, "text/html; charset=%s", 0 },
  { "html",    0, "text/html; charset=%s", 0 },
  { "ice",     0, "x-conference/x-cooltalk", 0 },
  { "ief",     0, "image/ief", 0 },
  { "iges",    0, "model/iges", 0 },
  { "igs",     0, "model/iges", 0 },
  { "iv",      0, "application/x-inventor", 0 },
  { "jar",     0, "application/x-java-archive", 0 },
  { "jfif",    0, "image/jpeg", 0 },
  { "jpe",     0, "image/jpeg", 0 },
  { "jpeg",    0, "image/jpeg", 0 },
  { "jpg",     0, "image/jpeg", 0 },
  { "js",      0, "application/x-javascript", 0 },
  { "kar",     0, "audio/midi", 0 },
  { "latex",   0, "application/x-latex", 0 },
  { "lha",     0, "application/octet-stream", 0 },
  { "lzh",     0, "application/octet-stream", 0 },
  { "m3u",     0, "audio/x-mpegurl", 0 },
  { "man",     0, "application/x-troff-man", 0 },
  { "mathml",  0, "application/mathml+xml", 0 },
  { "me",      0, "application/x-troff-me", 0 },
  { "mesh",    0, "model/mesh", 0 },
  { "mid",     0, "audio/midi", 0 },
  { "midi",    0, "audio/midi", 0 },
  { "mif",     0, "application/vnd.mif", 0 },
  { "mime",    0, "message/rfc822", 0 },
  { "mml",     0, "application/mathml+xml", 0 },
  { "mov",     0, "video/quicktime", 0 },
  { "movie",   0, "video/x-sgi-movie", 0 },
  { "mp2",     0, "audio/mpeg", 0 },
  { "mp3",     0, "audio/mpeg", 0 },
  { "mp4",     0, "video/mp4", 0 },
  { "mpe",     0, "video/mpeg", 0 },
  { "mpeg",    0, "video/mpeg", 0 },
  { "mpg",     0, "video/mpeg", 0 },
  { "mpga",    0, "audio/mpeg", 0 },
  { "ms",      0, "application/x-troff-ms", 0 },
  { "msh",     0, "model/mesh", 0 },
  { "mv",      0, "video/x-sgi-movie", 0 },
  { "mxu",     0, "video/vnd.mpegurl", 0 },
  { "nc",      0, "application/x-netcdf", 0 },
  { "o",       0, "application/octet-stream", 0 },
  { "oda",     0, "application/oda", 0 },
  { "ogg",     0, "application/x-ogg", 0 },
  { "pac",     0, "application/x-ns-proxy-autoconfig", 0 },
  { "pbm",     0, "image/x-portable-bitmap", 0 },
  { "pdb",     0, "chemical/x-pdb", 0 },
  { "pdf",     0, "application/pdf", 0 },
  { "pgm",     0, "image/x-portable-graymap", 0 },
  { "pgn",     0, "application/x-chess-pgn", 0 },
  { "png",     0, "image/png", 0 },
  { "pnm",     0, "image/x-portable-anymap", 0 },
  { "ppm",     0, "image/x-portable-pixmap", 0 },
  { "ppt",     0, "application/vnd.ms-powerpoint", 0 },
  { "ps",      0, "application/postscript", 0 },
  { "qt",      0, "video/quicktime", 0 },
  { "ra",      0, "audio/x-realaudio", 0 },
  { "ram",     0, "audio/x-pn-realaudio", 0 },
  { "ras",     0, "image/x-cmu-raster", 0 },
  { "rdf",     0, "application/rdf+xml", 0 },
  { "rgb",     0, "image/x-rgb", 0 },
  { "rm",      0, "audio/x-pn-realaudio", 0 },
  { "roff",    0, "application/x-troff", 0 },
  { "rpm",     0, "audio/x-pn-realaudio-plugin", 0 },
  { "rss",     0, "application/rss+xml", 0 },
  { "rtf",     0, "text/rtf", 0 },
  { "rtx",     0, "text/richtext", 0 },
  { "sgm",     0, "text/sgml", 0 },
  { "sgml",    0, "text/sgml", 0 },
  { "sh",      0, "application/x-sh", 0 },
  { "shar",    0, "application/x-shar", 0 },
  { "silo",    0, "model/mesh", 0 },
  { "sit",     0, "application/x-stuffit", 0 },
  { "skd",     0, "application/x-koan", 0 },
  { "skm",     0, "application/x-koan", 0 },
  { "skp",     0, "application/x-koan", 0 },
  { "skt",     0, "application/x-koan", 0 },
  { "smi",     0, "application/smil", 0 },
  { "smil",    0, "application/smil", 0 },
  { "snd",     0, "audio/basic", 0 },
  { "so",      0, "application/octet-stream", 0 },
  { "spl",     0, "application/x-futuresplash", 0 },
  { "src",     0, "application/x-wais-source", 0 },
  { "stc",     0, "application/vnd.sun.xml.calc.template", 0 },
  { "std",     0, "application/vnd.sun.xml.draw.template", 0 },
  { "sti",     0, "application/vnd.sun.xml.impress.template", 0 },
  { "stw",     0, "application/vnd.sun.xml.writer.template", 0 },
  { "sv4cpio", 0, "application/x-sv4cpio", 0 },
  { "sv4crc",  0, "application/x-sv4crc", 0 },
  { "svg",     0, "image/svg+xml", 0 },
  { "svgz",    0, "image/svg+xml", 0 },
  { "swf",     0, "application/x-shockwave-flash", 0 },
  { "sxc",     0, "application/vnd.sun.xml.calc", 0 },
  { "sxd",     0, "application/vnd.sun.xml.draw", 0 },
  { "sxg",     0, "application/vnd.sun.xml.writer.global", 0 },
  { "sxi",     0, "application/vnd.sun.xml.impress", 0 },
  { "sxm",     0, "application/vnd.sun.xml.math", 0 },
  { "sxw",     0, "application/vnd.sun.xml.writer", 0 },
  { "t",       0, "application/x-troff", 0 },
  { "tar",     0, "application/x-tar", 0 },
  { "tcl",     0, "application/x-tcl", 0 },
  { "tex",     0, "application/x-tex", 0 },
  { "texi",    0, "application/x-texinfo", 0 },
  { "texinfo", 0, "application/x-texinfo", 0 },
  { "tif",     0, "image/tiff", 0 },
  { "tiff",    0, "image/tiff", 0 },
  { "tr",      0, "application/x-troff", 0 },
  { "tsp",     0, "application/dsptype", 0 },
  { "tsv",     0, "text/tab-separated-values", 0 },
  { "txt",     0, "text/plain; charset=%s", 0 },
  { "ustar",   0, "application/x-ustar", 0 },
  { "vcd",     0, "application/x-cdlink", 0 },
  { "vrml",    0, "model/vrml", 0 },
  { "vx",      0, "video/x-rad-screenplay", 0 },
  { "wav",     0, "audio/x-wav", 0 },
  { "wax",     0, "audio/x-ms-wax", 0 },
  { "wbmp",    0, "image/vnd.wap.wbmp", 0 },
  { "wbxml",   0, "application/vnd.wap.wbxml", 0 },
  { "wm",      0, "video/x-ms-wm", 0 },
  { "wma",     0, "audio/x-ms-wma", 0 },
  { "wmd",     0, "application/x-ms-wmd", 0 },
  { "wml",     0, "text/vnd.wap.wml", 0 },
  { "wmlc",    0, "application/vnd.wap.wmlc", 0 },
  { "wmls",    0, "text/vnd.wap.wmlscript", 0 },
  { "wmlsc",   0, "application/vnd.wap.wmlscriptc", 0 },
  { "wmv",     0, "video/x-ms-wmv", 0 },
  { "wmx",     0, "video/x-ms-wmx", 0 },
  { "wmz",     0, "application/x-ms-wmz", 0 },
  { "wrl",     0, "model/vrml", 0 },
  { "wsrc",    0, "application/x-wais-source", 0 },
  { "wvx",     0, "video/x-ms-wvx", 0 },
  { "xbm",     0, "image/x-xbitmap", 0 },
  { "xht",     0, "application/xhtml+xml", 0 },
  { "xhtml",   0, "application/xhtml+xml", 0 },
  { "xls",     0, "application/vnd.ms-excel", 0 },
  { "xml",     0, "text/xml", 0 },
  { "xpm",     0, "image/x-xpixmap", 0 },
  { "xsl",     0, "text/xml", 0 },
  { "xwd",     0, "image/x-xwindowdump", 0 },
  { "xyz",     0, "chemical/x-xyz", 0 },
  { "zip",     0, "application/zip", 0 },
};
static const int n_typ_tab = sizeof(typ_tab) / sizeof(*typ_tab);

#endif /* __NETUTILS_THTTPD_MIME_TYPES_H */

