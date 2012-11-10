/************************************************************************
 * libc/string/lib_strerror.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Types
 ************************************************************************/

struct errno_strmap_s
{
  uint8_t     errnum;
  const char *str;
};

/************************************************************************
 * Private Data
 ************************************************************************/

#ifdef CONFIG_LIBC_STRERROR

/* This table maps all error numbers to descriptive strings.
 * The only assumption that the code makes with regard to this
 * this table is that it is ordered by error number.
 *
 * The size of this table is quite large.  Its size can be
 * reduced by eliminating some of the more obscure error
 * strings.
 */

#ifndef CONFIG_LIBC_STRERROR_SHORT

static const struct errno_strmap_s g_errnomap[] =
{
  { EPERM,               EPERM_STR           },
  { ENOENT,              ENOENT_STR          },
  { ESRCH,               ESRCH_STR           },
  { EINTR,               EINTR_STR           },
  { EIO,                 EIO_STR             },
  { ENXIO,               ENXIO_STR           },
  { E2BIG,               E2BIG_STR           },
  { ENOEXEC,             ENOEXEC_STR         },
  { EBADF,               EBADF_STR           },
  { ECHILD,              ECHILD_STR          },
  { EAGAIN,              EAGAIN_STR          },
  { ENOMEM,              ENOMEM_STR          },
  { EACCES,              EACCES_STR          },
  { EFAULT,              EFAULT_STR          },
  { ENOTBLK,             ENOTBLK_STR         },
  { EBUSY,               EBUSY_STR           },
  { EEXIST,              EEXIST_STR          },
  { EXDEV,               EXDEV_STR           },
  { ENODEV,              ENODEV_STR          },
  { ENOTDIR,             ENOTDIR_STR         },
  { EISDIR,              EISDIR_STR          },
  { EINVAL,              EINVAL_STR          },
  { ENFILE,              ENFILE_STR          },
  { EMFILE,              EMFILE_STR          },
  { ENOTTY,              ENOTTY_STR          },
  { ETXTBSY,             ETXTBSY_STR         },
  { EFBIG,               EFBIG_STR           },
  { ENOSPC,              ENOSPC_STR          },
  { ESPIPE,              ESPIPE_STR          },
  { EROFS,               EROFS_STR           },
  { EMLINK,              EMLINK_STR          },
  { EPIPE,               EPIPE_STR           },
  { EDOM,                EDOM_STR            },
  { ERANGE,              ERANGE_STR          },
  { EDEADLK,             EDEADLK_STR         },
  { ENAMETOOLONG,        ENAMETOOLONG_STR    },
  { ENOLCK,              ENOLCK_STR          },
  { ENOSYS,              ENOSYS_STR          },
  { ENOTEMPTY,           ENOTEMPTY_STR       },
  { ELOOP,               ELOOP_STR           },
  { ENOMSG,              ENOMSG_STR          },
  { EIDRM,               EIDRM_STR           },
  { ECHRNG,              ECHRNG_STR          },
  { EL2NSYNC,            EL2NSYNC_STR        },
  { EL3HLT,              EL3HLT_STR          },
  { EL3RST,              EL3RST_STR          },
  { ELNRNG,              ELNRNG_STR          },
  { EUNATCH,             EUNATCH_STR         },
  { ENOCSI,              ENOCSI_STR          },
  { EL2HLT,              EL2HLT_STR          },
  { EBADE,               EBADE_STR           },
  { EBADR,               EBADR_STR           },
  { EXFULL,              EXFULL_STR          },
  { ENOANO,              ENOANO_STR          },
  { EBADRQC,             EBADRQC_STR         },
  { EBADSLT,             EBADSLT_STR         },
  { EBFONT,              EBFONT_STR          },
  { ENOSTR,              ENOSTR_STR          },
  { ENODATA,             ENODATA_STR         },
  { ETIME,               ETIME_STR           },
  { ENOSR,               ENOSR_STR           },
  { ENONET,              ENONET_STR          },
  { ENOPKG,              ENOPKG_STR          },
  { EREMOTE,             EREMOTE_STR         },
  { ENOLINK,             ENOLINK_STR         },
  { EADV,                EADV_STR            },
  { ESRMNT,              ESRMNT_STR          },
  { ECOMM,               ECOMM_STR           },
  { EPROTO,              EPROTO_STR          },
  { EMULTIHOP,           EMULTIHOP_STR       },
  { EDOTDOT,             EDOTDOT_STR         },
  { EBADMSG,             EBADMSG_STR         },
  { EOVERFLOW,           EOVERFLOW_STR       },
  { ENOTUNIQ,            ENOTUNIQ_STR        },
  { EBADFD,              EBADFD_STR          },
  { EREMCHG,             EREMCHG_STR         },
  { ELIBACC,             ELIBACC_STR         },
  { ELIBBAD,             ELIBBAD_STR         },
  { ELIBSCN,             ELIBSCN_STR         },
  { ELIBMAX,             ELIBMAX_STR         },
  { ELIBEXEC,            ELIBEXEC_STR        },
  { EILSEQ,              EILSEQ_STR          },
  { ERESTART,            ERESTART_STR        },
  { ESTRPIPE,            ESTRPIPE_STR        },
  { EUSERS,              EUSERS_STR          },
  { ENOTSOCK,            ENOTSOCK_STR        },
  { EDESTADDRREQ,        EDESTADDRREQ_STR    },
  { EMSGSIZE,            EMSGSIZE_STR        },
  { EPROTOTYPE,          EPROTOTYPE_STR      },
  { ENOPROTOOPT,         ENOPROTOOPT_STR     },
  { EPROTONOSUPPORT,     EPROTONOSUPPORT_STR },
  { ESOCKTNOSUPPORT,     ESOCKTNOSUPPORT_STR },
  { EOPNOTSUPP,          EOPNOTSUPP_STR      },
  { EPFNOSUPPORT,        EPFNOSUPPORT_STR    },
  { EAFNOSUPPORT,        EAFNOSUPPORT_STR    },
  { EADDRINUSE,          EADDRINUSE_STR      },
  { EADDRNOTAVAIL,       EADDRNOTAVAIL_STR   },
  { ENETDOWN,            ENETDOWN_STR        },
  { ENETUNREACH,         ENETUNREACH_STR     },
  { ENETRESET,           ENETRESET_STR       },
  { ECONNABORTED,        ECONNABORTED_STR    },
  { ECONNRESET,          ECONNRESET_STR      },
  { ENOBUFS,             ENOBUFS_STR         },
  { EISCONN,             EISCONN_STR         },
  { ENOTCONN,            ENOTCONN_STR        },
  { ESHUTDOWN,           ESHUTDOWN_STR       },
  { ETOOMANYREFS,        ETOOMANYREFS_STR    },
  { ETIMEDOUT,           ETIMEDOUT_STR       },
  { ECONNREFUSED,        ECONNREFUSED_STR    },
  { EHOSTDOWN,           EHOSTDOWN_STR       },
  { EHOSTUNREACH,        EHOSTUNREACH_STR    },
  { EALREADY,            EALREADY_STR        },
  { EINPROGRESS,         EINPROGRESS_STR     },
  { ESTALE,              ESTALE_STR          },
  { EUCLEAN,             EUCLEAN_STR         },
  { ENOTNAM,             ENOTNAM_STR         },
  { ENAVAIL,             ENAVAIL_STR         },
  { EISNAM,              EISNAM_STR          },
  { EREMOTEIO,           EREMOTEIO_STR       },
  { EDQUOT,              EDQUOT_STR          },
  { ENOMEDIUM,           ENOMEDIUM_STR       },
  { EMEDIUMTYPE,         EMEDIUMTYPE_STR     }
};

#else /* CONFIG_LIBC_STRERROR_SHORT */

static const struct errno_strmap_s g_errnomap[] =
{
  { EPERM,               "EPERM"             },
  { ENOENT,              "ENOENT"            },
  { ESRCH,               "ESRCH"             },
  { EINTR,               "EINTR"             },
  { EIO,                 "EIO"               },
  { ENXIO,               "ENXIO"             },
  { E2BIG,               "E2BIG"             },
  { ENOEXEC,             "ENOEXEC"           },
  { EBADF,               "EBADF"             },
  { ECHILD,              "ECHILD"            },
  { EAGAIN,              "EAGAIN"            },
  { ENOMEM,              "ENOMEM"            },
  { EACCES,              "EACCES"            },
  { EFAULT,              "EFAULT"            },
  { ENOTBLK,             "ENOTBLK"           },
  { EBUSY,               "EBUSY"             },
  { EEXIST,              "EEXIST"            },
  { EXDEV,               "EXDEV"             },
  { ENODEV,              "ENODEV"            },
  { ENOTDIR,             "ENOTDIR"           },
  { EISDIR,              "EISDIR"            },
  { EINVAL,              "EINVAL"            },
  { ENFILE,              "ENFILE"            },
  { EMFILE,              "EMFILE"            },
  { ENOTTY,              "ENOTTY"            },
  { ETXTBSY,             "ETXTBSY"           },
  { EFBIG,               "EFBIG"             },
  { ENOSPC,              "ENOSPC"            },
  { ESPIPE,              "ESPIPE"            },
  { EROFS,               "EROFS"             },
  { EMLINK,              "EMLINK"            },
  { EPIPE,               "EPIPE"             },
  { EDOM,                "EDOM"              },
  { ERANGE,              "ERANGE"            },
  { EDEADLK,             "EDEADLK"           },
  { ENAMETOOLONG,        "ENAMETOOLONG"      },
  { ENOLCK,              "ENOLCK"            },
  { ENOSYS,              "ENOSYS"            },
  { ENOTEMPTY,           "ENOTEMPTY"         },
  { ELOOP,               "ELOOP"             },
  { ENOMSG,              "ENOMSG"            },
  { EIDRM,               "EIDRM"             },
  { ECHRNG,              "ECHRNG"            },
  { EL2NSYNC,            "EL2NSYNC"          },
  { EL3HLT,              "EL3HLT"            },
  { EL3RST,              "EL3RST"            },
  { EL3RST,              "EL3RST"            },
  { EUNATCH,             "EUNATCH"           },
  { ENOCSI,              "ENOCSI"            },
  { EL2HLT,              "EL2HLT"            },
  { EBADE,               "EBADE"             },
  { EBADR,               "EBADR"             },
  { EXFULL,              "EXFULL"            },
  { ENOANO,              "ENOANO"            },
  { EBADRQC,             "EBADRQC"           },
  { EBADSLT,             "EBADSLT"           },
  { EBFONT,              "EBFONT"            },
  { ENOSTR,              "ENOSTR"            },
  { ENODATA,             "ENODATA"           },
  { ETIME,               "ETIME"             },
  { ENOSR,               "ENOSR"             },
  { ENONET,              "ENONET"            },
  { ENOPKG,              "ENOPKG"            },
  { EREMOTE,             "EREMOTE"           },
  { ENOLINK,             "ENOLINK"           },
  { EADV,                "EADV"              },
  { ESRMNT,              "ESRMNT"            },
  { ECOMM,               "ECOMM"             },
  { EPROTO,              "EPROTO"            },
  { EMULTIHOP,           "EMULTIHOP"         },
  { EDOTDOT,             "EDOTDOT"           },
  { EBADMSG,             "EBADMSG"           },
  { EOVERFLOW,           "EOVERFLOW"         },
  { ENOTUNIQ,            "ENOTUNIQ"          },
  { EBADFD,              "EBADFD"            },
  { EREMCHG,             "EREMCHG"           },
  { ELIBACC,             "ELIBACC"           },
  { ELIBBAD,             "ELIBBAD"           },
  { ELIBSCN,             "ELIBSCN"           },
  { ELIBMAX,             "ELIBMAX"           },
  { ELIBEXEC,            "ELIBEXEC"          },
  { EILSEQ,              "EILSEQ"            },
  { ERESTART,            "ERESTART"          },
  { ESTRPIPE,            "ESTRPIPE"          },
  { EUSERS,              "EUSERS"            },
  { ENOTSOCK,            "ENOTSOCK"          },
  { EDESTADDRREQ,        "EDESTADDRREQ"      },
  { EMSGSIZE,            "EMSGSIZE"          },
  { EPROTOTYPE,          "EPROTOTYPE"        },
  { ENOPROTOOPT,         "ENOPROTOOPT"       },
  { EPROTONOSUPPORT,     "EPROTONOSUPPORT"   },
  { ESOCKTNOSUPPORT,     "ESOCKTNOSUPPORT"   },
  { EOPNOTSUPP,          "EOPNOTSUPP"        },
  { EPFNOSUPPORT,        "EPFNOSUPPORT"      },
  { EAFNOSUPPORT,        "EAFNOSUPPORT"      },
  { EADDRINUSE,          "EADDRINUSE"        },
  { EADDRNOTAVAIL,       "EADDRNOTAVAIL"     },
  { ENETDOWN,            "ENETDOWN"          },
  { ENETUNREACH,         "ENETUNREACH"       },
  { ENETRESET,           "ENETRESET"         },
  { ECONNABORTED,        "ECONNABORTED"      },
  { ECONNRESET,          "ECONNRESET"        },
  { ENOBUFS,             "ENOBUFS"           },
  { EISCONN,             "EISCONN"           },
  { ENOTCONN,            "ENOTCONN"          },
  { ESHUTDOWN,           "ESHUTDOWN"         },
  { ETOOMANYREFS,        "ETOOMANYREFS"      },
  { ETIMEDOUT,           "ETIMEDOUT"         },
  { ECONNREFUSED,        "ECONNREFUSED"      },
  { EHOSTDOWN,           "EHOSTDOWN"         },
  { EHOSTUNREACH,        "EHOSTUNREACH"      },
  { EALREADY,            "EALREADY"          },
  { EINPROGRESS,         "EINPROGRESS"       },
  { ESTALE,              "ESTALE"            },
  { EUCLEAN,             "EUCLEAN"           },
  { ENOTNAM,             "ENOTNAM"           },
  { ENAVAIL,             "ENAVAIL"           },
  { EISNAM,              "EISNAM"            },
  { EREMOTEIO,           "EREMOTEIO"         },
  { EDQUOT,              "EDQUOT"            },
  { ENOMEDIUM,           "ENOMEDIUM"         },
  { EMEDIUMTYPE,         "EMEDIUMTYPE"     }
};

#endif /* CONFIG_LIBC_STRERROR_SHORT */

#define NERRNO_STRS (sizeof(g_errnomap) / sizeof(struct errno_strmap_s))

#endif /* CONFIG_LIBC_STRERROR */

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: strerror
 ************************************************************************/

FAR const char *strerror(int errnum)
{
#ifdef CONFIG_LIBC_STRERROR
  int ndxlow = 0;
  int ndxhi  = NERRNO_STRS - 1;
  int ndxmid;

  do
    {
      ndxmid = (ndxlow + ndxhi) >> 1;
      if (errnum > g_errnomap[ndxmid].errnum)
        {
          ndxlow = ndxmid + 1;
        }
      else if (errnum < g_errnomap[ndxmid].errnum)
        {
          ndxhi = ndxmid - 1;
        }
      else
        {
          return g_errnomap[ndxmid].str;
        }
    }
  while (ndxlow <= ndxhi);
#endif
  return "Unknown error";
}
