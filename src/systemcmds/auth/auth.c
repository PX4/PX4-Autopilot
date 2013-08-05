/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *      OTP, Flash, Skeleton 
 *   Author: @author David "Buzz" Bussenschutt <davidbuzz@gmail.com>
 *      Encryption, Signing/Verify, SD reading/writing, parameters, keys, etc. 
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

/**
 * @file auth.c 
 * Tool similar to UNIX reboot command
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>

//#include <apps/netutils/base64.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <libtomcrypt/tomcrypt_custom.h>
#include <libtomcrypt/tomcrypt.h>
#include <libtomcrypt/tomcrypt_misc.h>
#include <libtomfastmath/tfm.h>

/* RM page 75 OTP. Size is 528 bytes total (512 bytes data and 16 bytes locking) */
#define ADDR_OTP_START			0x1FFF7800
#define ADDR_OTP_LOCK_START		0x1FFF7A00

#define OTP_LEN				512
#define OTP_LOCK_LEN			16

#define OTP_LOCK_LOCKED			0x00
#define OTP_LOCK_UNLOCKED		0xFF

__EXPORT int auth_main(int argc, char *argv[]);

#pragma pack(push, 1)

/*
 * The OTP area is divided into 16 OTP data blocks of 32 bytes and one lock OTP block of 16 bytes.
 * The OTP data and lock blocks cannot be erased. The lock block contains 16 bytes LOCKBi (0 ≤ i ≤ 15)
 * to lock the corresponding OTP data block (blocks 0 to 15). Each OTP data block can be programmed
 * until the value 0x00 is programmed in the corresponding OTP lock byte. The lock bytes must only
 * contain 0x00 and 0xFF values, otherwise the OTP bytes might not be taken into account correctly.
 */

	struct otp {
	    // first 32 bytes =  the '0' Block 
		char		id[4];		///4 bytes < 'P' 'X' '4' '\n'
		uint8_t		id_type;	///1 byte < 0 for USB VID, 1 for generic VID
		uint32_t	vid;        ///4 bytes
		uint32_t	pid;        ///4 bytes
		char        unused[19];  ///3 bytes 
		// Cert-of-Auth is next 4 blocks ie 1-4  ( where zero is first block ) 	
		char        signature[128];
        // insert extras here 
		uint32_t	lock_bytes[4];
	};

	struct otp_lock {
		uint8_t		lock_bytes[16];
	};
#pragma pack(pop)

#define UDID_START		0x1FFF7A10
#define ADDR_FLASH_SIZE		0x1FFF7A22

#pragma pack(push, 1)
		union udid {
		uint32_t	serial[3];
		char  data[12];
	};
#pragma pack(pop)

int flash_unlock(void)
{
	/* unlock the control register */
	volatile uint32_t *keyr = (volatile uint32_t *)0x40023c04;
	*keyr = 0x45670123U;
	*keyr = 0xcdef89abU;

	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;

	/* check the control register */
	if (*cr & 0x80000000) {
		warnx("WARNING: flash unlock failed, flash aborted");
		return 1;
	}

	return 0;
}

int flash_lock(void)
{
	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;
	/* re-lock the flash control register */
	*cr = 0x80000000;
}

int val_read(void* dest, volatile const void* src, int bytes)
{
	
	int i;
	for (i = 0; i < bytes / 4; i++) {
		*(((volatile uint32_t *)dest) + i) = *(((volatile uint32_t *)src) + i);
	}
	return i*4;
}

int val_write(volatile void* dest, const void* src, int bytes)
{
	flash_unlock();

	int i;
	

	volatile uint32_t *sr = (volatile uint32_t *)0x40023c0c;
	volatile uint32_t *cr = (volatile uint32_t *)0x40023c10;


	volatile uint32_t *d = (volatile uint32_t *)dest;
	volatile uint32_t *s = (volatile uint32_t *)src;

	for (i = 0; i < bytes / 4; i++) {
		/* program a byte */
		*cr = 1;
		
		warnx("%016x",s[i]);

		d[i] = s[i];

		/* wait for the operation to complete */
		while (*sr & 0x1000) {
		}

		if (*sr & 0xf2) {
			warnx("WARNING: program error 0x%02x after %d bytes", *sr, i*4);
			goto flash_end;
		}
	}


flash_end:
	flash_lock();

	return i*4;
}

int write_otp(uint8_t id_type, uint32_t vid, uint32_t pid, char* signature)
{
	struct otp otp_mem;
	memset(&otp_mem, 0, sizeof(otp_mem));

	/* fill struct */
	otp_mem.id[0] = 'P';
	otp_mem.id[1] = 'X';
	otp_mem.id[2] = '4';
	otp_mem.id[3] = '\0';
	
	memcpy(otp_mem.signature,signature,128); 
	
	warnx("write_otp: %s  / %s ",signature, otp_mem.signature); 

	volatile uint32_t* otp_ptr = ADDR_OTP_START;
	val_write(otp_ptr, &otp_mem, sizeof(struct otp));
}

int lock_otp()
{
	/* determine the required locking size - can only write full lock bytes */
	int size = sizeof(struct otp) / 32;

	struct otp_lock otp_lock_mem;

	memset(&otp_lock_mem, OTP_LOCK_UNLOCKED, sizeof(otp_lock_mem));
	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
		otp_lock_mem.lock_bytes[i] = OTP_LOCK_LOCKED;

	/* XXX add the actual call here to write the OTP_LOCK bytes only at final stage */
	// val_copy(lock_ptr, &otp_lock_mem, sizeof(otp_lock_mem));
}


#define SERIAL_LEN 27
#define BIN_SERIAL_LEN 20  // or 24 
#define PADDING_TYPE LTC_LTC_PKCS_1_V1_5 // or LTC_LTC_PKCS_1_PSS
#define SALTLEN 0 // or 8, or 16 


void sign_serial_and_return_cert( rsa_key * private_key , prng_state * prng, char * serialid,  char * newcert, int * certlen) { 
    	// locate indexes into hash list and prng list  
	int hash_idx, prng_idx, err;
    prng_idx = find_prng("yarrow");
    hash_idx = find_hash("sha1"); 

	  if (hash_idx == -1 || prng_idx == -1) {
          warnx( "sign requires LTC_SHA1(%d) and yarrow(%d)",hash_idx,prng_idx );
          return 1;
     }

    
    *certlen = 1024; // sizeof() no worky on undef pointers like the empty cert[] we passed in. 
    
    if ((err =  rsa_sign_hash_ex(
        serialid,
        (unsigned long) BIN_SERIAL_LEN,
        newcert,
        certlen,
        PADDING_TYPE, // LTC_LTC_PKCS_1_V1_5, // V1.5 is older padding type, PSS is newer : LTC_LTC_PKCS_1_PSS, // padding type 
        prng,
        prng_idx,
        hash_idx,
        SALTLEN,   // saltlen 
        private_key)) != CRYPT_OK) {
            warnx("rsa_sign_hash_ex %s\n", error_to_string(err));
        } else { 
             warnx("\t rsa_sign_hash_ex OK. length: %d", (int)(*certlen)); 
        }       
        
    
} 

int  verify_cert_and_return(rsa_key * public_key, prng_state * prng, char * cert  , int certlen, char * oldserialid ) { 
    	// locate indexes into hash list and prng list  
	int hash_idx, prng_idx;
    prng_idx = find_prng("yarrow");
    hash_idx = find_hash("sha1"); 
    
      if (hash_idx == -1 || prng_idx == -1) {
          warnx( "verify requires LTC_SHA1(%d) and yarrow(%d)",hash_idx,prng_idx );
          return 1;
     }

    
   int retval, err; // serial_len;
   
  // warnx("%s:%d:%s:%d\n",cert, certlen, oldserial, BIN_SERIAL_LEN ); 
   
   if ((err =   rsa_verify_hash_ex(
        cert,
        certlen,
        oldserialid,
        BIN_SERIAL_LEN, // serial length / 
        PADDING_TYPE,
        hash_idx,
        SALTLEN, // saltlen /
        &retval,
        public_key)) != CRYPT_OK) {
            warnx("rsa_verify_hash %s\n", error_to_string(err));
            retval = 0; // bad. 
        } else { 
             warnx("\t rsa_verify_hash OK. length: %d retval: %d\n", BIN_SERIAL_LEN, retval); 
        }         

  fflush(stdout);

    return retval; //   1 is GOOD.   0 is BAD. 
    
} 


int read_key_from_sd(char * filename , char * blob ) { 
    
  //  warnx("reading some key from SD card..."); 
  FILE *fp;

  if ((fp = fopen(filename, "rb")) == NULL) //read from file
    {
      warnx("ERROR! Unable to read private/public file \"%s\" from SD card, sorry.",filename); 
      return "";
    }

  int filelen = 0;
  //while ( !feof(fp) ) { 
    filelen += fread( blob, 1, 1024 , fp ) ; // try to read 1024 bytes into "blob", private key is never that long. 
  //  warnx("read: %s",blob);
  //}
  //filelen = actual number of bytes read from file.
  fclose(fp);
 
  //add end-of-string marker to blob, needed for "warn" statement to work below.
  blob[filelen] = 0; 
  
  // OPTIONAL TODO remove "-----BEGIN RSA PRIVATE KEY-----\n" and "-----END RSA PRIVATE KEY-----\n" from a file if its there.
  // for the moment, we assume the user has correctly formatted their private key file for this system by removing those strings. 
  
 // warnx("sd len: %d  data: %s\n",filelen, blob ) ; 
 warnx("\t reading file '%s' from SD done.", filename);
  
  return filelen; 
} 

void append_serial_data_to_sd_log( char * filename, char * serial, char * cert, char * private ) { 
    
    warnx("appending to file: %s on SD card.",filename); 
static const char *log_format_string ="serial=%s cert=%s privatekey=%s\n";

  FILE *fp;

  if ((fp = fopen(filename, "a")) == NULL) //apend to file
    {
      return -1;
    }

  fprintf(fp, log_format_string, serial, cert, private);

  fflush(fp);
  fclose(fp);

  return 0;
    
} 

write_whole_file_to_sd(char * filename, char * publickeydata, int datalen){ 

  FILE *fp;

 warnx("writing file: '%s' to SD done.",filename);
 
 // static const char *format_string ="%s";

  if ((fp = fopen(filename, "wb")) == NULL) //apend to file
    {
      return -1;
    }

//  fprintf(fp, format_string, publickeydata); // not binary safe

    fwrite(publickeydata , 1 , datalen , fp ); // binary safe


  fflush(fp);
  fclose(fp);

  return 0;

} 

// BIN_SERIAL_LEN in, SERIAL_LEN out. 
// serialid in, serial out. 
void human_readable_serial(char *serialid, char * serial ){ 
  
  	sprintf(serial, "%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
	   serialid[0],serialid[1],serialid[2],serialid[3],serialid[4],serialid[5],serialid[6],serialid[7],serialid[8],serialid[9],serialid[10],serialid[11]); 
    
} 


// pass in the private key, and we'll return the compatible formatted public key that libtomcrypt can embed
void  make_public_key_from_private(rsa_key * key, char * outbuffer) { 
    
   
	     unsigned char OUTbuffer[255];	 
         unsigned char OUTPEMbuffer[255];	 
         unsigned long outlen2;
         unsigned long outlenpem2;
         int err; 
          outlen2 = sizeof(OUTbuffer);
          
                    
         //PUBLIC
         // PK_PUBLIC means its just a public key component. 
         if ( (err = rsa_export(OUTbuffer,&outlen2, PK_PUBLIC, key)) != CRYPT_OK ) {
              warnx("Error with rsa_export: %s\n", error_to_string(err));
          } else { 
      	   warnx("PUBLIC rsa_export OK: size: %d\n", outlen2 );//
      	 }
         // convert what is essentially DER data into naked PEM format. ( ie no header or footer lines )  
          if ( (err = base64_encode(OUTbuffer,outlen2,OUTPEMbuffer, &outlenpem2)) != CRYPT_OK ) {
              warnx("base64_encode error: %s\n", error_to_string(err));
          }  else { 
          
          // this should print our public key, human readable:      
          warnx("base64 PUBLIC KEY.  len: %d key: %s\n", outlenpem2, OUTPEMbuffer );
          } 
          
          strcpy(outbuffer,OUTPEMbuffer); // return the good bit. 

}

// private or public, either is good. 
void load_key( rsa_key * key, char * PEMbuffer ) { 
    
    
	 //warnx(" load key start --------------------\n");// %s\n",PEMbuffer);
	   
	   int err; 
	       
   
        unsigned long elen;
        unsigned long *eoutlen;
        // well convert it to DER format and store it here, in a sec. 
        unsigned char DERbuffer[1024];

            
        elen = strlen(PEMbuffer);// actual in len
        eoutlen = sizeof(DERbuffer); // max out len

        
        // convert what is essentially PEM data into DER format so we can decrypt it. 
        if ( (err = base64_decode(PEMbuffer,elen,DERbuffer, &eoutlen)) != CRYPT_OK ) {
            warnx("\t base64_decode error: %s\n", error_to_string(err));
        }  else { 
    //warnx("\t base64_decode OK actualoutlen: %d\n",eoutlen);
      //  warnx("PUBLIC base64_decode OK \n");
        }   
        
       	    // read OpenSSL DER formatted key into Tom's format. :-)
        if ( (err = rsa_import( &DERbuffer,eoutlen,key)) != CRYPT_OK ) {
            warnx("\t Error with rsa_import : %s\n", error_to_string(err));
        } else { 
    	   warnx("\t rsa_import OK, length: %d",eoutlen);//
    	 }

    //warnx(" load key end --------------------.\n");

	// }

} 


static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);
	errx(1, 
		"usage:\n"
		"auth [-x] [-p] [-d] [-w] [-k] [-v] [-l] \n"
		"  -x     Read Private Key from /mnt/microsd/privatekey.txt and display it\n"
		"  -p     Read Public Key from  /mnt/microsd/publickey.txt and display it\n"
		"  -c     make a Certificate-Of-Authenticity with the private key and display it ( assumes also -x ) \n"
		"  -w     write the Certificate-Of-Authenticity to Flash ( assumes also -x and -c )  \n"
		"  -k     LOCK the Certificate-Of-Authenticity to ONE-TIME-PROGRAMMABLE Flash PERMANENTLY ( assumes -w -x -c ) \n"
		"  -v     Verify the Certificate-Of-Authenticity in OTP with the public key, and display results.  ( assumes -p ) \n"
		"  -t     test cert-of-auth just generated without OTP ( equivalent to -x -p -c -v ) \n"
		"  -l     log it.  Append the COA and privatekey info to the SD card at /fs/microsd/OTPCertificates.log \n"
		"  -h     use Hardcoded Private/Public Keys, no SD card needed\n"
		"  -s     use hardcoded serial: '33002E 32314704 34303736'  \n"
		);

}


int auth_main(int argc, char *argv[])
{

     warnx("AUTH started.\n");
     
     flash_unlock(); 
     
          warnx("AUTH started.2\n");

  
    int ch, err; 

	if (argc < 2)
		usage(NULL);
		

    bool readprivate = false; 
    bool readpublic = false; 
    bool makecert = false;
    bool writecert = false;
    bool lockcert  = false;
    bool verifycert = false;
    bool testCOA = false;
    bool log = false;
    bool hardcoded = false;
    bool hardcodedserial = false;

	while ((ch = getopt(argc, argv, "xpcwlvths")) != EOF) {
		switch (ch) {
		case 'x': // load private 
		    readprivate = true;
			break;
		case 'p': // load public
		      readpublic = true;
			break;
		case 'c': // generate cert-of-auth,  assumes -x 
		      makecert = true;
		      // also
		      readprivate = true;
			break;
		case 'w': // write cert-of-auth to flash, assumes -x and -c 
		      writecert = true;
		      //also
		      readprivate = true; makecert = true;
			break;
		case 'k': // lock flash, assumes -w -x -c
		      lockcert = true;
		      //also
		      readprivate = true;makecert = true;writecert = true;
			break;
		case 'v': //verify cert-of-auth, assumes -p
		      verifycert = true;
		      // also 
		       readpublic = true;
			break;
		case 't': //verify/test cert-of-auth just generated without OTP ( equivalent to -x -p -c -v )
		      testCOA = true;
		      //also
		      readprivate = true; makecert = true; readpublic = true;verifycert = true;
			break;
		case 'l': //log it.
		      log = true;
			break;
		case 'h': //Hardcoded keys
		      hardcoded = true;
			break;
		case 's': //Hardcoded serial
		      hardcodedserial = true;
		      // precaution to avoid bad OTP
		      writecert = false;
		      lockcert = false;
			break;
		default:
			usage(NULL);
		}
	}
	argc -= optind;
	argv += optind;
  
	
	 // ONE-TIME-PROGRAMMABLE (OTP) MEMORY HANDLING SECTION
	

	//disable scheduling, leave interrupt processing untouched 
	sched_lock();


	
	// read out unique chip ID /
	const volatile uint32_t* udid_ptr = (const uint32_t*)UDID_START;
	union udid id;
	val_read(&id, udid_ptr, sizeof(id));

    // raw from OTP.
	//warnx("Unique serial # [%08X %08X %08X] size: %d", id.serial[0], id.serial[1], id.serial[2], sizeof(id));
	
 	unsigned char serialid[BIN_SERIAL_LEN]; //binary version of serial, padded with zero bytes. 
 	
 	
 	//  this copies the data from the id "union" to the serialid char[] array, but on hthe px4, it's the wrong endian order.
    //memcpy(serialid,id.data,12); // first 12 bytes are from serial 
	//      
	   // this is the right order for the px4: , there's a neater way to do this, but it'll do for now. 
	serialid[0] = id.data[3]; 	serialid[1] = id.data[2];	serialid[2] = id.data[1];	serialid[3] = id.data[0];
	serialid[4] = id.data[7]; 	serialid[5] = id.data[6];	serialid[6] = id.data[5];	serialid[7] = id.data[4];
	serialid[8] = id.data[11]; 	serialid[9] = id.data[10];	serialid[10] = id.data[9];	serialid[11] = id.data[8];


 
    // allow to override the serial number 	
    if ( hardcodedserial ) { 
        
        // compound literal trick to assign a whole array at once... // michael! 
        memcpy(serialid, (const char[]){0x00,0x33,0x00,0x2E,0x32,0x31,0x47,0x04,0x34,0x30,0x37,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, sizeof serialid);
        
       //   0x00, 0x3D, 0x00, 0x1F, 0x32, 0x31, 0x47, 0x0C, 0x34, 0x30, 0x37, 0x36    // buzz
       //   0x00, 0x33, 0x00, 0x2E, 0x32, 0x31, 0x47, 0x04, 0x34, 0x30, 0x37, 0x36    // michael 
    }

    // zero buffer to 20 bytes. 
    serialid[12] = 0;  serialid[13] = 0;  serialid[14] = 0;  serialid[15] = 0;   
    serialid[16] = 0;  serialid[17] = 0;  serialid[18] = 0;  serialid[19] = 0; 
    
 	   	
	unsigned char serial[SERIAL_LEN]; // human-readable version is 27 bytes  ( 3x8 + two whitespace +null) 
	
	human_readable_serial( &serialid, &serial ) ;  // from serialid, to serial
	
	if ( hardcodedserial ) { 
        warnx("WARNING !!!! FORCING hardcoded serial: '%s' - TESTING ONLY\n", serial); 
    }
    
    //TIP:    FROM THIS POINT FORWARD, 'serial' is the human-readable version, and  'serialid' is the binary version.
    // the human-readalbe version is used to display to teh screen, and in logs... while the binary version is used in encryption functuions and on SD card in .txt files etc 

 
    // tell the user; 
    warnx("Unique serial # [%s]\n", serial); 

       
    rsa_key private_key;
    rsa_key public_key;
    
    prng_state prng;
   
            // / register the yarrow RNG /
                if (register_prng(&yarrow_desc) == -1) {
                    warnx("Error registering Yarrow: register_prng \n");
                } else { 
            //	   warnx("register_prng(&yarrow_desc) OK\n");
            	 }
            
            /* start it */
            if ((err = yarrow_start(&prng)) != CRYPT_OK) {
            warnx("Start error: %s\n", error_to_string(err));
            }
            /* add entropy */
            if ((err = yarrow_add_entropy("hello world", 11, &prng))
            != CRYPT_OK) {
            warnx("Add_entropy error: %s\n", error_to_string(err));
            }
            /* ready and read */
            if ((err = yarrow_ready(&prng)) != CRYPT_OK) {
            warnx("Ready error: %s\n", error_to_string(err));
            }
            
            //warnx("Read %lu bytes from yarrow\n", yarrow_read(buf, sizeof(buf), &yarrow_prng));
        
           if (register_hash(&sha1_desc) == -1) {  warnx("Critical Error registering sha1 hash");   }
 

    // register a math library (in this case TomsFastMath)
    // this is important, don't forget it. 
    ltc_mp = tfm_desc;
    
    // private key
    // note that tabs, backslash and tabs and newlines should be ignored by this particular impl. 
    
    // EITHER hard-code a private key like this: 
    char privkeydata[1024] = ""; 
    if ( hardcoded && readprivate ) { 
        strcpy(privkeydata, "MIICXgIBAAKBgQDQKDPZdeOCSID6jSYiP8X2mrCRYHNwbp8zHm7um/sqo2xkWnI1\n\
ITvKjhXypFxQ/oe1g1yebgJJY0AS+OpeZy3RHsDRgPLonVzql0c6YZdlreycORLu\n\
JK2XFkUOSVIXwan7IiEI1DMbJ0doQ5oOAia1qVKPXxzpADwWdjV0y0756wIDAQAB\n\
AoGBAMISGXVP8lPPoWD4JGueJcWrp5+C214h5Q/V+ftBNkUkpLRTl1Ntrr9FBbV6\n\
BBAHnyNeXAXh7wPZIy4NIQXvEMjPDk8krOWoX1QShjqk8Xl7BoZlshKRU84/7Gr3\n\
nd2dZ5JRiVv4yLcohZ0MlSIASW/0t8T93AsJmDaUxVbRAghhAkEA8YAumnlwmaSl\n\
sSfKawsMEB9GFZ3JxQ59URheX5b5OPbd+j7lurbCCY6Q99GdIjG7pFvl8+AOFR6V\n\
ottYCHSX6QJBANyniO/K0J8ndDW1mamf45170L7TuDkEEPKjeGHjc+PSLXdFfFcD\n\
VSW/DUz2ta5HE8kV03w1yZun/2r8jKIrcrMCQQDkHA5pDNIl3hY/qnUQ/ONNCy04\n\
18yw3EnUYq8pnUIU42GysNxvq5bGTipyWkUQ+mbiDYe7/mNu4W+333VcrzyZAkA/\n\
7vcZa62I/9iHG2g7os1DuzVfpV7SfmAevcjKrCnPD/4Gegat+5Q3TKUg8LbxmTyd\n\
XgqaCcexpzq1mBlzf51LAkEAjaYN/u8C10f93s+hvdqS3BS8PtxlycAHtDXSMoiO\n\
tDFFYr41gHNDt7loUH1tufL4BUZy5R+9MT7ChaDvX8MLzQ==");  //"
        load_key( &private_key, privkeydata) ;
        warnx("HARDCODED PRIVATE KEY LOADED: \n%s\n",privkeydata);
        write_whole_file_to_sd("/fs/microsd/privatekey.txt",privkeydata,strlen(privkeydata) );  // convenient way to get it on SD.          
    } 
    if ( !hardcoded && readprivate ) {
        // OR read a private key from SD card like this: 
        read_key_from_sd("/fs/microsd/privatekey.txt", &privkeydata);   
        load_key( &private_key, privkeydata );  
        warnx("SD PRIVATE KEY LOADED.\n"); //,privkeydata);
    }
 
    
    
    char cert[1024];  // this is where the binary CERT-OF-AUTH goes.
    int certlen; 
    char out[255];  // THIS IS WHERE THE human-readable CERT-OF-AUTH goes….
    int outlen = 128; // basic cert len.
    if ( makecert ) { 
         warnx("\t signing [binary] serial data with private key...");
         sign_serial_and_return_cert( &private_key, &prng, serialid, &cert, &certlen);    
          
         // give us the human-readable version too….  
         warnx("\t cert len:%d rawbytes: %02x %02x %02x %02x %02x ...(etc)", certlen, cert[0],cert[1],cert[2],cert[3],cert[4]);
         outlen = 1024; // maxlen
         base64_encode(cert, certlen, &out, &outlen);
         warnx("\nCERT-OF-AUTHENTICITY: for serial: %s \nCERT-OF-AUTHENTICITY: humanreadable: %s\nCERT-OF-AUTHENTICITY: rawsize: %d  humanlen: %d \n", serial, out,certlen, outlen);
    } 
    
    // simple test suite for SD card read/write with both binary and ascii files.
    if ( false ) { 
        write_whole_file_to_sd("/fs/microsd/test1.txt",cert,certlen ); 
        char cert2[1024];  // this is where the binary CERT-OF-AUTH goes.
        int certlen2;
        certlen2 = read_key_from_sd("/fs/microsd/test1.txt", &cert2);   
        warnx("SD test 1:  certlen: %d certlen2: %d cert: %s  cert2:%s\n",certlen, certlen2, cert, cert2);
        
        write_whole_file_to_sd("/fs/microsd/test2.txt",out,outlen ); 
        char out2[255];  // this is where the text CERT-OF-AUTH goes.
        int outlen2;
        outlen2 = read_key_from_sd("/fs/microsd/test2.txt", &out2);   
        warnx("SD test 2:  outlen: %d outlen2: %d out: %s  out2:%s\n",outlen, outlen2, out, out2);
    }
    
    // mess with cert:  should cause a fail when this is uncommented. 
    //cert[12] = 1;
    
    // SAMPLE method on how to "test" any given COA agains any given Serial, such as these two: 
   // OK, which cert do we verify.., the one we just generated, or the one from OTP? 
     if ( ! testCOA ) { 
        warnx("TODO: reading from OTP is not yet impl. Faking it with cached COA data from SD card... \n");
 
      // read from SD instead of OTP, to fake it, run 'auth -c' to create the files before using them.  
      outlen = read_key_from_sd("/fs/microsd/COA.b64", &out);    
      certlen = read_key_from_sd("/fs/microsd/COA.bin", &cert);   
      //if ( hardcodedserial ) {   
      //  read_key_from_sd("/fs/microsd/COA.sid", &serialid);  // binary   
      //  read_key_from_sd("/fs/microsd/COA.ser", &serial);    // ascii readable
      //}
        warnx("SD CERT-OF-AUTHENTICITY for serial?: %s rawsize: %d  humanlen: %d , humanreadable:%s\n", serial, certlen, outlen, out);
        
      // then we continue on below to read the public key, and report back on the verification.... 

    }  else { 
        
         warnx("TESTING: due to -t, we are testing the generated cert-of-auth ( using -c ) , not one in OTP.... continuing.... \n"); 
    
        if ( log ) { 
          warnx("LOGGING: Logging COA data to SD card ... with certlen: %d\n",certlen); 
          write_whole_file_to_sd("/fs/microsd/COA.bin",cert,certlen ); 
          write_whole_file_to_sd("/fs/microsd/COA.b64",out,outlen ); 
          write_whole_file_to_sd("/fs/microsd/COA.ser",serial,SERIAL_LEN ); 
          write_whole_file_to_sd("/fs/microsd/COA.sid",serialid,BIN_SERIAL_LEN ); 
        } else { 
          warnx("TESTING: To write this COA data to SD card, also specify -l\n"); 
        }
    }
 
     // need public key to do the "verify" ... THREE ways to get it:  
      char pubkeydata[255] = "";
      if ( 0 ) { 
        // TODO make this test for the presence of the publickey.txt, and if not there, make it.  :-) 
          // this way to start with, we use the private key ( loaded above) to make the public key, and put it on the SD card in a file.
          make_public_key_from_private(&private_key, &pubkeydata); 
          load_key( &public_key, pubkeydata ); 
          write_whole_file_to_sd("/fs/microsd/publickey.txt",pubkeydata,strlen(pubkeydata) ); 
     } 
     if ( hardcoded && readpublic ) {
       // OR we load it hard-coded like this: 
       strcpy(pubkeydata, "MIGJAoGBANAoM9l144JIgPqNJiI/xfaasJFgc3BunzMebu6b+yqjbGRacjUhO8qOF\n\
fKkXFD+h7WDXJ5uAkljQBL46l5nLdEewNGA8uidXOqXRzphl2Wt7Jw5Eu4krZcWRQ5\n\
JUhfBqfsiIQjUMxsnR2hDmg4CJrWpUo9fHOkAPBZ2NXTLTvnrAgMBAAE="); //"
       load_key( &public_key, pubkeydata);
       warnx("HARDCODED PUBLIC KEY LOADED: \n%s\n",pubkeydata);
       write_whole_file_to_sd("/fs/microsd/publickey.txt",pubkeydata,strlen(pubkeydata) );  // convenient way to get it on SD. 
     }
     if ( !hardcoded && readpublic ) {
     // OR you can read the public key from the SD card, if it exists there too: 
        read_key_from_sd("/fs/microsd/publickey.txt", &pubkeydata);   
        load_key( &public_key, pubkeydata );    
       warnx("SD PUBLIC KEY LOADED: %s",pubkeydata);
     }
     
      
   
    if (   verifycert ) { 
    warnx("VALIDATING THE CERT-OF-AUTHENTICITY using the PUBLIC key...\n");
    int result;
    result = verify_cert_and_return( &public_key, &prng, &cert, certlen, serialid); 
    if ( result == 1 ) { 
       warnx("PASSED Certificate Check for serial : %s\n", serial);
    
    } else { 
       warnx("FAILURE! FAILURE!! FAILURE!!! on Certificate Check for serial: %s\n", serial);
       return; // don't write OTP or log on errors 
    } 
    } 
    
      
    if ( log ) { 
    warnx("WRITING THE DATA TO THE SD CARD LOG\n"); 
    append_serial_data_to_sd_log( "/fs/microsd/OTPCertificates.log", serial, out, privkeydata );

    }

    // fetch OTP info: 
  
	uint16_t *fsize = ADDR_FLASH_SIZE;

	warnx("Flash size: %d", (int)*fsize);

	// get OTP memory /
	warnx("otp_mem Struct size: %d",sizeof(struct otp)); 
	struct otp otp_mem;
	const volatile uint32_t* otp_ptr = ADDR_OTP_START;
	val_read(&otp_mem, otp_ptr, sizeof(struct otp));

	// ID string //
	otp_mem.id[3] = '\0';
	
	warnx("ID: %s CERT: %s", otp_mem.id ,otp_mem.signature);

	// get OTP lock //
	struct otp_lock otp_lock_mem;
	const volatile uint32_t* otp_lock_ptr = ADDR_OTP_LOCK_START;
	val_read(&otp_lock_mem, otp_lock_ptr, sizeof(struct otp_lock));

	printf("OTP LOCK STATUS: ");
	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
		printf("%0X", otp_lock_mem.lock_bytes[i]);
	printf("\n");



	// TODO  Write signature to OTP and lock, but only on special command
  
    // WRITE THE DATA TO THE OTP
    //TODO 
    if ( writecert ) { 
        warnx("-w writecert not impl yet\n");
        
		warnx("Writing (but not locking) OTP");
		// write OTP /
		uint8_t id_type = 0;
		uint32_t vid = 0x26AC;
		uint32_t pid = 0x10;
		char* signature = 0;
		char* public_key = 0;

		write_otp(id_type, vid, pid, cert); // cert is 128 binary bytes.   
        
    }
    
    // LOCK THE OTP
    // TODO 
    if ( lockcert ) { 
        warnx("-l lockcert not impl yet\n");
		warnx("Locking OTP, no further write operations are permitted");
		lock_otp();
    } 
    
 	


	sched_unlock();
}
