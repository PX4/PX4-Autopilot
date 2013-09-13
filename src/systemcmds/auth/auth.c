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
#include <systemlib/otp.h>
#include <libtomcrypt/tomcrypt_custom.h>
#include <libtomcrypt/tomcrypt.h>
#include <libtomcrypt/tomcrypt_misc.h>
#include <libtomfastmath/tfm.h>


__EXPORT int auth_main(int argc, char *argv[]);

// function headers: 
void sign_serial_and_return_cert( rsa_key * private_key , prng_state * prng, char * serialid,  char * newcert, int * certlen);
int  verify_cert_and_return(rsa_key * public_key, prng_state * prng, char * cert  , int certlen, char * oldserialid );
int read_key_from_sd(char * filename , char * blob );
int append_serial_data_to_sd_log( char * filename, char * serial, char * cert, char * private );
int write_whole_file_to_sd(char * filename, char * publickeydata, int datalen);
void human_readable_serial(char *serialid, char * serial );
void  make_public_key_from_private(rsa_key * key, char * outbuffer);
void load_key( rsa_key * key, char * PEMbuffer );



#define SERIAL_LEN 27
#define BIN_SERIAL_LEN 20  // or 24 
#define PADDING_TYPE LTC_LTC_PKCS_1_V1_5 // or LTC_LTC_PKCS_1_PSS
#define SALTLEN 0 // or 8, or 16 


void sign_serial_and_return_cert( rsa_key * private_key , prng_state * prng, char * serialid,  char * newcert, int * certlen) { 
    	// locate indexes into hash list and prng list  
	int hash_idx, prng_idx, errr;
    prng_idx = find_prng("yarrow");
    hash_idx = find_hash("sha1"); 

	  if (hash_idx == -1 || prng_idx == -1) {
          warnx( "sign requires LTC_SHA1(%d) and yarrow(%d)",hash_idx,prng_idx );
          return;
     }

    
    *certlen = 1024; // sizeof() no worky on undef pointers like the empty cert[] we passed in. 
    
    if ((errr =  rsa_sign_hash_ex(
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
            warnx("rsa_sign_hash_ex %s\n", error_to_string(errr));
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

    
   int retval, errr; // serial_len;
   
  // warnx("%s:%d:%s:%d\n",cert, certlen, oldserial, BIN_SERIAL_LEN ); 
   
   if ((errr =   rsa_verify_hash_ex(
        cert,
        certlen,
        oldserialid,
        BIN_SERIAL_LEN, // serial length / 
        PADDING_TYPE,
        hash_idx,
        SALTLEN, // saltlen /
        &retval,
        public_key)) != CRYPT_OK) {
            warnx("rsa_verify_hash %s\n", error_to_string(errr));
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
  
  //TODO check does this work in a way that doesn't crash or hang if SD isn't mounted. ? 

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

int append_serial_data_to_sd_log( char * filename, char * serial, char * cert, char * private ) { 
    
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

int write_whole_file_to_sd(char * filename, char * publickeydata, int datalen){ 

  FILE *fp;

 warnx("\t writing file: '%s' to SD done.",filename);
 
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
         int errr; 
          outlen2 = sizeof(OUTbuffer);
          outlenpem2 = sizeof(OUTPEMbuffer);
                    
         //PUBLIC
         // PK_PUBLIC means its just a public key component. 
         if ( (errr = rsa_export(OUTbuffer,&outlen2, PK_PUBLIC, key)) != CRYPT_OK ) {
              warnx("Error with rsa_export: %s\n", error_to_string(errr));
          } else { 
      	   warnx("PUBLIC rsa_export OK: size: %d\n", outlen2 );//
      	 }
         // convert what is essentially DER data into naked PEM format. ( ie no header or footer lines )  
          if ( (errr = base64_encode(OUTbuffer,outlen2,OUTPEMbuffer, &outlenpem2)) != CRYPT_OK ) {
              warnx("base64_encode error: %s\n", error_to_string(errr));
          }  else { 
          
          // this should print our public key, human readable:      
          warnx("base64 PUBLIC KEY.  len: %d key: %s\n", outlenpem2, OUTPEMbuffer );
          } 
          
          strcpy(outbuffer,OUTPEMbuffer); // return the good bit. 

}

// private or public, either is good. 
void load_key( rsa_key * key, char * PEMbuffer ) { 
    
    
	 //warnx(" load key start --------------------\n");// %s\n",PEMbuffer);
	   
	   int errr; 
	       
   
        unsigned long elen;
        unsigned long *eoutlen;
        // well convert it to DER format and store it here, in a sec. 
        unsigned char DERbuffer[1024];

            
        elen = strlen(PEMbuffer);// actual in len
        eoutlen = sizeof(DERbuffer); // max out len

        
        // convert what is essentially PEM data into DER format so we can decrypt it. 
        if ( (errr = base64_decode(PEMbuffer,elen,DERbuffer, &eoutlen)) != CRYPT_OK ) {
            warnx("\t base64_decode error: %s\n", error_to_string(errr));
        }  else { 
    //warnx("\t base64_decode OK actualoutlen: %d\n",eoutlen);
      //  warnx("PUBLIC base64_decode OK \n");
        }   
        
       	    // read OpenSSL DER formatted key into Tom's format. :-)
        if ( (errr = rsa_import( &DERbuffer,eoutlen,key)) != CRYPT_OK ) {
            warnx("\t Error with rsa_import : %s\n", error_to_string(errr));
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
		"auth [-x] [-p] [-h] [-m] [-c] [-e|-r|-o] [-d] [-w] [-k] [-v] [-l] [-x] [-t] [-d] [-f] [-s] [-q] \n"
// key related methods: 
		"  -x     Read Private Key ( usually from /fs/microsd/privatekey.txt ) and display it\n"
		"  -p     Read Public Key ( usually from /fs/microsd/publickey.txt ) and display it\n"
		"  -h     use Hardcoded Private/Public 'test' Keys, instead of the above file/s, no SD card needed\n"
		"  -m     make a public key file on SD card at /fs/microsd/publickey.txt from the private key and display it ( assumes also -x -p ) \n"
		"  -c     create a Certificate-Of-Authenticity with the private key and display it ( assumes also -x )  - do not use at same time as -r or -o \n"
// COA reading methods: 
        "  -e     read COA from Ephemeral output of -c command ( not compatible with -r or -o , assumes -c )   \n"
        "  -r     read a Raw COA from sd card in file: /fs/microsd/COA.bin \n"
        "  -o     read COA from Otp area in-chip. \n"
// serial reading methods: 		
		"  -a     read true serial from chip ROM ( the default , if none of -f -s -q are used )  \n"
		"  -f     use serial from /fs/microsd/COA.sid eg: a file containing:  '33002E 32314704 34303736'  \n"
		"  -s     use this specific hardcoded 'test' serial '33002E 32314704 34303736'  \n"
		"  -q     query user ( interactive) for a serial number, and create an OTP for that, like it was pulled from the ROM ( assumes -x -e -t ) \n"
// actions and whatnot: 
		"  -w     write the Certificate-Of-Authenticity to Flash ( assumes also -x and [-e|-r|-o] )  \n"
		"  -k     LOCK the Certificate-Of-Authenticity to ONE-TIME-PROGRAMMABLE Flash PERMANENTLY ( assumes -w -x [-e|-r|-o] ) \n"
		"  -v     Verify the Certificate-Of-Authenticity just read, with just the public key, and display results.  ( assumes -p [-e|-r|-o]) \n"
		"  -t     test a bunch ( equivalent to -x -p -c -e -a -v -l -y -n)  \n"
		"  -l     log it.  Append the COA and privatekey info to the SD card at /fs/microsd/OTPCertificates.log \n"
		"  -y     extended dump.  Write the COA and Serial info to 4 files on the SD card at /fs/microsd/COA.* \n"
		"  -d     dump Hardcoded Private/Public Keys onto SD card ( assumes also -x -p -h ) \n"
		"  -n     dump true serial number from chip onto SD card in file: /fs/microsd/COA.sid \n"
		
		" CAUTION: AUTH IS STACK/RAM HUNGRY. CAN HANG IF YOU RUN IT MORE THAN ONCE PER BOOT\n"
		" typical use-case for blank SD card:  ( to put keys on SD card from flash )  'auth -d' , [reboot] , then 'auth -e -t -l' or 'auth -v -l ' \n"
		);

}


int auth_main(int argc, char *argv[])
{

     warnx("AUTH started.\n");
     
     F_unlock();
       
    int ch, errr; 

	if (argc < 2)
		usage(NULL);
		

    bool readprivate = false;        //-x
    bool readpublic = false;         //-p
    bool hardcoded = false;        //-h
    bool makepublic = false;        //-m
    bool createcert = false;        //-c
    
    bool writeCOA = false;        //-w
    bool lockOTP  = false;        //-k
    bool verifyCOA = false;        //-v
  //  bool testCOA = false;        //-t
    bool log = false;        //-l
    bool xlog = false;        //-x
    bool dumpkeystosd = false;        //-d
    bool dumpserialtosd = false;        //-n
     
    bool usetrueserial = false;        //-a
    bool usehardcodedserial = false;        //-s
    bool useSDserial = false;        //-f
    bool useinteractiveserial = false;         //-q 
    
   
    bool ephloader = false;        //-
    bool rawloader = false;        //-
    bool otploader = false;        //-

	while ((ch = getopt(argc, argv, "xpcwlyvthdmkafsqeron")) != EOF) {
		switch (ch) {
		case 'e': // ephermal OTP loader from -c results
		    ephloader = true;
		    rawloader = false; otploader = false;
		    createcert = true;  // can't run tests/verify on a ephermal cert without making it. 
		    // and createcert needs this too: 
		    readprivate = true;
			break;
		case 'r': // raw OTP loader from SD  similar to -c , but read from file. 
		    rawloader = true;
		    ephloader = false; otploader = false; 
		    createcert = false; // just in case, idiots.
			break;
		case 'o': // typical OTP loader from OTP area  
		    otploader = true;
		    rawloader = false; ephloader = false;
		    createcert = false; // just in case, idiots.
			break;
		case 'x': // load private 
		    readprivate = true;
			break;
		case 'p': // load public
		      readpublic = true;
			break;
		case 'm': // load public
		      makepublic = true;
		      // and 
		      readpublic = true; readprivate = true;
			break;
		case 'c': // generate cert-of-auth,  assumes -x 
		      createcert = true;
		      // also
		      readprivate = true;
			break;
		case 'w': // write cert-of-auth to flash, assumes -x and -c 
		      writeCOA = true;
		      //also
		      readprivate = true; createcert = true;
			break;
		case 'k': // lock flash, assumes -w -x -c
		      lockOTP = true;
		      //also
		      readprivate = true;createcert = true;writeCOA = true;
			break;
		case 'v': //verify cert-of-auth, assumes -p
		      verifyCOA = true;
		      // also 
		       readpublic = true;
			break;
		case 't': //verify/test cert-of-auth just generated without OTP ( equivalent to - //-x -p -c -e -a -v -l -y -n )
		      readprivate = true; readpublic = true; createcert = true; 
		      ephloader = true; usetrueserial = true; verifyCOA = true;
		      log = true; xlog = true; dumpserialtosd = true;
		     
			break;
		case 'l': //log it.
		      log = true;
			break;
		case 'y': //y = extended dump/log it.
		      xlog = true;
			break;
		case 'h': //Hardcoded keys
		      hardcoded = true;
			break;
		case 'd': //dump Hardcoded keys to SD
		      dumpkeystosd = true;
		      // also neds: 
		      hardcoded = true; readprivate = true; readpublic = true;
			break;
		case 'n': //dump serial number to SD
		      dumpserialtosd = true;
		      // also neds: 
		      usetrueserial = true; useSDserial=false; usehardcodedserial=false; useinteractiveserial=false;
			break;
		case 'a': //true serial from chip/ROM
		      usetrueserial = true;
			break;
		case 'f': //Hardcoded serial
		      useSDserial = true;
			break;
		case 's': //Hardcoded serial
		      usehardcodedserial = true;
			break;
		case 'q': // query interactive  serial 
		      useinteractiveserial = true;
			break;
		default:
			usage(NULL);
		}
	}
	argc -= optind;
	argv += optind;

	// precaution to avoid bad OTP	
	if ( ( writeCOA  ||  lockOTP ) && (useSDserial || usehardcodedserial || useinteractiveserial)  ) { 
		      warnx("I'm sorry, you really shouldn't write or lock an OTP for that serial, so we don't let you"); 	
		      writeCOA = false;  lockOTP = false;
   
	} ;
  
	
	 // ONE-TIME-PROGRAMMABLE (OTP) MEMORY HANDLING SECTION
	

	//disable scheduling, leave interrupt processing untouched 
	sched_lock();


//-----------------------------------KEY SETUP CODE----------------------------------------------	
	

    // setup RSA support code   
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
            if ((errr = yarrow_start(&prng)) != CRYPT_OK) {
            warnx("Start error: %s\n", error_to_string(errr));
            }
            /* add entropy */
            if ((errr = yarrow_add_entropy("hello world", 11, &prng))
            != CRYPT_OK) {
            warnx("Add_entropy error: %s\n", error_to_string(errr));
            }
            /* ready and read */
            if ((errr = yarrow_ready(&prng)) != CRYPT_OK) {
            warnx("Ready error: %s\n", error_to_string(errr));
            }
            
            //warnx("Read %lu bytes from yarrow\n", yarrow_read(buf, sizeof(buf), &yarrow_prng));
        
           if (register_hash(&sha1_desc) == -1) {  warnx("Critical Error registering sha1 hash");   }
 

    // register a math library (in this case TomsFastMath)
    // this is important, don't forget it. 
    ltc_mp = tfm_desc;
    
//----------------------------------- PRIVATE KEY -READING-CODE----------------------------------------------	
    
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
        if ( dumpkeystosd ) { 
            write_whole_file_to_sd("/fs/microsd/privatekey.txt",privkeydata,strlen(privkeydata) );  // convenient way to get it on SD.  
        }        
    } 
    if ( !hardcoded && readprivate ) {
        // OR read a private key from SD card like this: 
        read_key_from_sd("/fs/microsd/privatekey.txt", &privkeydata);   
        load_key( &private_key, privkeydata );  
        warnx("SD PRIVATE KEY LOADED.\n"); //,privkeydata);
    }
    

//----------------------------------- PUBLIC KEY -READING-CODE----------------------------------------------	
    

     // need public key to do the "verify" ... THREE ways to get it:  
      char pubkeydata[255] = "";
      
      // optional way of using the private key to make the public key, works, but not typiucally needed if you make your keys externally. 
      if ( makepublic ) { 
        // TODO make this test for the presence of the publickey.txt, and only if not there, make it?  :-) 
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
       if ( dumpkeystosd ) { 
            write_whole_file_to_sd("/fs/microsd/publickey.txt",pubkeydata,strlen(pubkeydata) );  // convenient way to get it on SD. 
       }
     }
     if ( !hardcoded && readpublic ) {
     // OR you can read the public key from the SD card, if it exists there too: 
        read_key_from_sd("/fs/microsd/publickey.txt", &pubkeydata);   
        load_key( &public_key, pubkeydata );    
       warnx("SD PUBLIC KEY LOADED: %s",pubkeydata);
     }
     
    

	
//-----------------------------------SERIAL-READING-CODE----------------------------------------------	
	
    //binary version of serial, padded with zero bytes. 
 	unsigned char serialid[BIN_SERIAL_LEN]; 
 	
 	// assume -a if not told otherwise. 
 	if ( ( usetrueserial == false ) && (  usehardcodedserial == false ) && (useSDserial == false) && (useinteractiveserial == false) ) { 
	     warnx("assuming -a as none of -a -f -s -q given.");
  	   usetrueserial = true; 
 	}
 	
 		
	// read serial from real chip ROM. 
	if ( usetrueserial  ) { 
	     warnx("usetrueserial");
    	// read out unique chip ID / 
    	const volatile uint32_t* udid_ptr = (const uint32_t*)UDID_START;
    	union udid id;
    	val_read(&id, udid_ptr, sizeof(id));
 
 	
     	//  this copies the data from the id "union" to the serialid char[] array, but on hthe px4, it's the wrong endian order.
        //memcpy(serialid,id.data,12); // first 12 bytes are from serial 
    	//      
    	   // this is the right order for the px4: , there's a neater way to do this, but it'll do for now. 
    	serialid[0] = id.data[3]; 	serialid[1] = id.data[2];	serialid[2] = id.data[1];	serialid[3] = id.data[0];
    	serialid[4] = id.data[7]; 	serialid[5] = id.data[6];	serialid[6] = id.data[5];	serialid[7] = id.data[4];
    	serialid[8] = id.data[11]; 	serialid[9] = id.data[10];	serialid[10] = id.data[9];	serialid[11] = id.data[8];
    	
    	// only allows to dump the *real* serial to the SD card ( in binary format, for compatability with -f ) 
    	if ( dumpserialtosd ) { 
            //write_whole_file_to_sd("/fs/microsd/COA.ser",serial,SERIAL_LEN ); 
            write_whole_file_to_sd("/fs/microsd/COA.sid",serialid,BIN_SERIAL_LEN ); 
    	} 
    	
   	    useSDserial = false;  usehardcodedserial = false;  useinteractiveserial = false; 
	} 	
 
    // allow to override the serial number 	
    if ( usehardcodedserial ) { 
	     warnx("usehardcodedserial");
        
        // compound literal trick to assign a whole array at once... // michael! 
        memcpy(serialid, (const char[]){0x00,0x33,0x00,0x2E,0x32,0x31,0x47,0x04,0x34,0x30,0x37,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, sizeof serialid);
        
       //   0x00, 0x3D, 0x00, 0x1F, 0x32, 0x31, 0x47, 0x0C, 0x34, 0x30, 0x37, 0x36    // buzz
       //   0x00, 0x33, 0x00, 0x2E, 0x32, 0x31, 0x47, 0x04, 0x34, 0x30, 0x37, 0x36    // michael 
       
       useSDserial = false;  useinteractiveserial = false; 
    }
    
    // load an arbitrary serial ( in BINARY FORMAT! ) from a file on SD, and use that for testing: 
    if ( useSDserial ) { 
 	     warnx("useSDserial");
        
           int t;
//           t = read_key_from_sd("/fs/microsd/COA.ser",&serial);
//           if ( t != SERIAL_LEN ) {    
//             warnx("COA.ser is not %d bytes long - PROBABLY A BIG ERROR",SERIAL_LEN);
//           }
           t = read_key_from_sd("/fs/microsd/COA.sid",&serialid ); 
           if ( t != BIN_SERIAL_LEN ) {    
             warnx("COA.sid is not %d bytes long - PROBABLY A BIG ERROR");            
           }
           
           useinteractiveserial = false; 
    } 
    
    if ( useinteractiveserial )  { 
 	     warnx("useinteractiveserial");
        //TODO 
        warnx("ERROR !! -q is NOT IMPLEMENTED YET!  - sorry! "); 
        //die (); 
        
    } 

    // zero buffer to 20 bytes. 
    serialid[12] = 0;  serialid[13] = 0;  serialid[14] = 0;  serialid[15] = 0;   
    serialid[16] = 0;  serialid[17] = 0;  serialid[18] = 0;  serialid[19] = 0; 
    
 	   	
	unsigned char serial[SERIAL_LEN]; // human-readable version is 27 bytes  ( 3x8 + two whitespace +null) 
	
	human_readable_serial( &serialid, &serial ) ;  // from serialid, to serial
	
	if ( usehardcodedserial ) { 
        warnx("WARNING !!!! FORCING hardcoded serial: '%s' - TESTING ONLY\n", serial); 
    }
    if ( useSDserial  ) { 
        warnx("WARNING !!!! FORCING override serial from SD: '%s' - TESTING ONLY\n", serial); 
    } 
    
    
    
    //TIP:    FROM THIS POINT FORWARD, 'serial' is the human-readable version, and  'serialid' is the binary version.
    // the human-readalbe version is used to display to teh screen, and in logs... while the binary version is used in encryption functuions and on SD card in .txt files etc 

 
    // tell the user; 
    warnx("Using serial # [%s]", serial); 

  
//-----------------------------------COA-CODE----------------------------------------------	

//  CERT-OF-AUTHENTICITY PRIMARY VARIABLES: 
    char cert[1024];  // this is where the binary CERT-OF-AUTH goes.
    int certlen; 
    char out[255];  // THIS IS WHERE THE human-readable CERT-OF-AUTH goes….
    int outlen = 128; // basic cert len.
    

//-----------------------------------COA-MAKING-CODE----------------------------------------------	

    if ( createcert ) { 
         warnx("\t signing [binary] serial data with private key...");
         sign_serial_and_return_cert( &private_key, &prng, serialid, &cert, &certlen);    
          
         // give us the human-readable version too….  
         warnx("\t cert len:%d rawbytes: %02x %02x %02x %02x %02x ...(etc)", certlen, cert[0],cert[1],cert[2],cert[3],cert[4]);
         outlen = 1024; // maxlen
         base64_encode(cert, certlen, &out, &outlen);
         warnx("\nMADE:\nCERT-OF-AUTHENTICITY: for serial: %s \nCERT-OF-AUTHENTICITY: humanreadable: %s\nCERT-OF-AUTHENTICITY: rawsize: %d  humanlen: %d \n", serial, out,certlen, outlen);
    } 
    

//-----------------------------------COA-READING-CODE----------------------------------------------	

    // use generated OTP from -c output 
	if (   ephloader )  { 
	   
	   // neat, we've already loaded it into cert, certlen, out, outlen above in the "COA making code" .  
	   // nothing else to do here but force-prevent other loaders working in the parameter parser. 
	   otploader = false; 	rawloader = false;   
	   
	} 

	// read COA from actual OTP area, if requested. 
	if ( otploader  )  { 
	
      	// fetch OTP info: 
      	uint16_t *fsize = ADDR_F_SIZE;
      	warnx("Flash size: %d", (int)*fsize);
      	// get OTP memory 
      	warnx("otp_mem Struct size: %d",sizeof(struct otp)); 
      	struct otp otp_mem;
      	const volatile uint32_t* otp_ptr = ADDR_OTP_START;
      	val_read(&otp_mem, otp_ptr, sizeof(struct otp));
      	
        // OTP data as binary COA data
        certlen = sizeof(otp_mem.signature); // 128bytes
        memcpy(cert,otp_mem.signature,certlen);
       // human readable too: 
        outlen = 1024; // maxlen
        base64_encode(cert, certlen, &out, &outlen);

        // tell them the key OTP stuff ( like PX4/VID/PID/ etc ) too...
        warnx("OTP DATA:- id: %s (%02X-%02X-%02X-%02X) vid:%04X pid:%04X \n", otp_mem.id, \
        otp_mem.id[0],otp_mem.id[1],otp_mem.id[2],otp_mem.id[3], otp_mem.vid, otp_mem.pid); //4 char/bytes

        warnx("OTP CERT-OF-AUTHENTICITY for serial?: %s rawsize: %d  humanlen: %d , humanreadable:%s\n", serial, certlen, outlen, out);

       rawloader = false;   
	} 
	
	// read OTP from SD, fake it.  :-) 
	if (   rawloader )  { 
	   
         certlen = read_key_from_sd("/fs/microsd/COA.bin", &cert);   
//         outlen = read_key_from_sd("/fs/microsd/COA.b64", &out);      // a human-readable equivalent
         
        // make a human readable from the binary version: 
        outlen = 1024; // maxlen
        base64_encode(cert, certlen, &out, &outlen);

        warnx("SD CERT-OF-AUTHENTICITY for serial?: %s rawsize: %d  humanlen: %d , humanreadable:%s\n", serial, certlen, outlen, out);
	} 
  	

 
 
//-----------------------------------OTHER TASKS CODE: ----------------------------------------------	

   
    
    // simple test suite for SD card read/write with both binary and ascii files.
//    if ( false ) { 
//        write_whole_file_to_sd("/fs/microsd/test1.txt",cert,certlen ); 
//        char cert2[1024];  // this is where the binary CERT-OF-AUTH goes.
//        int certlen2;
//        certlen2 = read_key_from_sd("/fs/microsd/test1.txt", &cert2);   
//        warnx("SD test 1:  certlen: %d certlen2: %d cert: %s  cert2:%s\n",certlen, certlen2, cert, cert2);
//        
//        write_whole_file_to_sd("/fs/microsd/test2.txt",out,outlen ); 
//        char out2[255];  // this is where the text CERT-OF-AUTH goes.
//        int outlen2;
//        outlen2 = read_key_from_sd("/fs/microsd/test2.txt", &out2);   
//        warnx("SD test 2:  outlen: %d outlen2: %d out: %s  out2:%s\n",outlen, outlen2, out, out2);
//    }
    
    // mess with cert:  should cause a fail when this is uncommented. 
    //cert[12] = 1;
    
    if  ( certlen <= 0 ) { 
        warnx("WARNING: no COA loaded or generated, can not dump, test, verify, or log.  use -e , -r or -o ");
          writeCOA = false;  lockOTP = false; // to be safe. 
    } 
    
    if ( xlog && ( certlen > 0 )) { 
        warnx("EXTENDED DUMP: Writing COA and Serial data to SD card ( .b64 are human readable)  ... with certlen: %d\n",certlen); 
        write_whole_file_to_sd("/fs/microsd/COA.bin",cert,certlen );        
        write_whole_file_to_sd("/fs/microsd/COA.b64",out,outlen ); 
        write_whole_file_to_sd("/fs/microsd/COA.ser",serial,SERIAL_LEN ); 
        write_whole_file_to_sd("/fs/microsd/COA.sid",serialid,BIN_SERIAL_LEN ); 
    } else { 
       // warnx("TESTING: To write this COA data to SD card, also specify -l\n"); 
    }
        

    if ( verifyCOA && ( certlen > 0 ) ) { 
        warnx("VALIDATING THE CERT-OF-AUTHENTICITY using the available PUBLIC key/s...\n");

//BUZZ KEY:     
strcpy(pubkeydata, "MIGJAoGBANAoM9l144JIgPqNJiI/xfaasJFgc3BunzMebu6b+yqjbGRacjUhO8qOF\n\
fKkXFD+h7WDXJ5uAkljQBL46l5nLdEewNGA8uidXOqXRzphl2Wt7Jw5Eu4krZcWRQ5\n\
JUhfBqfsiIQjUMxsnR2hDmg4CJrWpUo9fHOkAPBZ2NXTLTvnrAgMBAAE="); //"

       load_key( &public_key, pubkeydata);
       warnx("BUZZS PUBLIC KEY LOADED: \n%s\n",pubkeydata);

       int result1;
       result1 = verify_cert_and_return( &public_key, &prng, &cert, certlen, serialid); 
       if ( result1 == 1 ) {        warnx("PASSED Certificate Check WITH BUZZS KEY for serial : %s\n", serial);    }  

//3DR KEY:  
strcpy(pubkeydata, "MIGfMA0GCSqGSIb3DQEBAQUAA4GNADCBiQKBgQDqi8E6EdZ11iE7nAc95bjdUTwd\n\
/gLetSAAx8X9jgjInz5j47DIcDqFVFKEFZWiAc3AxJE/fNrPQey16SfI0FyDAX/U\n\
4jyGIv9w+M1dKgUPI8UdpEMS2w1YnfzW0GO3PX0SBL6pctEIdXr0NGsFFaqU9Yz4\n\
DbgBdR6wBz9qdfRRoQIDAQAB");

       load_key( &public_key, pubkeydata);
       warnx("3DR PUBLIC KEY LOADED: \n%s\n",pubkeydata);
 
       int result2; 
       result2 = verify_cert_and_return( &public_key, &prng, &cert, certlen, serialid); 
       if ( result2 == 1 ) {        warnx("PASSED Certificate Check WITH 3DR KEY for serial : %s\n", serial);    }

// ADD MORE PUBLIC KEYS HERE, IF DESIRED:     
    
       if ( ( result1 != 1 ) && ( result2 != 1  ) ) { 
          warnx("FAILURE! FAILURE!! FAILURE!!! on ALL Certificate Check/s for serial: %s\n", serial);
          writeCOA = false;
          lockOTP = false;
          return; // don't write OTP or log on errors 
       } 
    } 
    
      
    if ( log && ( certlen > 0 )) { 
    warnx("WRITING THE DATA TO THE SD CARD LOG\n"); 
    append_serial_data_to_sd_log( "/fs/microsd/OTPCertificates.log", serial, out, privkeydata );

    }



	// get OTP lock //
	struct otp_lock otp_lock_mem;
	const volatile uint32_t* otp_lock_ptr = ADDR_OTP_LOCK_START;
	val_read(&otp_lock_mem, otp_lock_ptr, sizeof(struct otp_lock));

	printf("OTP LOCK STATUS: ");
	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
		printf("%02X ", otp_lock_mem.lock_bytes[i]);
	printf("\n");



	// TODO  Write signature to OTP and lock, but only on special command
  
    // WRITE THE DATA TO THE OTP
    //TODO 
    if ( writeCOA ) { 
        
		warnx("Writing (but not locking) OTP");
		// write OTP /
		uint8_t id_type = 0;
		uint32_t vid = 0x26AC;
		uint32_t pid = 0x10;
		
		write_otp(id_type, vid, pid, cert); // cert is 128 binary bytes.   
       
    }
    
    // LOCK THE OTP - this is the -k parameter!
    if ( lockOTP ) { 
 		warnx("Locking OTP, no further write operations are permitted");
		lock_otp();
		
 
 		warnx("New Lock Status:");
   			// get OTP lock //
    	struct otp_lock otp_lock_mem;
    	const volatile uint32_t* otp_lock_ptr = ADDR_OTP_LOCK_START;
    	val_read(&otp_lock_mem, otp_lock_ptr, sizeof(struct otp_lock));
    
    	printf("OTP LOCK STATUS: ");
    	for (int i = 0; i < sizeof(otp_lock_mem) / sizeof(otp_lock_mem.lock_bytes[0]); i++)
    		printf("%02X ", otp_lock_mem.lock_bytes[i]);
    	printf("\n");


    } 
    
 	
    F_lock();

	sched_unlock();
	warnx("ALL DONE!");
}
