#!/usr/bin/python

import os
import sys,getopt
import commands
import ConfigParser
import socket
import fcntl
import struct
import re

class Enum(object):
    def __init__(self, *keys):
        self.__dict__.update(zip(keys, range(len(keys))))

enum_parse_tag = Enum('tag','item')
enum_config = Enum('std','ext')

def put_file_contents(filename,data=''):
    if not os.path.isfile(filename):
        return ''
    try:
        f = open(filename,'w')
        contents = f.write(data)
        f.close()
    except IOError:
        print 'file open or write err!'

def fix_nuttx_cfg_value(key,val):
    #if key == "CONFIG_NSH_IPADDR":
    if re.search(r"_IPADDR$", key):
        return '%s=%s\n'%(key,get_local_ip(50,2))
    #elif key == "CONFIG_NSH_DRIPADDR":
    elif re.search(r"_DRIPADDR$", key):
        return '%s=%s\n'%(key,get_local_ip(1,2))
    else:
        return '%s=%s\n'%(key,val)

def fix_nuttx_config(cfg,fix_cfgs):
    result = ''
    for line in cfg.readlines():
        line = line.strip()
        if line.startswith('#'):
            result += line+'\n'
            continue
        elif len(line) == 0:
            result += '\n'
            continue
        elif line.count('=') > 0 :
            #print line
            it = line.find('=')
            key = line[0:it].strip()
            val = line[it + 1:].strip()
            if fix_cfgs.has_key(key):
                result += fix_nuttx_cfg_value(key, fix_cfgs.pop(key))
            else:
                result += line+'\n'
        else:
            result += line+'\n'
    #fix_cfgs = {}
    #print fix_cfgs.viewitems()
    if (len(fix_cfgs.keys())>0):
        result += '\n\n####################################################\n\n'
    for key in fix_cfgs.keys():
        result += fix_nuttx_cfg_value(key, fix_cfgs.pop(key))
    return result


def usage():
    program_name = sys.argv[0]
    print 'Nuttx configure utils v 0.3\n'
    print '  usage: %s [-abcdfhmr] [-b boardname] [-l number]\n'%(program_name)
    print '    -c, --clean      : Will execute "make clean" command.'
    print '    -d, --distclean  : Will execute "make distclean" command.'
    print '    -l, --cleanlevel : Will execute "make clean"(value of 1) or "make distclean"(value of 2) command.'
    print '    -f, --fixcfg     : The configuration correction nuttx.cfg defconfig configuration items.'
    print '    -a, --auto       : Equivalent parameters -d and -f.'
    print '    -b, --boardname  : The boardname configuration.'
    print '    -r, --mkromfs    : Make Romfs.'
    print '    -m, --make       : Make now.'
    print '    -h, --help       : Help Message.'
    print '\n  example:'
    print '     usage 1 :   %s -b fire-stm32v2/nsh'%(program_name)
    print '     usage 2 :   %s -f -b fire-stm32v2/nsh'%(program_name)
    print '     usage 3 :   %s -a -b fire-stm32v2/nsh'%(program_name)
    print '     usage 4 :   %s -l 1 -f -b fire-stm32v2/nsh'%(program_name)

def fix_config(boardname,fix_cfgs):
    cfg_path = './configs/%s/defconfig'%boardname
    if (os.path.isfile(cfg_path)):
        try:
            cfg = open(cfg_path, "r")
            contents = fix_nuttx_config(cfg,fix_cfgs)
            cfg.close()
            #print contents
            put_file_contents(cfg_path,contents)
        except IOError:
            print 'nuttx config open err!'
            return ''

def fix_root_config(fix_cfgs):
    cfg_path = '.config'
    if (os.path.isfile(cfg_path)):
        try:
            cfg = open(cfg_path, "r")
            contents = fix_nuttx_config(cfg,fix_cfgs)
            cfg.close()
            #print contents
            put_file_contents(cfg_path,contents)
        except IOError:
            print 'nuttx config open err!'
            return ''

def get_local_ipn(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = fcntl.ioctl(s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15]))[20:24]
    return ip

def get_local_ip(id=-1,mode=1):
    ip = get_local_ipn('eth0')
    if (id < 0):
        id = ord(ip[3])
    ipr = ord(ip[0])<<24|ord(ip[1])<<16|ord(ip[2])<<8|(id & 0xFF)
    if mode == 1 :
        return socket.inet_ntoa(struct.pack('>L', ipr))
    if mode == 2 :
        return "0x%x"%ipr
    else:
        return ipr

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def config(cfg={}):
    output = ''
    fix_cfgs = {}
    if cfg.clearlevel == 2:
        print "make distclean apps_distclean"
        output = commands.getoutput("make distclean apps_distclean")
    elif cfg.clearlevel == 1:
        print "make clean apps_clean"
        output = commands.getoutput("make clean apps_clean")

    ini = ConfigParser.ConfigParser()
    ini.optionxform = str
    cfgf = 'nuttx_cfg.py'
    #if not(os.path.isfile(cfgf)):
        #fp = open(cfgf,'w')
        #fp.write('# is nuttx config\n[info]\nlast=')
        #fp.close()
    #cfgf = open(cfgf,'rw')
    ini.readfp(open(cfgf))
    lastfile = ini.get('info','last')
    if (cmp(lastfile,cfg.boardname) != 0):
        if ((cfg.boardname.strip() !='')):
            lastfile = cfg.boardname
            ini.set('info','last',lastfile)
            ini.write(open(cfgf, "w"))
    if cfg.usefix:
        #print lastfile
        if ini.has_section('defconfig'):
            opts = ini.items('defconfig')
            for item in opts:
                fix_cfgs[item[0]] = item[1]
        if ini.has_section(lastfile):
            opts = ini.items(lastfile)
            for item in opts:
                fix_cfgs[item[0]] = item[1]
        #print fix_cfgs
        #fix_config(boardname,fix_cfgs)

    output = commands.getoutput("cd tools;./configure.sh %s;cd .."%lastfile)
    print "tools/configure.sh %s"%lastfile
    if cfg.usefix:
        fix_root_config(fix_cfgs)
        print "fix .config file"
    if cfg.mkromfs:
        if (os.path.isfile("./rcS.template")):
            print "tools/mkromfsimg.sh ."
            #output = commands.getoutput("tools/mkromfs.sh .")
            os.system("tools/mkromfsimg.sh .")
            bpath = "configs/%s/include"%(lastfile.split('/')[0])
            if (not os.path.exists(bpath)):
                print "mkdir -p %s"%(bpath)
                #commands.getoutput("mkdir -pv %s"%(bpath))
                os.system("mkdir -pv %s"%(bpath))
                if (not os.path.exists(bpath)):
                    print "[ERROR] no %s"%(bpath)
            if (os.path.isfile("nsh_romfsimg.h")):
                print "cp nsh_romfsimg.h %s/nsh_romfsimg.h"%(bpath)
                #output = commands.getoutput("cp nsh_romfsimg.h include/arch/board/nsh_romfsimg.h")
                os.system("cp nsh_romfsimg.h %s/nsh_romfsimg.h"%(bpath))
    if cfg.make:
        os.system("sleep 3")
        print "make"
        os.system("make")
    if '' != output.strip():
        print output

class cfgobj():
    usefix = False
    mkromfs = False
    make = False
    clearlevel = 0
    boardname = ''

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "dcb:hal:frm",
            ["dictclean","clean","board=", "auto", "cleanlevel=",
            "help","fixcfg","mkromfs","make"])
    except getopt.GetoptError:
        # print help information and exit:
        usage()
        sys.exit(2)
    #cfg = {"usefix":False,"mkromfs":False,"make":False,"clearlevel":0,"boardname":'' }
    cfg = cfgobj()
    for o, v in opts:
        if o in ("-a","--auto"):
            cfg.clearlevel = 2
            cfg.usefix = True
            cfg.mkromfs = True
            cfg.make = True
        if o in ("-c","--clean"):
            cfg.clearlevel = 1
        if o in ("-d","--distclean"):
            cfg.clearlevel = 2
        if o in ("-l","--cleanlevel"):
            if v == '1':
                cfg.clearlevel = 1
            elif v == '2':
                cfg.clearlevel = 2
            else:
                cfg.clearlevel = 0
        if o in ("-b","--board"):
            cfg.boardname = v
        if o in ("-f","--fixcfg"):
            cfg.usefix  = True
        if o in ("-r","--mkromfs"):
            cfg.mkromfs  = True
        if o in ("-m","--make"):
            cfg.make  = True
        if o in ("-h", "--help"):
            usage()
            sys.exit()
    config(cfg)

if __name__ == "__main__":
    main()
