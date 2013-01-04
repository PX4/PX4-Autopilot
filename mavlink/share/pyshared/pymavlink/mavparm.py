'''
module for loading/saving sets of mavlink parameters
'''

import fnmatch, math, time

class MAVParmDict(dict):
    def __init__(self, *args):
        dict.__init__(self, args)
        # some parameters should not be loaded from files
        self.exclude_load = ['SYSID_SW_MREV', 'SYS_NUM_RESETS', 'ARSPD_OFFSET', 'GND_ABS_PRESS',
                             'GND_TEMP', 'CMD_TOTAL', 'CMD_INDEX', 'LOG_LASTFILE', 'FENCE_TOTAL',
                             'FORMAT_VERSION' ]
        self.mindelta = 0.000001


    def mavset(self, mav, name, value, retries=3):
        '''set a parameter on a mavlink connection'''
        got_ack = False
        while retries > 0 and not got_ack:
            retries -= 1
            mav.param_set_send(name.upper(), float(value))
            tstart = time.time()
            while time.time() - tstart < 1:
                ack = mav.recv_match(type='PARAM_VALUE', blocking=False)
                if ack == None:
                    time.sleep(0.1)
                    continue
                if str(name).upper() == str(ack.param_id).upper():
                    got_ack = True
                    self.__setitem__(name, float(value))
                    break
        if not got_ack:
            print("timeout setting %s to %f" % (name, float(value)))
            return False
        return True


    def save(self, filename, wildcard='*', verbose=False):
        '''save parameters to a file'''
        f = open(filename, mode='w')
        k = self.keys()
        k.sort()
        count = 0
        for p in k:
            if p and fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                f.write("%-16.16s %f\n" % (p, self.__getitem__(p)))
                count += 1
        f.close()
        if verbose:
            print("Saved %u parameters to %s" % (count, filename))


    def load(self, filename, wildcard='*', mav=None):
        '''load parameters from a file'''
        try:
            f = open(filename, mode='r')
        except:
            print("Failed to open file '%s'" % filename)
            return False
        count = 0
        changed = 0
        for line in f:
            line = line.strip()
            if not line or line[0] == "#":
                continue
            line = line.replace(',',' ')
            a = line.split()
            if len(a) != 2:
                print("Invalid line: %s" % line)
                continue
            # some parameters should not be loaded from files
            if a[0] in self.exclude_load:
                continue
            if not fnmatch.fnmatch(a[0].upper(), wildcard.upper()):
                continue
            if mav is not None:
                if a[0] not in self.keys():
                    print("Unknown parameter %s" % a[0])
                    continue
                old_value = self.__getitem__(a[0])
                if math.fabs(old_value - float(a[1])) > self.mindelta:
                    if self.mavset(mav, a[0], a[1]):
                        print("changed %s from %f to %f" % (a[0], old_value, float(a[1])))
                    changed += 1
            else:
                self.__setitem__(a[0], float(a[1]))
            count += 1
        f.close()
        if mav is not None:
            print("Loaded %u parameters from %s (changed %u)" % (count, filename, changed))
        else:
            print("Loaded %u parameters from %s" % (count, filename))
        return True

    def show(self, wildcard='*'):
        '''show parameters'''
        k = sorted(self.keys())
        for p in k:
            if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                print("%-16.16s %f" % (str(p), self.get(p)))

    def diff(self, filename, wildcard='*'):
        '''show differences with another parameter file'''
        other = MAVParmDict()
        if not other.load(filename):
            return
        keys = sorted(list(set(self.keys()).union(set(other.keys()))))
        for k in keys:
            if not fnmatch.fnmatch(str(k).upper(), wildcard.upper()):
                continue
            if not k in other:
                print("%-16.16s              %12.4f" % (k, self[k]))
            elif not k in self:
                print("%-16.16s %12.4f" % (k, other[k]))
            elif abs(self[k] - other[k]) > self.mindelta:
                print("%-16.16s %12.4f %12.4f" % (k, other[k], self[k]))
                
        
