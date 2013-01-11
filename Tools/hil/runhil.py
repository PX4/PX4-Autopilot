#!/usr/bin/env python

'''
runs hil simulation
'''

# system import
import sys, struct, time, os, argparse, signal, math
import pexpect, socket, fdpexpect, select
import pymavlink.mavutil as mavutil

if os.getenv('MAVLINK09') or 'MAVLINK09' in os.environ:
    import pymavlink.mavlinkv09_pixhawk as mavlink
else:
    import pymavlink.mavlinkv10_pixhawk as mavlink

# set path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))

import hangar
import gcs
from constants import *

# local imports
import util, atexit
import pymavlink.fgFDM as fgFDM

from math import sin, cos

class SensorHIL(object):
    ''' This class executes sensor level hil communication '''

    @classmethod
    def command_line(cls):
        ''' command line parser '''
        parser = argparse.ArgumentParser()
        parser.add_argument('--master', help='device', required=True)
        parser.add_argument('--baud', help='master port baud rate', default=921600)
        parser.add_argument('--script', help='jsbsim script', default='data/easystar_test.xml')
        parser.add_argument('--options', help='jsbsim options', default=None)
        parser.add_argument('--gcs', help='gcs host', default='localhost:14550')
        parser.add_argument('--waypoints', help='waypoints file', default='data/sf_waypoints.txt')
        parser.add_argument('--mode', help="hil mode (sensor or state)", default='sensor')
        args = parser.parse_args()
        if args.master is None:
            raise IOError('must specify device with --dev')
        if args.mode not in ['sensor','state']:
            raise IOError('mode must be sensor or state')
        inst = cls(master_dev=args.master, baudrate=args.baud, script=args.script, options=args.options, gcs_dev=args.gcs, waypoints=args.waypoints, mode = args.mode)
        inst.run()

    def __init__(self, master_dev, baudrate, script, options, gcs_dev, waypoints, mode):
        ''' default ctor 
        @param dev device
        @param baud baudrate
        '''
        self.script = script
        self.options = options
        self.waypoints = waypoints
        self.mode = mode

        self.ac = hangar.BasicAircraft()
        self.jsb = None
        self.jsb_console = None
        self.gcs = None
        self.master = None

        self.counts = {}
        self.bytes_sent = 0
        self.bytes_recv = 0
        self.frame_count = 0
        self.last_report = 0

        self.init_jsbsim()
        self.init_mavlink(master_dev, gcs_dev, baudrate)
        self.wpm = gcs.WaypointManager(self.master)

    def __del__(self):
        print 'SensorHil shutting down'
        # JSBSim really doesn't like to die ...
        if self.jsb_console is not None:
            self.jsb_console.send('quit\n')
        if self.jsb is not None and getattr(self.jsb, 'pid', None) is not None:
            os.kill(self.jsb.pid, signal.SIGKILL)
            self.jsb.close(force=True)
        util.pexpect_close_all()

    def init_mavlink(self, master_dev, gcs_dev, baudrate):

        # master
        master = mavutil.mavserial(master_dev, baud=baudrate, autoreconnect=True)
        print 'master connected on device: ', master_dev

        # gcs
        if gcs_dev is not None:
            gcs = mavutil.mavudp(gcs_dev, input=False)
            print 'gcs connected on device: ', gcs_dev

        # class data
        self.master = master
        self.gcs = gcs

    def init_jsbsim(self):
        cmd = "JSBSim --realtime --suspend --nice --simulation-rate=1000 --logdirectivefile=data/fgout.xml --script=%s" % self.script
        if self.options:
            cmd += ' %s' % self.options

        jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
        jsb.delaybeforesend = 0
        util.pexpect_autoclose(jsb)
        i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                        "Could not bind to socket for input"])
        if i == 1:
            print("Failed to start JSBSim - is another copy running?")
            sys.exit(1)
        jsb_out_address = self.interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
        jsb.expect("Creating UDP socket on port (\d+)")
        jsb_in_address = self.interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
        jsb.expect("Successfully connected to socket for output")
        jsb.expect("JSBSim Execution beginning")

        # setup output to jsbsim
        print("JSBSim console on %s" % str(jsb_out_address))
        jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        jsb_out.connect(jsb_out_address)
        jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
        jsb_console.delaybeforesend = 0

        # setup input from jsbsim
        print("JSBSim FG FDM input on %s" % str(jsb_in_address))
        jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        jsb_in.bind(jsb_in_address)
        jsb_in.setblocking(0)

        # set class data
        self.jsb = jsb
        self.jsb_in = jsb_in
        self.jsb_out = jsb_out
        self.jsb_console = jsb_console
        self.fdm = fgFDM.fgFDM()

    def hil_enabled(self):
        return self.master.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED

    def enable_hil(self):
        ''' enable hil mode '''
        t_start = time.time()
        while not self.hil_enabled():
            self.master.set_mode_flag(mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,True)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
            time.sleep(0.001)
            if time.time()  - t_start > 5: raise IOError('Failed to enable HIL, check port')

    def wait_for_no_msg(self, msg, period, callback=None):
        done = False
        t_last = time.time()
        while not done:
            if callback is not None: callback()
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
                if m is None: continue
                if m.get_type() == msg:
                    t_last = time.time()
            if time.time() - t_last > period:
                done = True
            time.sleep(0.001)
 
    def wait_for_msg(self, msg, callback=None):
        done = False
        while not done:
            if callback is not None: callback()
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
                if m is None: continue
                if m.get_type() == msg:
                    done = True
                    break
            time.sleep(0.001)

    def reboot_autopilot(self):
        # must be in hil mode to reboot from auto/ armed
        if not self.hil_enabled(): self.enable_hil()
        # send reboot command 4 times so it gets it
        # consistently
        print 'shutting down ...'
        self.wait_for_no_msg('HEARTBEAT',3, self.master.reboot_autopilot)
        print 'complete'
        # clear master buffer
        self.master.reset()
        print 'waiting for startup ...'
        self.wait_for_msg('HEARTBEAT')
        print 'complete'
        time.sleep(1)
        print 'received heartbeat from sys: ', self.master.target_system
        # Reenable HIL and reset serial (clear buffer)
        if not self.hil_enabled(): self.enable_hil()

    def jsb_set(self, variable, value):
        '''set a JSBSim variable'''
        self.jsb_console.send('set %s %s\r\n' % (variable, value))

    def reset_sim(self):
        self.jsb_set('simulation/reset',1)
        time.sleep(2)
        return time.time()

    def process_jsb_input(self):
        '''process FG FDM input from JSBSim'''
        buf = self.jsb_in.recv(self.fdm.packet_size())
        self.fdm.parse(buf)
        self.ac.update_state(self.fdm)

    @staticmethod
    def interpret_address(addrstr):
        '''interpret a IP:port string'''
        a = addrstr.split(':')
        a[1] = int(a[1])
        return tuple(a)

    def process_master(self):
        m = self.master.recv_msg()
        if m == None: return

        # forward to gcs
        self.gcs.write(m.get_msgbuf())

        # record counts
        if m.get_type() not in self.counts:
            self.counts[m.get_type()] = 0
        self.counts[m.get_type()] += 1

        # handle messages
        mtype = m.get_type()

        if mtype == 'HIL_CONTROLS':
            self.ac.update_controls(m)
            self.ac.send_controls(self.jsb_console)

        elif mtype == 'STATUSTEXT':
            print 'sys %d: %s' % (self.master.target_system, m.text)

        # handle waypoint messages
        self.wpm.process_msg(m)

        # send waypoint messages to mav
        self.wpm.send_messages()

    def process_gcs(self):
        '''process packets from MAVLink slaves, forwarding to the master'''
        try:
            buf = self.gcs.recv()
        except socket.error:
            return
        try:
            if self.gcs.first_byte:
                self.gcs.auto_mavlink_version(buf)
            msgs = self.gcs.mav.parse_buffer(buf)
        except mavutil.mavlink.MAVError as e:
            print "Bad MAVLink gcs message from %s: %s" % (slave.address, e.message)
            return
        if msgs is None:
            return
        for m in msgs:
            self.master.write(m.get_msgbuf())

        if 'Slave' not in self.counts:
            self.counts['Slave'] = 0
        self.counts['Slave'] += 1

    def run(self):
        ''' main execution loop '''

        # start simulation
        self.jsb_console.send('info\n')
        self.jsb_console.send('resume\n')
        self.jsb.expect("trim computation time")
        self.jsb_console.send('hold\n')
        time.sleep(1.5)
        self.jsb_console.logfile = None
        t_hil_state = 0

        print 'Rebooting autopilot'
        self.reboot_autopilot()

        print 'load waypoints'
        if not self.waypoints is None:
            self.wpm.set_waypoints(self.waypoints)
            self.wpm.send_waypoints()
        
        print 'set auto'
        self.master.set_mode_auto()

        # run main loop
        self.jsb_console.send('resume\n')
        while True:

            # make sure hil is enabled
            if not self.hil_enabled(): self.enable_hil()

            # watch files
            rin = [self.jsb_in.fileno(), self.jsb_console.fileno(), self.jsb.fileno(), self.gcs.fd]

            # receive messages on serial port
            while self.master.port.inWaiting() > 0:
                self.process_master()

            try:
                (rin, win, xin) = select.select(rin, [], [], 1.0)
            except select.error:
                util.check_parent()
                continue

            # if new gcs input, process it
            if self.gcs.fd in rin:
                self.process_gcs()

            # if new jsbsim input, process it
            if self.jsb_in.fileno() in rin:
                if self.mode == 'state':
                    if (time.time() - t_hil_state) > 1.0/50:
                        t_hil_state = time.time()
                        self.ac.send_state(self.master.mav)
                elif self.mode == 'sensor':
                    self.ac.send_sensors(self.master.mav)
                self.process_jsb_input()
                # gcs not currently getting HIL_STATE message
                #self.x.send_to_mav(self.gcs.mav)
                self.frame_count += 1

            # show any jsbsim console output
            if self.jsb_console.fileno() in rin:
                util.pexpect_drain(self.jsb_console)
            if self.jsb.fileno() in rin:
                util.pexpect_drain(self.jsb)

            dt_report = time.time() - self.last_report
            if dt_report > 5:
                print '\nmode: {0:X}, JSBSim {1:5.0f} Hz, {2:d} sent, {3:d} received, {4:d} errors, bwin={5:.1f} kB/s, bwout={6:.1f} kB/s'.format(
                    self.master.base_mode,
                    self.frame_count/dt_report,
                    self.master.mav.total_packets_sent,
                    self.master.mav.total_packets_received,
                    self.master.mav.total_receive_errors,
                    0.001*(self.master.mav.total_bytes_received-self.bytes_recv)/dt_report,
                    0.001*(self.master.mav.total_bytes_sent-self.bytes_sent)/dt_report)
                print self.counts
                self.bytes_sent = self.master.mav.total_bytes_sent
                self.bytes_recv = self.master.mav.total_bytes_received
                self.frame_count = 0
                self.last_report = time.time()

if __name__ == "__main__":
    SensorHIL.command_line()
