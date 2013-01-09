#!/usr/bin/env python

'''
check bandwidth of link
'''

# system import
import sys, struct, time, os, argparse, signal, math
import pexpect, socket, fdpexpect, select
import pymavlink.mavutil as mavutil
import pymavlink.mavwp as mavwp

# set path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'jsbsim'))

# local imports
import util, atexit
import pymavlink.fgFDM as fgFDM

from math import sin, cos

class AircraftControls(object):
    
    def __init__(self, aileron, elevator, rudder, throttle, aux1, aux2, aux3, aux4,
                mode, nav_mode):
        self.aileron = aileron
        self.elevator = elevator
        self.rudder = rudder
        self.throttle = throttle
        self.aux1 = aux1
        self.aux2 = aux2
        self.aux3 = aux3
        self.aux4 = aux4
        self.mode = mode
        self.nav_mode = nav_mode

    def send_to_jsbsim(self, jsb_console):
        jsb_console.send('set %s %s\r\n' % ('fcs/aileron-cmd-norm', self.aileron))
        jsb_console.send('set %s %s\r\n' % ('fcs/elevator-cmd-norm', self.elevator))
        jsb_console.send('set %s %s\r\n' % ('fcs/rudder-cmd-norm', self.rudder))
        jsb_console.send('set %s %s\r\n' % ('fcs/throttle-cmd-norm', self.throttle))

    @classmethod
    def from_mavlink(cls,msg):
        return cls(
            aileron = msg.roll_ailerons,
            elevator = msg.pitch_elevator,
            rudder = msg.yaw_rudder,
            throttle = msg.throttle,
            aux1 = msg.aux1,
            aux2 = msg.aux2,
            aux3 = msg.aux3,
            aux4 = msg.aux4,
            mode = msg.mode,
            nav_mode = msg.nav_mode)

class AircraftState(object):
    
    def __init__(self, time,
                 phi, theta, psi,
                 p, q, r,
                 lat, lon, alt,
                 vN, vE, vD,
                 xacc, yacc, zacc):
        self.time = time
        self.phi = phi
        self.theta = theta
        self.psi = psi
        self.p = p
        self.q = q
        self.r = r
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vN = vN
        self.vE = vE
        self.vD = vD
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc

    def send_to_mav(self, mav):
        g = 9.80665
        mpss2mg = 1000.0/g
        m2cm = 100.0
        m2mm = 1000.0
        rad2degE7 = 1.0e7*180.0/math.pi
        mav.hil_state_send(
            self.time, self.phi, self.theta, self.psi,
            self.p, self.q, self.r,
            int(self.lat*rad2degE7), int(self.lon*rad2degE7), int(self.alt*m2mm),
            int(self.vN*m2cm), int(self.vE*m2cm), int(self.vD*m2cm),
            int(self.xacc*mpss2mg), int(self.yacc*mpss2mg), int(self.zacc*mpss2mg))

    @classmethod
    def from_fdm(cls, fdm):
        # position
        lat = fdm.get('latitude', units='radians')
        lon = fdm.get('longitude', units='radians')
        alt = fdm.get('altitude', units='meters')

        # attitude
        phi = fdm.get('phi', units='radians')
        theta = fdm.get('theta', units='radians')
        psi = fdm.get('psi', units='radians')

        # rotation rates
        phidot = fdm.get('phidot', units='rps')
        thetadot = fdm.get('thetadot', units='rps')
        psidot = fdm.get('psidot', units='rps')

        p = phidot - psidot*sin(theta)
        q = cos(phi)*thetadot + sin(phi)*cos(theta)*psidot
        r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot

        # acceleration
        xacc = fdm.get('A_X_pilot', units='mpss')
        yacc = fdm.get('A_Y_pilot', units='mpss')
        zacc = fdm.get('A_Z_pilot', units='mpss')

        # velocitiy
        vN = fdm.get('v_north', units='mps')
        vE = fdm.get('v_east', units='mps')
        vD = fdm.get('v_down', units='mps')

        return cls(time=time.time(),
                             phi=phi, theta=theta, psi=psi,
                             p=p, q=q, r=r,
                             lat=lat, lon=lon, alt=alt,
                             vN=vN, vE=vE, vD=vD,
                             xacc=xacc, yacc=yacc, zacc=zacc)
        
class SensorHIL(object):
    ''' This class executes sensor level hil communication '''

    @classmethod
    def command_line(cls):
        ''' command line parser '''
        parser = argparse.ArgumentParser()
        parser.add_argument('--master', help='device', default=None)
        parser.add_argument('--baud', help='master port baud rate', default=921600)
        parser.add_argument('--script', help='jsbsim script', default='jsbsim/easystar_test.xml')
        parser.add_argument('--options', help='jsbsim options', default=None)
        parser.add_argument('--gcs', help='gcs host', default='localhost:14550')
        parser.add_argument('--waypoints', help='waypoints file', default=None)
        args = parser.parse_args()
        if args.master is None:
            raise IOError('must specify device with --dev')
        inst = cls(master_dev=args.master, baudrate=args.baud, script=args.script, options=args.options, gcs_dev=args.gcs, waypoints=args.waypoints)
        inst.run()

    def __init__(self, master_dev, baudrate, script, options, gcs_dev, waypoints):
        ''' default ctor 
        @param dev device
        @param baud baudrate
        '''
        self.script = script
        self.options = options
        self.waypoints = waypoints
        self.x = AircraftState(time=time.time(),
                               phi=0, theta=0, psi=0,
                               lat=0, lon=0, alt=0,
                               vN=0, vE=0, vD=0,
                               p=0, q=0, r=0,
                               xacc=0, yacc=0, zacc=0)
        self.u = AircraftControls(aileron=0, elevator=0, rudder=0, throttle=0,
                                  aux1=0, aux2=0, aux3=0, aux4=0, mode=0, nav_mode=0)
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
        cmd = "JSBSim --realtime --suspend --nice --simulation-rate=1000 --logdirectivefile=jsbsim/fgout.xml --script=%s" % self.script
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

        # waypoint setting variables
        self.wploading = False
        self.wpload = None
        self.wpload_time = 0

    def hil_enabled(self):
        return self.master.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED

    def enable_hil(self):
        ''' enable hil mode '''
        while not self.hil_enabled():
            self.master.set_mode_flag(mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,True)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()

    def reboot_autopilot(self):
        self.master.reboot_autopilot()

    def set_waypoints(self, waypoints):
        self.wpload_time = time.time()
        self.wploading = True
        self.wpload = mavwp.MAVWPLoader()

        self.wpload.target_system = 1
        self.wpload.taget_component = 1
        wpcount = self.wpload.load(waypoints)
        print 'Waypoint Count = %i' % wpcount

        #self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wpcount)
        #self.read_waypoint_request()
        
    def process_waypoint_request(self, m):
        print 'read request'
        if time.time()-self.wpload_time > 10:
            self.wploading = False

        if self.wploading:
            num = m.get_seq()
            print m.get_seq()
            print m.get_payload()
            wp = self.wpload.wp(num)
            wp.target_system = self.wpload.target_system
            wp.target_component = self.wpload.target_component
            self.master.mav.send(wp)
            self.wpload_time = time.time()
            print 'Sent waypoint %u: %s' % (num, wp)
            if num == self.wploader.count() - 1:
                self.wploading = False

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
        self.x = AircraftState.from_fdm(self.fdm)

    @staticmethod
    def interpret_address(addrstr):
        '''interpret a IP:port string'''
        a = addrstr.split(':')
        a[1] = int(a[1])
        return tuple(a)

    def process_master(self):
        m = self.master.recv_msg()
        if m == None: return

        self.gcs.write(m.get_msgbuf())

        if m.get_type() not in self.counts:
            self.counts[m.get_type()] = 0
        self.counts[m.get_type()] += 1

        mtype = m.get_type()
        # hil control message
        if mtype == 'HIL_CONTROLS':
            self.u = AircraftControls.from_mavlink(m)
            self.u.send_to_jsbsim(self.jsb_console)
        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
            self.process_waypoint_request(m)

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
        time.sleep(1.5)
        self.jsb_console.logfile = None

        
        #i = 0
        #while True:
        #    m = self.master.recv_msg()
        #    if m == None: continue

        #    i += 1
        #    print m.get_type()
        #    if i > 100:
        #        break

        #time.sleep(10)
        #print 'load waypoints'
        #self.set_waypoints(self.waypoints)
        #self.process_waypoint_request()

        # run main loop
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

            tnow = time.time()

            # if new gcs input, process it
            if self.gcs.fd in rin:
                self.process_gcs()

            # if new jsbsim input, process it
            if self.jsb_in.fileno() in rin:
                self.process_jsb_input()
                self.x.send_to_mav(self.master.mav)
                # gcs not currently getting HIL_STATE message
                #self.x.send_to_mav(self.gcs.mav)
                self.frame_count += 1

            # show any jsbsim console output
            if self.jsb_console.fileno() in rin:
                util.pexpect_drain(self.jsb_console)
            if self.jsb.fileno() in rin:
                util.pexpect_drain(self.jsb)

            # periodic output
            dt_report = tnow - self.last_report
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
