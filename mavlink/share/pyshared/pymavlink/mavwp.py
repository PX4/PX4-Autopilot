'''
module for loading/saving waypoints
'''

import mavutil, time, copy
import logging
import mavutil
try:
    from google.protobuf import text_format
    import mission_pb2
    HAVE_PROTOBUF = True
except ImportError:
    HAVE_PROTOBUF = False


class MAVWPError(Exception):
    '''MAVLink WP error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class MAVWPLoader(object):
    '''MAVLink waypoint loader'''
    def __init__(self, target_system=0, target_component=0):
        self.wpoints = []
        self.target_system = target_system
        self.target_component = target_component
        self.last_change = time.time()

    def count(self):
        '''return number of waypoints'''
        return len(self.wpoints)

    def wp(self, i):
        '''return a waypoint'''
        return self.wpoints[i]

    def add(self, w, comment=''):
        '''add a waypoint'''
	w = copy.copy(w)
	if comment:
		w.comment = comment
        w.seq = self.count()
        self.wpoints.append(w)
        self.last_change = time.time()

    def set(self, w, idx):
        '''set a waypoint'''
        w.seq = idx
        if w.seq == self.count():
            return self.add(w)
        if self.count() <= idx:
            raise MAVWPError('adding waypoint at idx=%u past end of list (count=%u)' % (idx, self.count()))
        self.wpoints[idx] = w
        self.last_change = time.time()

    def remove(self, w):
        '''remove a waypoint'''
        self.wpoints.remove(w)
        self.last_change = time.time()

    def clear(self):
        '''clear waypoint list'''
        self.wpoints = []
        self.last_change = time.time()

    def _read_waypoints_v100(self, file):
        '''read a version 100 waypoint'''
        cmdmap = {
            2 : mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            3 : mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            4 : mavutil.mavlink.MAV_CMD_NAV_LAND,
            24: mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            26: mavutil.mavlink.MAV_CMD_NAV_LAND,
            25: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT ,
            27: mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            }
        comment = ''
        for line in file:
            if line.startswith('#'):
                comment = line[1:].lstrip()
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 13:
                raise MAVWPError("invalid waypoint line with %u values" % len(a))
            if mavutil.mavlink10():
                fn = mavutil.mavlink.MAVLink_mission_item_message
            else:
                fn = mavutil.mavlink.MAVLink_waypoint_message
            w = fn(self.target_system, self.target_component,
                   int(a[0]),    # seq
                   int(a[1]),    # frame
                   int(a[2]),    # action
                   int(a[7]),    # current
                   int(a[12]),   # autocontinue
                   float(a[5]),  # param1,
                   float(a[6]),  # param2,
                   float(a[3]),  # param3
                   float(a[4]),  # param4
                   float(a[9]),  # x, latitude
                   float(a[8]),  # y, longitude
                   float(a[10])  # z
                   )
            if not w.command in cmdmap:
                raise MAVWPError("Unknown v100 waypoint action %u" % w.command)

            w.command = cmdmap[w.command]
            self.add(w, comment)
            comment = ''

    def _read_waypoints_v110(self, file):
        '''read a version 110 waypoint'''
        comment = ''
        for line in file:
            if line.startswith('#'):
                comment = line[1:].lstrip()
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 12:
                raise MAVWPError("invalid waypoint line with %u values" % len(a))
            if mavutil.mavlink10():
                fn = mavutil.mavlink.MAVLink_mission_item_message
            else:
                fn = mavutil.mavlink.MAVLink_waypoint_message
            w = fn(self.target_system, self.target_component,
                   int(a[0]),    # seq
                   int(a[2]),    # frame
                   int(a[3]),    # command
                   int(a[1]),    # current
                   int(a[11]),   # autocontinue
                   float(a[4]),  # param1,
                   float(a[5]),  # param2,
                   float(a[6]),  # param3
                   float(a[7]),  # param4
                   float(a[8]),  # x (latitude)
                   float(a[9]),  # y (longitude)
                   float(a[10])  # z (altitude)
                   )
            if w.command == 0 and w.seq == 0 and self.count() == 0:
                # special handling for Mission Planner created home wp
                w.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.add(w, comment)
            comment = ''

    def _read_waypoints_pb_110(self, file):
        if not HAVE_PROTOBUF:
            raise MAVWPError(
                'Cannot read mission file in protobuf format without protobuf '
                'library. Try "easy_install protobuf".')
        explicit_seq = False
        warned_seq = False
        mission = mission_pb2.Mission()
        text_format.Merge(file.read(), mission)
        defaults = mission_pb2.Waypoint()
        # Set defaults (may be overriden in file).
        defaults.current = False
        defaults.autocontinue = True
        defaults.param1 = 0.0
        defaults.param2 = 0.0
        defaults.param3 = 0.0
        defaults.param4 = 0.0
        defaults.x = 0.0
        defaults.y = 0.0
        defaults.z = 0.0
        # Use defaults specified in mission file, if there are any.
        if mission.defaults:
            defaults.MergeFrom(mission.defaults)
        for seq, waypoint in enumerate(mission.waypoint):
            # Consecutive sequence numbers are automatically assigned
            # UNLESS the mission file specifies sequence numbers of
            # its own.
            if waypoint.seq:
                explicit_seq = True
            else:
                if explicit_seq and not warned_seq:
                    logging.warn(
                            'Waypoint file %s: mixes explicit and implicit '
                            'sequence numbers' % (file,))
                    warned_seq = True
            # The first command has current=True, the rest have current=False.
            if seq > 0:
                current = defaults.current
            else:
                current = True
            w = mavutil.mavlink.MAVLink_mission_item_message(
                self.target_system, self.target_component,
                   waypoint.seq or seq,
                   waypoint.frame,
                   waypoint.command,
                   waypoint.current or current,
                   waypoint.autocontinue or defaults.autocontinue,
                   waypoint.param1 or defaults.param1,
                   waypoint.param2 or defaults.param2,
                   waypoint.param3 or defaults.param3,
                   waypoint.param4 or defaults.param4,
                   waypoint.x or defaults.x,
                   waypoint.y or defaults.y,
                   waypoint.z or defaults.z)
            self.add(w)

    def load(self, filename):
        '''load waypoints from a file.
        returns number of waypoints loaded'''
        f = open(filename, mode='r')
        version_line = f.readline().strip()
        if version_line == "QGC WPL 100":
            readfn = self._read_waypoints_v100
        elif version_line == "QGC WPL 110":
            readfn = self._read_waypoints_v110
        elif version_line == "QGC WPL PB 110":
            readfn = self._read_waypoints_pb_110
        else:
            f.close()
            raise MAVWPError("Unsupported waypoint format '%s'" % version_line)

        self.clear()
        readfn(f)
        f.close()

        return len(self.wpoints)

    def save_as_pb(self, filename):
        mission = mission_pb2.Mission()
        for w in self.wpoints:
            waypoint = mission.waypoint.add()
            waypoint.command = w.command
            waypoint.frame = w.frame
            waypoint.seq = w.seq
            waypoint.current = w.current
            waypoint.autocontinue = w.autocontinue
            waypoint.param1 = w.param1
            waypoint.param2 = w.param2
            waypoint.param3 = w.param3
            waypoint.param4 = w.param4
            waypoint.x = w.x
            waypoint.y = w.y
            waypoint.z = w.z
        with open(filename, 'w') as f:
            f.write('QGC WPL PB 110\n')
            f.write(text_format.MessageToString(mission))

    def save(self, filename):
        '''save waypoints to a file'''
        f = open(filename, mode='w')
        f.write("QGC WPL 110\n")
        for w in self.wpoints:
	    if getattr(w, 'comment', None):
	        f.write("# %s\n" % w.comment)
            f.write("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u\n" % (
                w.seq, w.current, w.frame, w.command,
                w.param1, w.param2, w.param3, w.param4,
                w.x, w.y, w.z, w.autocontinue))
        f.close()

    def polygon(self, done=None):
        '''return a polygon for the waypoints'''
        points = []
        if done is None:
            done = set()
        idx = 0

        # find first point not done yet
        while idx < self.count():
            if not idx in done:
                break
            idx += 1
            
        while idx < self.count():
            w = self.wp(idx)
            if idx in done:
                if w.x != 0 or w.y != 0:
                    points.append((w.x, w.y))
                break
            done.add(idx)
            if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
                idx = int(w.param1)
                w = self.wp(idx)
                if w.x != 0 or w.y != 0:
                    points.append((w.x, w.y))
                continue
            idx += 1
            if (w.x != 0 or w.y != 0) and w.command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                                                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]:
                points.append((w.x, w.y))
        return points

    def polygon_list(self):
        '''return a list of polygons for the waypoints'''
        done = set()
        ret = []
        while len(done) != self.count():
            p = self.polygon(done)
            if len(p) > 0:
                ret.append(p)
        return ret

class MAVFenceError(Exception):
        '''MAVLink fence error class'''
        def __init__(self, msg):
            Exception.__init__(self, msg)
            self.message = msg


class MAVFenceLoader(object):
    '''MAVLink geo-fence loader'''
    def __init__(self, target_system=0, target_component=0):
        self.points = []
        self.target_system = target_system
        self.target_component = target_component
        self.last_change = time.time()

    def count(self):
        '''return number of points'''
        return len(self.points)

    def point(self, i):
        '''return a point'''
        return self.points[i]

    def add(self, p):
        '''add a point'''
        self.points.append(p)
        self.last_change = time.time()

    def clear(self):
        '''clear point list'''
        self.points = []
        self.last_change = time.time()

    def load(self, filename):
        '''load points from a file.
        returns number of points loaded'''
        f = open(filename, mode='r')
        self.clear()
        for line in f:
            if line.startswith('#'):
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 2:
                raise MAVFenceError("invalid fence point line: %s" % line)
            p = mavutil.mavlink.MAVLink_fence_point_message(self.target_system, self.target_component,
                                                            self.count(), 0, float(a[0]), float(a[1]))
            self.add(p)
        f.close()
        for i in range(self.count()):
            self.points[i].count = self.count()
        return len(self.points)

    def save(self, filename):
        '''save fence points to a file'''
        f = open(filename, mode='w')
        for p in self.points:
            f.write("%f\t%f\n" % (p.lat, p.lng))
        f.close()

    def polygon(self):
            '''return a polygon for the fence'''
            points = []
            for fp in self.points[1:]:
                    points.append((fp.lat, fp.lng))
            return points
