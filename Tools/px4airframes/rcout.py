from xml.sax.saxutils import escape
import codecs
import os

class RCOutput():
    """
        Generates RC scripts for the airframes
    """
    def __init__(self, groups, board, post_start=False):

        result = (  "#\n"
                    "#\n"
                    "#  THIS FILE IS AUTO-GENERATED. DO NOT EDIT!\n"
                    "#\n"
                    "#\n"
                    "# SYS_AUTOSTART = 0 means no autostart (default)\n"
                    "#\n"
                    "# AUTOSTART PARTITION:\n"
                    "#  0    ..   999        Reserved (historical)\n"
                    "#  1000 ..   1999       Simulation setups\n"
                    "#  2000 ..   2999       Standard planes\n"
                    "#  3000 ..   3999       Flying wing\n"
                    "#  4000 ..   4999       Quadrotor x\n"
                    "#  5000 ..   5999       Quadrotor +\n"
                    "#  6000 ..   6999       Hexarotor x\n"
                    "#  7000 ..   7999       Hexarotor +\n"
                    "#  8000 ..   8999       Octorotor x\n"
                    "#  9000 ..   9999       Octorotor +\n"
                    "# 10000 ..  10999       Quadrotor Wide arm / H frame\n"
                    "# 11000 ..  11999       Hexa Cox\n"
                    "# 12000 ..  12999       Octo Cox\n"
                    "# 13000 ..  13999       VTOL\n"
                    "# 14000 ..  14999       Tri Y\n"
                    "# 17000 ..  17999       Autogyro\n"
                    "\n")
        result += "\n"
        result += "set AIRFRAME none\n"
        result += "\n"
        for group in groups:
            result += "# GROUP: %s\n\n" % group.GetType()
            for airframe in group.GetAirframes():
                excluded = False
                for code in airframe.GetArchCodes():
                    if "{0}".format(code) == board and airframe.GetArchValue(code) == "exclude":
                        excluded = True
                if excluded:
                    continue

                if post_start:
                    # Path to post-start script
                    path = airframe.GetPostPath()
                else:
                    # Path to start script
                    path = airframe.GetPath()

                if not path:
                    continue

                path = os.path.split(path)[1]

                id_val = airframe.GetId()
                name = airframe.GetFieldValue("short_desc")
                long_desc = airframe.GetFieldValue("long_desc")

                result +=   "#\n"
                result +=   "# %s\n" % airframe.GetName()
                result +=   "if param compare SYS_AUTOSTART %s\n" % id_val
                result +=   "then\n"
                result +=   "\tset AIRFRAME %s\n" % path
                result +=   "fi\n"

                #if long_desc is not None:
                #    result += "# %s\n" % long_desc
                result += "\n"

            result += "\n"
        result += "\n"
        result += "if [ ${AIRFRAME} != none ]\n"
        result += "then\n"
        result += "\techo \"Loading airframe: /etc/init.d/airframes/${AIRFRAME}\"\n"
        result += "\t. /etc/init.d/airframes/${AIRFRAME}\n"
        result += "fi\n"
        result += "unset AIRFRAME"
        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
