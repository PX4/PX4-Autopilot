from xml.sax.saxutils import escape
import codecs

class RCOutput():
    def __init__(self, groups, board):
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
                    "\n")
        for group in groups:
            result += "# GROUP: %s\n\n" % group.GetName()
            for param in group.GetParams():
                path = param.GetPath().rsplit('/', 1)[1]
                id_val = param.GetId()
                name = param.GetFieldValue("short_desc")
                long_desc = param.GetFieldValue("long_desc")

                result +=   "#\n"
                result +=   "# %s\n" % param.GetName()
                result +=   "if param compare SYS_AUTOSTART %s\n" % id_val
                result +=   "then\n"
                result +=   "\tsh /etc/init.d/%s\n" % path
                result +=   "fi\n"

                #if long_desc is not None:
                #    result += "# %s\n" % long_desc
                result += "\n"

            result += "\n"
        self.output = result;

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
