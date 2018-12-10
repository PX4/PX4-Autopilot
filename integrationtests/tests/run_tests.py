#!/usr/bin/env python

import multiprocessing as mp
import os
import subprocess
import signal
import sys
import argparse
import yaml

parser = argparse.ArgumentParser(description='Runs maneuver, simulation and tests based on yaml file')
parser.add_argument('px4dir', help='PX4 src directory')

class ProcessWrapper(mp.Process):
    def __init__(self, cmds, env=None):
        mp.Process.__init__(self)
        self.exit = mp.Event()
        self.cmds = cmds
        self.env = env

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        p = subprocess.Popen(self.cmds, env=self.env)
        while (not self.exit.is_set()) and (p.poll() is None):
            pass
        p.terminate()

def handle_ctrC(*args):
    p_maneuver.exit.set()
    p_server.exit.set()
    p_gui.exit.set()
    p_px4.exit.set()
    if p_gui.is_alive():
        p_gui.join()
    p_maneuver.join()
    p_server.join()
    p_px4.join()
    print("Closed all processes")
    sys.exit(0)

if __name__ == '__main__':

    ##############
    ### Simulation
    ##############
    args = parser.parse_args()

    # fork method
    mp.set_start_method('fork')

    # read yaml
    dir_path = os.path.dirname(__file__)
    yaml_file = open(dir_path + "/test_config.yml", "r")
    config = yaml.load(yaml_file)

    deactivate_tests = config['deactivate_tests']
    simulation_setup = config['simulation_setup']

    maneuver = simulation_setup['maneuver']
    world = simulation_setup['world']
    sim = simulation_setup['sim']
    headless = simulation_setup['headless']
    start_script = simulation_setup['start_script']

    # path to px4-src directory
    px4_src_dir = args.px4dir

    # define processes
    global p_maneuver, p_server, p_gui, p_px4
    cmds = ['{0}/Tools/Maneuvers/build/maneuvers/{1}'.format(px4_src_dir, maneuver), 'udp://:14540']
    p_maneuver = ProcessWrapper(cmds)

    cmds = ['gzserver', '{0}/Tools/sitl_gazebo/worlds/{1}.world'.format(px4_src_dir, world)]
    p_server = ProcessWrapper(cmds)

    cmds = ['gzclient']
    p_gui = ProcessWrapper(cmds)

    cmds = ['{0}/build/px4_sitl_default/bin/px4'.format(px4_src_dir), '{0}/ROMFS/px4fmu_common'.format(px4_src_dir), '-s', 'etc/init.d-posix/{0}'.format(start_script)]
    env = os.environ.copy()
    env['PX4_SIM_MODEL'] = world
    p_px4 = ProcessWrapper(cmds, env)

    # start processes
    p_maneuver.start()
    p_server.start()
    if not headless:
        p_gui.start()
    p_px4.start()

    # catch ctrl-c
    signal.signal(signal.SIGINT, handle_ctrC)

    # Close processes if:
    # 1. Simulation server is not running anymore
    # 2. maneuver closed susccessfully
    # 3. timeout

    # Simulation closed before maneuver finished
    while p_maneuver.is_alive():

        # server or px4 not running anymore: close eveything
        if not p_server.is_alive(): #or not p_px4.is_alive():
            p_maneuver.exit.set()

    # maneuver is not running anymore: close everything
    p_server.exit.set()
    p_gui.exit.set()
    p_maneuver.exit.set()
    p_px4.exit.set()
    p_maneuver.join()
    print("Maneuver closed")
    p_server.join()
    if p_gui.is_alive():
        p_gui.join()
    print("Simulation closed")
    p_px4.join()
    print("PX4 closed")

    ################
    ### log-Analysis
    ################
    logdir= os.getcwd()+'/log/'
    newest_dir= logdir + max(os.listdir(logdir), key=lambda x: os.stat(logdir+x).st_mtime) + '/'
    newest_log=newest_dir + max(os.listdir(newest_dir), key=lambda x: os.stat(newest_dir+x).st_mtime)

    # run all general tests except for those that are deselected in the yaml file
    if deactivate_tests is not None:
        deselected_tests = '-k'
        for index, test in enumerate(deactivate_tests):
            if index == 0:
                deselected_tests = deselected_tests + ' not ' + test
            else:
                deselected_tests = deselected_tests + ' and not ' + test 
    else:
        deselected_tests = ''

    testfile = px4_src_dir+'/Tools/ulogtests/tests/test_general.py'
    p_test = subprocess.Popen(['py.test', '-s', deselected_tests, testfile, '--filepath={0}'.format(newest_log)])
    while p_test.poll() is None:
        pass
    p_test.terminate()
    print("test finished")
