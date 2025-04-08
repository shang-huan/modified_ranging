import motioncapture
import argparse
import time

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
    
f_console = None

def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def log_console_callback(text):
    print(text,end="")
    if(f_console != None):
        f_console.write(text)

if __name__ == "__main__":

    # parser = argparse.ArgumentParser()
    # parser.add_argument("type")
    # parser.add_argument("hostname")
    # args = parser.parse_args()
    
    # vicon
    # 172.20.10.7
    cflib.crtp.init_drivers()
    
    # uri = 'radio://0/80/2M/E7E7E7E783'
    uri = 'radio://0/80/2M/E7E7E7E757'
    
    groupstr = "vicon"
    namestr = "viconXYZ"
    param_name_XYZ = "vicon.viconXYZ"
    hostname = "172.20.10.7"
    
    desired_interval = 0.05
    mc = motioncapture.connect("vicon", {"hostname": hostname})
    print("Connected to Vicon {}".format(hostname))
    
    f_console = open("result/console_log.txt", "w")
    try:
        with open("result/log.txt", "w") as f_vicon:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                # scf.cf.param.add_update_callback(group=groupstr, name=namestr,
                #                            cb=param_stab_est_callback)
                print("Connected to {}".format(scf.cf.link_uri))
                scf.cf.console.receivedChar.add_callback(log_console_callback)
                for i in range(2000):
                    start = time.time()
                    mc.waitForNextFrame()
                    for name, obj in mc.rigidBodies.items():
                        positon = obj.position
                        rotation = obj.rotation
                        # print("{},[{:.8f},{:.8f},{:.8f}],{:.8f}".format(name,positon[0], positon[1], positon[2], rotation.z))
                        x = (int)(positon[0]*1000)
                        y = (int)(positon[1]*1000)
                        z = (int)(positon[2]*1000)
                        xyz = (x<<32 | y<<16 | z)
                        print(x, y, z, "=>", xyz)
                        scf.cf.param.set_value(param_name_XYZ, xyz)
                        f_vicon.write("{},[{:.8f},{:.8f},{:.8f}],{:.8f}\n".format(name,positon[0], positon[1], positon[2], rotation.z))
                    end = time.time()
                    if end - start < desired_interval:
                        time.sleep(desired_interval - (end - start))
    finally:
        f_console.close()

    print("done")