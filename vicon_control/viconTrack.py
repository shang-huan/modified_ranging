import motioncapture
import argparse
import time


if __name__ == "__main__":
    
    desired_interval = 0.05
    mc = motioncapture.connect("vicon", {"hostname": "172.20.10.7"})
    
    with open("result/log.txt", "w") as f_vicon:
        for i in range(2000):
            start = time.time()
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                positon = obj.position
                rotation = obj.rotation
                print("{},[{:.8f},{:.8f},{:.8f}],{:.8f}".format(name,positon[0], positon[1], positon[2], rotation.z))
                f_vicon.write("{},[{:.8f},{:.8f},{:.8f}],{:.8f}\n".format(name,positon[0], positon[1], positon[2], rotation.z))
            end = time.time()
            if end - start < desired_interval:
                time.sleep(desired_interval - (end - start))

    print("done")