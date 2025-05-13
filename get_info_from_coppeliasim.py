import numpy as np

from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ

ci = DQ_CoppeliaSimInterfaceZMQ()
if ci.connect("127.0.0.1"):
    print("Connected")

    np.savetxt("center.txt", ci.get_object_pose("center").q)
    np.savetxt("x_R1.txt", ci.get_object_pose("/LBR4p[0]/joint").q)
    np.savetxt("x_R2.txt", ci.get_object_pose("/LBR4p[1]/joint").q)


