"""Generate a new mesh world and launch it."""

import pybullet as p
import pybullet_data
import random
import time

import mesh_utils


random.seed()
rand_seed = int(random.random() * 10000)
print("Initializing noise with seed %d" % rand_seed)

mesh = mesh_utils.Mesh(rand_seed)

print("Generating environment.")
mesh.generate(2)
mesh.clip_mesh(min=-1.5, max=10)
mesh.calculate_min_max()
mesh.init_texture()


print("Finding Goal")
start = (7 * int(mesh.width_steps/8), 7 * int(mesh.depth_steps/8))
goal = (int(mesh.width_steps/8), int(mesh.depth_steps/8))
print("Start: %r, %r" % start)
print("Goal: %r, %r" % goal)

mesh.find_path_costs(start, goal)

print("Reconstructing Path")
mesh.reconstruct_path()

print("Flattening and Texturing Path of length {}".format(len(mesh.path)))

mesh.build_path()
# mesh.visualize_search()
mesh.save_mesh()
mesh.save_texture()
mesh.save_mesh_details()


#########################################################
#  ____  _____ _   _ ____  _____ ____  ___ _   _  ____  #
# |  _ \| ____| \ | |  _ \| ____|  _ \|_ _| \ | |/ ___| #
# | |_) |  _| |  \| | | | |  _| | |_) || ||  \| | |  _  #
# |  _ <| |___| |\  | |_| | |___|  _ < | || |\  | |_| | #
# |_| \_\_____|_| \_|____/|_____|_| \_\___|_| \_|\____| #
#########################################################
physicsClient = p.connect(p.GUI)
# physicsClient = p.connect(p.GUI, options="--opengl2")

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

world_id = p.loadURDF("generated.urdf")

for i in range(1000000):
    p.stepSimulation()
    time.sleep(1./240.)


time.sleep(1)
p.disconnect()
