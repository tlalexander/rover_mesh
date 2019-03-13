import pybullet as p
import time
import pybullet_data
import numpy as np
from PIL import Image
import pickle
import mesh_utils
import math

WIDTH = 640
HEIGHT = 480
DEFAULT_DISTANCE = 5000000

def compute_score(p, vehicle_id, trail_path, trail_distances):
    carpos, carorn = p.getBasePositionAndOrientation(vehicle_id)
    closest_this_pass = DEFAULT_DISTANCE
    for i in range(len(trail_path)):
        point = trail_path[i]
        x_dist = carpos[0] - trail_path[i][0]
        y_dist = carpos[1] - trail_path[i][1]
        dist = math.sqrt(x_dist**2 + y_dist**2)
        if dist < closest_this_pass:
            closest_this_pass = dist
        dist = int(dist**5) # 5th power penalty for distance from trail
        if trail_distances[i] > dist:
            trail_distances[i] = dist
    return trail_distances, closest_this_pass


def getExtendedObservation(p, vehicle_id):
    carpos, carorn = p.getBasePositionAndOrientation(vehicle_id)

    # Using the ZED calculations, I needed to add in rotation too.
    rotate = p.getQuaternionFromEuler([0, 0, np.pi])
    carpos, carorn = p.multiplyTransforms(carpos, carorn, [0, 0, .1], rotate)

    # Most of this is cribbed from racecarZEDGymEnv.py
    carmat = p.getMatrixFromQuaternion(carorn)
    dist0 = 0.3
    dist1 = 1.
    eyePos = [carpos[0]+dist0*carmat[0], carpos[1] +
              dist0*carmat[3], carpos[2]+dist0*carmat[6]]
    targetPos = [carpos[0]+dist1*carmat[0], carpos[1] +
                 dist1*carmat[3], carpos[2]+dist1*carmat[6]]
    up = [carmat[2], carmat[5], carmat[8]]

    # These things taken mostly from testrender_np.py
    pixelWidth = 64
    pixelHeight = 64
    nearPlane = 0.01
    farPlane = 100
    fov = 60

    viewMatrix = p.computeViewMatrix(eyePos, targetPos, up)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov, aspect, nearPlane, farPlane)
    img_arr = p.getCameraImage(
        pixelWidth, pixelHeight, viewMatrix, projectionMatrix, shadow=1,
        lightDirection=[1, 1, 1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return img_arr


physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
# physicsClient = p.connect(p.GUI,options="--opengl2")

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

# Load our list of trail points.
with open(mesh_utils.MESH_DETAILS_NAME, 'rb') as f:
    trail_path = pickle.load(f)

trail_distances = []
for _ in trail_path:
    trail_distances.append(DEFAULT_DISTANCE)

#for i in range(len(trail_path)):
#    trail_path[i] = (trail_path[i][0] - 250) * 0.1, (trail_path[i][1] - 250) * 0.1

#print(trail_path)
start_index = 4

start_pos = trail_path[start_index]
orientation = start_pos[0] - trail_path[start_index+1][0], start_pos[1] - trail_path[start_index+1][1]
orientation = math.atan(orientation[1]/orientation[0])
orientation=p.getQuaternionFromEuler([0, 0, orientation])

robot_id = p.loadURDF("skid_steer_robot.urdf", basePosition=[start_pos[0],start_pos[1],1], baseOrientation=orientation)
world_id = p.loadURDF("generated.urdf")

#for jointnum in range(p.getNumJoints(robot_id)-1):
#  print(p.getJointInfo(robot_id, jointnum))

# frame_id = p.createCollisionShape(
#     shapeType=p.GEOM_BOX, halfExtents=[.2, .4, .1])
#
# suspension_id1 = p.createCollisionShape(shapeType=p.GEOM_BOX,
#                                         halfExtents=[.05, .3, .1])
# suspension_id2 = p.createCollisionShape(shapeType=p.GEOM_BOX,
#                                         halfExtents=[.05, .3, .1])
# suspension_id3 = p.createCollisionShape(shapeType=p.GEOM_BOX,
#                                         halfExtents=[.05, .3, .1])
# suspension_id4 = p.createCollisionShape(shapeType=p.GEOM_BOX,
#                                         halfExtents=[.05, .3, .1])
#
# suspension_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX,
#                                            halfExtents=[.05, .3, .1])
# wheel_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
#                                   radius=0.16, height=0.08)
# wheel_visual_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
#                                       radius=0.16, length=0.08)
#
# basePosition = [0, 0, 2]
# linkMass = 0.1
# linkCollisionShapeIndices=[
#     suspension_id1, suspension_id2, suspension_id3, suspension_id4, wheel_id, wheel_id, wheel_id, wheel_id]

# multi = p.createMultiBody(baseMass=10.0, baseCollisionShapeIndex=frame_id,
#                           basePosition=basePosition, baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi/4]),
#                           linkMasses=[linkMass] * 8,
#                           linkCollisionShapeIndices=linkCollisionShapeIndices,
#                           linkVisualShapeIndices=[suspension_visual_id, suspension_visual_id, suspension_visual_id,
#                                                   suspension_visual_id, wheel_visual_id, wheel_visual_id, wheel_visual_id, wheel_visual_id],
#                           linkPositions=[[0.35, 0.55, -.10], [-0.35, -0.55, -.10], [-0.35, 0.55, -.10], [
#                               0.35, -0.55, -.10], [0.1, -.25, 0], [-0.1, .25, 0], [-0.1, -.25, 0], [0.1, .25, 0]],
#                           linkOrientations=[[1, 0, 0, 0.3], [1, 0, 0, -0.3], [1, 0, 0, 0.3], [1, 0, 0, -0.3], [
#                               0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5]],
#                           linkInertialFramePositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [
#                               0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
#                           linkInertialFrameOrientations=[[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [
#                               1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
#                           linkParentIndices=[
#                               0, 0, 0, 0, suspension_id1, suspension_id2, suspension_id3, suspension_id4],
#                           linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED,
#                                           p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
#                           linkJointAxis=[[0, 1, 0], [0, 1, 0], [0, 1, 0], [
#                               0, 1, 0], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
#                           )


# indicator_visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
#                                           radius=0.5, rgbaColor=[1, 0, 0, 0.75])
#
# indicator_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
# indicator = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=indicator_id,
#                               baseVisualShapeIndex=indicator_visual_id, basePosition=[
#                                   -10,
#                                   0,
#                                   0])

#
# jump_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[2, 2, .01])
# jump_visual_id = p.createVisualShape(
#     shapeType=p.GEOM_BOX, halfExtents=[2, 2, .01])

# for i in range(100):
#     p.stepSimulation()
#
maxForce = 1000
velocity = 5

def set_motor_velocities(p, robot_id, velocity, steering):
    """Sets motor velocities with steering where -1 is left and 1 is right."""
    velocity *= -1
    left_velocity = velocity - velocity/1.2 * steering
    right_velocity = velocity + velocity/1.2 * steering
    p.setJointMotorControlArray(bodyIndex=robot_id,
                            jointIndices=[0,2,1,3],
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocities=[right_velocity,right_velocity,left_velocity,left_velocity],
                            forces=[maxForce,maxForce,maxForce,maxForce])

# Change wheel friction and mass.
friction_val = 0.8
for link_index in [0,1,2,3]:
    p.changeDynamics(bodyUniqueId=robot_id,linkIndex=link_index,lateralFriction=friction_val, spinningFriction=0.0, rollingFriction=0.1, mass=3.0)
    #p.changeDynamics(bodyUniqueId=robot_id,linkIndex=link_index,lateralFriction=friction_val, mass=3.0)
# Change base mass.
p.changeDynamics(bodyUniqueId=robot_id,linkIndex=-1, mass=100.0)

for i in range(1000):
    p.stepSimulation()

time.sleep(5)

for i in range(10000):
    start = time.time()
    p.stepSimulation()
    #if i > 10: time.sleep(1000000)
    getExtendedObservation(p, robot_id)
    duration = time.time() - start
    if duration < 1.0/240.0:
        time.sleep(1./240. - duration)
    trail_distances, current_distance = compute_score(p, robot_id, trail_path, trail_distances)
    steering = 0
    set_motor_velocities(p, robot_id, velocity, steering)
    #print(sum(trail_distances))
    print(current_distance)
    # img = Image.fromarray(ob, 'RGBA')
    # img.save('my.png')
    # img.show()
    # time.sleep(20)


time.sleep(1)
p.disconnect()
