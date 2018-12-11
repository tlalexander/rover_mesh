import pybullet as p
import time
import pybullet_data
import numpy as np
from PIL import Image

WIDTH = 640
HEIGHT = 480


def getExtendedObservation(p, vehicle_id):
    carpos, carorn = p.getBasePositionAndOrientation(vehicle_id)

    # Using the ZED calculations, I needed to add in rotation too.
    rotate = p.getQuaternionFromEuler([0, 0, -np.pi/2])
    carpos, carorn = p.multiplyTransforms(carpos, carorn, [0, 0, .2], rotate)

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
    pixelWidth = 320
    pixelHeight = 200
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

world_id = p.loadURDF("generated.urdf")

frame_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=[.2, .4, .1])

suspension_id1 = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[.05, .3, .1])
suspension_id2 = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[.05, .3, .1])
suspension_id3 = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[.05, .3, .1])
suspension_id4 = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[.05, .3, .1])

suspension_visual_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                           halfExtents=[.05, .3, .1])
wheel_id = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                  radius=0.16, height=0.08)
wheel_visual_id = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                      radius=0.16, length=0.08)

multi = p.createMultiBody(baseMass=10.0, baseCollisionShapeIndex=frame_id,
                          basePosition=[-20, 0, 2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2]),
                          linkMasses=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                          linkCollisionShapeIndices=[
                              suspension_id1, suspension_id2, suspension_id3, suspension_id4, wheel_id, wheel_id, wheel_id, wheel_id],
                          linkVisualShapeIndices=[suspension_visual_id, suspension_visual_id, suspension_visual_id,
                                                  suspension_visual_id, wheel_visual_id, wheel_visual_id, wheel_visual_id, wheel_visual_id],
                          linkPositions=[[0.35, 0.55, -.10], [-0.35, -0.55, -.10], [-0.35, 0.55, -.10], [
                              0.35, -0.55, -.10], [0.1, -.25, 0], [-0.1, .25, 0], [-0.1, -.25, 0], [0.1, .25, 0]],
                          linkOrientations=[[1, 0, 0, 0.3], [1, 0, 0, -0.3], [1, 0, 0, 0.3], [1, 0, 0, -0.3], [
                              0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5]],
                          linkInertialFramePositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [
                              0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
                          linkInertialFrameOrientations=[[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [
                              1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
                          linkParentIndices=[
                              0, 0, 0, 0, suspension_id1, suspension_id2, suspension_id3, suspension_id4],
                          linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED,
                                          p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
                          linkJointAxis=[[0, 1, 0], [0, 1, 0], [0, 1, 0], [
                              0, 1, 0], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
                          )


indicator_visual_id = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                          radius=0.5, rgbaColor=[1, 0, 0, 0.75])

indicator_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
indicator = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=indicator_id,
                              baseVisualShapeIndex=indicator_visual_id, basePosition=[
                                  -10,
                                  0,
                                  0])


jump_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[2, 2, .01])
jump_visual_id = p.createVisualShape(
    shapeType=p.GEOM_BOX, halfExtents=[2, 2, .01])

for i in range(100):
    p.stepSimulation()

maxForce = 10
velocity = 100
p.setJointMotorControl2(bodyUniqueId=multi,
                        jointIndex=1,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=velocity,
                        force=maxForce)

p.setJointMotorControl2(bodyUniqueId=multi,
                        jointIndex=3,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=velocity,
                        force=maxForce)

p.setJointMotorControl2(bodyUniqueId=multi,
                        jointIndex=5,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=velocity,
                        force=maxForce)

p.setJointMotorControl2(bodyUniqueId=multi,
                        jointIndex=7,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=velocity,
                        force=maxForce)


for i in range(10000):
    start = time.time()
    p.stepSimulation()
    getExtendedObservation(p, multi)
    duration = time.time() - start
    if duration < 1.0/240.0:
        time.sleep(1./240. - duration)
    # img = Image.fromarray(ob, 'RGBA')
    # img.save('my.png')
    # img.show()
    # time.sleep(20)


time.sleep(1)
p.disconnect()
