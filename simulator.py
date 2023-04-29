import pybullet as p
import pybullet_data
import time
import numpy as np

# Set up the physics simulation
p.connect(p.GUI) # or p.DIRECT for headless mode
p.setGravity(0, 0, -9.81) # set gravity in z direction
p.setTimeStep(1./240.) # set simulation time step
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add data path for built-in assets

# Load the pedestrian model
# startPos = [0,0,0.9]
startOrientation = p.getQuaternionFromEuler([np.pi/2,0,0])
# pedestrianId = p.loadURDF("humanoid/humanoid.urdf", startPos, startOrientation, globalScaling=0.25) # create a pedestrian at (0, 0, 0.5)

# Create a plaza with random static obstacles
plazaId = p.loadURDF("plane.urdf", [0, 0, 0])
num_obstacles = 2
obstacleIds = []
for i in range(num_obstacles):
    x = np.random.uniform(low=-50, high=50)
    y = np.random.uniform(low=-50, high=50)
    obstacleIds.append(p.loadURDF("cube.urdf", [x, y, 1], globalScaling=2))


# Simulate the crowd of dynamic cubes moving in the plaza
num_dynamic = 2
dynamicIds = []
for i in range(num_dynamic):
    x = np.random.uniform(low=-50, high=50)
    y = np.random.uniform(low=-50, high=50)
    dynamicId = p.loadURDF("sphere2.urdf", [x, y, 1], startOrientation, globalScaling=1.5)
    # Set the velocity
    print(p.getBaseVelocity(dynamicId))
    p.resetBaseVelocity(dynamicId, [np.random.uniform(-10, 10), np.random.uniform(-10, 10), 0], [0, 0, 0])
    dynamicIds.append(dynamicId)


for i in range(2400): # simulate for 2400 time steps
    # for j in range(num_dynamic):
    #     pos, orientation = p.getBasePositionAndOrientation(dynamicIds[j])
    #     newPos = [pos[0] + 0.5, pos[1] + 0.5, pos[2]]
    #     p.resetBasePositionAndOrientation(dynamicIds[j], newPos, orientation)

    p.stepSimulation() # simulate one time step
    time.sleep(1./240.) # wait for a short time
