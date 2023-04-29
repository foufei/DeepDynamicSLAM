import pybullet as p
import pybullet_data
import time
import numpy as np

class Simulator:
    def __init__(self):
        # TODO: load the agent model
        # self.agent = p.loadURDF("humanoid/humanoid.urdf", [0,0,0.9], p.getQuaternionFromEuler([np.pi/2,0,0]), globalScaling=0.25)
        self.obstacles = []
        self.dynamics = []

        self.initialize_env()

    def initialize_env(self, num_obstacles=2, num_dynamics=2, dynamic_velocity=10, distance=50):
        # Set up the physics simulation
        p.connect(p.GUI) # or p.DIRECT for headless mode
        p.setGravity(0, 0, -9.81) # set gravity in z direction
        p.setTimeStep(1./240.) # set simulation time step
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add data path for built-in assets

        # Create a plaza with random static obstacles
        plazaId = p.loadURDF("plane.urdf", [0, 0, 0])

        # Simulate the static cubes in the plaza as the obstacles
        for _ in range(num_obstacles):
            x = np.random.uniform(low=-distance, high=distance)
            y = np.random.uniform(low=-distance, high=distance)
            self.obstacles.append(p.loadURDF("cube.urdf", [x, y, 1], globalScaling=2))

        # Simulate the crowd of dynamic cubes moving in the plaza
        for _ in range(num_dynamics):
            # Set the initial position
            x = np.random.uniform(low=-distance, high=distance)
            y = np.random.uniform(low=-distance, high=distance)
            dynamic = p.loadURDF("sphere2.urdf", [x, y, 1], p.getQuaternionFromEuler([0,0,0]), globalScaling=1.5)

            # Set the velocity
            v_x = np.random.uniform(-dynamic_velocity, dynamic_velocity)
            v_y = np.random.uniform(-dynamic_velocity, dynamic_velocity)
            p.resetBaseVelocity(dynamic, [v_x, v_y, 0], [0, 0, 0])

            self.dynamics.append(dynamic)

    def run(self, num_timesteps = 2400):
        for i in range(num_timesteps): # simulate for 2400 time steps
            p.stepSimulation() # simulate one time step
            time.sleep(1./240.) # wait for a short time
