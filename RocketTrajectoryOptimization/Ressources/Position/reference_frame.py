import numpy as np

from RocketTrajectoryOptimization.Ressources.Position.tools import *
from RocketTrajectoryOptimization.Ressources.Position.quaternion import *

class Frame:
    def __init__(self, e1: np.ndarray, e2: np.ndarray, e3: np.ndarray, V_inertial: np.ndarray, quaternion = UnitQuaternion(None, None, [1, [0, 0, 0]])) -> None:
        
        self.G = 9.81

        self.BASIS = np.array([[e1[0], e2[0], e3[0]], [e1[1], e2[1], e3[1]], [e1[2], e2[2], e3[2]]])
        self.BASIS_INV = np.linalg.inv(self.BASIS)
        self.V = {"X": V_inertial[0], "Y": V_inertial[1], "Z": V_inertial[2]}

        if (np.matmul(self.BASIS, self.BASIS_INV) != np.identity(3)).all():
            raise ValueError(f"Basis Vectors {e1}, {e2}, {e3} are not orthonormal")
        elif np.dot(e1, e1) != 1 or np.dot(e2, e2) != 1 or np.dot(e3, e3) != 1:
            raise ValueError(f"Basis Vectors {e1}, {e2}, {e3} don't all have magnitude 1")
        
        self.quaternion = quaternion # make this come from basis vecotrs
        
    def update(self, omega: np.ndarray, dt: float) -> None:
        omega_quaternion = UnitQuaternion(None, None, [0, omega]) # omega is a 
        derivative = self.quaternion*omega_quaternion
        quatenrion_derivative = derivative*0.5
        self.quaternion = self.quaternion + quatenrion_derivative*dt
        self.quaternion.correct_norm()

        self.BASIS = np.array([[1-2*(self.quaternion.q[1]**2 + self.quaternion.q[2]**2), 2*(self.quaternion.q[0]*self.quaternion.q[1]+self.quaternion.q0*self.quaternion.q[2]), 2*(self.quaternion.q[0]*self.quaternion.q[2]+self.quaternion.q0*self.quaternion.q[1])], 
                               [2*(self.quaternion.q[0]*self.quaternion.q[1]-self.quaternion.q0*self.quaternion.q[2]), 1-2*(self.quaternion.q[0]**2+ self.quaternion.q[2]**2), 2*(self.quaternion.q[1]*self.quaternion.q[2]+self.quaternion.q0*self.quaternion.q[0])], 
                               [2*(self.quaternion.q[0]*self.quaternion.q[2]-self.quaternion.q0*self.quaternion.q[1]), 2*(self.quaternion.q[1]*self.quaternion.q[2]-self.quaternion.q0*self.quaternion.q[0]), 1-2*(self.quaternion.q[0]**2+ self.quaternion.q[1]**2)]])

    def __repr__(self) -> str:
        return str(self.BASIS)


class ParentFrame(Frame):
    def __init__(self, e1: np.ndarray, e2: np.ndarray, e3: np.ndarray, V_inertial: np.ndarray) -> None:
        super().__init__(e1, e2, e3, V_inertial)

        self.CHILDREN = []

    def add_child_frame(self, RefFrame: Frame, relative_velocity: np.ndarray, relative_distance: np.ndarray):
        self.CHILDREN.append({"FRAME": RefFrame, "V": relative_velocity, "D": relative_distance, "A": None})

    def update_child(self, child: int, IMU_DATA: dict, dt: float):
        #update velocity and position of frame relative to parent # Do I need other component of velocity like rate of change of rotation
        #need to multiply accels by rotation matrix
        accel_quaternion = UnitQuaternion(None, None, [0, IMU_DATA["ACCELERATION"]])
        conj = self.CHILDREN[child]["FRAME"].quaternion.conjugate()
        relative_accel = (self.CHILDREN[child]["FRAME"].quaternion * accel_quaternion * conj).q

        self.CHILDREN[child]["A"] = relative_accel

        self.CHILDREN[child]["D"][0] += self.CHILDREN[child]["V"][0]*dt + 0.5*relative_accel[0]*dt**2
        self.CHILDREN[child]["D"][1] += self.CHILDREN[child]["V"][1]*dt + 0.5*relative_accel[1]*dt**2
        self.CHILDREN[child]["D"][2] += self.CHILDREN[child]["V"][2]*dt + 0.5*(relative_accel[2] - self.G)*dt**2
        #print(f"Pos: {self.CHILDREN[child]['D']}")
        #print(f"Vel: {self.CHILDREN[child]['V']}")

        self.CHILDREN[child]["V"][0] += + relative_accel[0]*dt
        self.CHILDREN[child]["V"][1] += relative_accel[1]*dt
        self.CHILDREN[child]["V"][2] += (relative_accel[2] - self.G)*dt


        self.CHILDREN[child]["FRAME"].update(IMU_DATA["ANGULAR VELOCITY"], dt) # update frames angles/quatenrion from rates 

    def print_children(self):
        for child in self.CHILDREN:
            print(f"\nFrame: {child['Frame']}\n Relative Velocity: {child['V']} m/s\n Relative Position {child['D']}m\n")

class EarthCenteredInertial(Frame):
    def __init__(self, rokcet_frame: Frame,  rocket_relative_position: np.ndarray):
        e1 = np.array([1, 0, 0])
        e1 = np.array([0, 1, 0])
        e1 = np.array([0, 0, 1])
        V = np.array([0, 0 , 0])
        super().__init__(e1, e2, e3, V)

        self.rocket = {"Frame": rokcet_frame, "D": rocket_relative_position, "V": np.array([0, 0, 0]), "A": np.array([0, 0, 0])}

    def update_rocket(self):
        if np.linalg.norm(self.rocket["V"]) == 0:
            pass



    
    

if __name__ == "__main__":
    e1 = np.array([1, 2, 0])
    e2 = np.array([0, 3, 4])
    e3 = np.array([5, 9, 5])
    V = np.array([5, 9, 5])
    f = Frame(e1, e2, e3)
    print(f)
