import numpy as np

class UnitQuaternion:
    def __init__(self, rotation_axis: np.ndarray|list|None, angle: float|None, quaternion: list|None = None) -> None:
        '''Rotation_axis: expected 3 by 1 vecoor
            angle: rotation angle in radians
        '''
        if quaternion is None:
            self.q0 = np.cos(angle/2)
            self.q = np.sin(angle/2)*rotation_axis
        else:
            self.q0 = quaternion[0]
            self.q = np.array(quaternion[1])
    
    def __mul__(self, p) -> 'UnitQuaternion':
        q = []
        if isinstance(p, UnitQuaternion):
            q.append(self.q0*p.q0 - (self.q[0]*p.q[0] + self.q[1]*p.q[1] + self.q[2]*p.q[2]))
            q.append(self.q0*p.q + p.q0*self.q + np.cross(self.q, p.q))
            return UnitQuaternion(None, None, q)
        elif isinstance(p, float) or isinstance(p, int):
            q.append(self.q0 * p)
            q.append(self.q * p)
            return UnitQuaternion(None, None, q)
        else:
            raise NotImplementedError(f"Quaternion multiplication not defined between quaternion and {type(p)}")

    def __div__(self, d):
        q= []
        if isinstance(d, float) or isinstance(d, int):
            q.append(self.q0 / d)
            q.append(self.q / d)
            return UnitQuaternion(None, None, q)
    
    def __add__(self, a) -> 'UnitQuaternion':
        q = []
        if type(a) == UnitQuaternion:
            q.append(self.q0 + a.q0)
            q.append([self.q[0] + a.q[0], self.q[1] + a.q[1], self.q[2]+a.q[2]])
            return UnitQuaternion(None, None, q)
        else:
            raise NotImplementedError(f"Quaternion addition not defined between quaternion and {type(a)}")

    def norm(self) -> float:
        return self.q0*self.q0 + np.dot(self.q, self.q)
    
    def correct_norm(self):
        n = np.sqrt(self.norm())
        if n == 0:
            return
        self.q0 = np.divide(self.q0, n)
        self.q = np.divide(self.q, n)

    # def derivative(self, omega: np.ndarray) -> 'UnitQuaternion':
    #     omega_quaternion = UnitQuaternion(None, None, [0, omega])
    #     print(omega_quaternion)
    #     return (self.q*omega_quaternion)/2

    def conjugate(self):
        return UnitQuaternion(None, None, [self.q0, -1*self.q])

    def __repr__(self) -> str:
        return f"{self.q0} + {self.q[0]}i, {self.q[1]}j + {self.q[2]}k"
