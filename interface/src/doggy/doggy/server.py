import numpy as np
import rclpy
from rclpy.node import Node
from angle_interface.srv import Data

# para el codigo del server se debe compilar antes que el client
# para ejecutar el codigo se emplea el siguiente comando:
# ros2 run doggy server

L1 = 69.5
L2 = 71.5

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],
                     [0,c,-s,0],
                     [0,s, c,0],
                     [0,0,0,1]], dtype=float)

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c,0, s,0],
                     [ 0,1, 0,0],
                     [-s,0, c,0],
                     [ 0,0, 0,1]], dtype=float)

def Tz(d):
    return np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,d],
                     [0,0,0,1]], dtype=float)

class ServidorFK(Node):
    def __init__(self):
        super().__init__('servidor_fk_aibo')
        self.srv = self.create_service(Data, 'Data', self.compute_fk)
        self.get_logger().info('Servicio /Data listo con NumPy.')

    def compute_fk(self, req, resp):
        i1, i2, i3 = np.radians([req.i1, req.i2, req.i3])

        M = Rx(i1) @ Ry(i2) @ Tz(-L1) @ Ry(i3) @ Tz(-L2)

        resp.ox, resp.oy, resp.oz = M[0,3], M[1,3], M[2,3]

        self.get_logger().info(f"θ=[{req.i1:.1f}, {req.i2:.1f}, {req.i3:.1f}]  -> "
                               f"p=[{resp.ox:.2f}, {resp.oy:.2f}, {resp.oz:.2f}] mm")
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = ServidorFK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()