import rclpy
from rclpy.node import Node
from angle_interface.srv import Data

class ClienteFK(Node):
    def __init__(self):
        super().__init__('cliente_fk_aibo')
        self.cli = self.create_client(Data, 'Data')

    def call_once(self, i1, i2, i3):
        req = Data.Request()
        req.i1, req.i2, req.i3 = float(i1), float(i2), float(i3)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ClienteFK()

    # Pedir valores al usuario
    t1 = float(input("Introduce θ1 (grados): "))
    t2 = float(input("Introduce θ2 (grados): "))
    t3 = float(input("Introduce θ3 (grados): "))

    resp = node.call_once(t1, t2, t3)
    node.get_logger().info(
        f"θ=[{t1:>4.1f}, {t2:>5.1f}, {t3:>5.1f}]  "
        f"->  p=[{resp.ox:>8.3f}, {resp.oy:>8.3f}, {resp.oz:>8.3f}]"
    )

    rclpy.shutdown()

if __name__ == '__main__':
    main()