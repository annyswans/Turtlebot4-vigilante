import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
import time

class PatrolLoopNode(Node): 
    def __init__(self):
        super().__init__('patrol_loop_node')
        
        # 1. Configuração de Namespace 
        self.namespace = '' 
        self.nav = BasicNavigator(namespace=self.namespace)
        
        # 2. Monitoramento de Bateria
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.namespace}/battery_state',
            self.battery_callback,
            10)
        self.battery_level = 100.0

    def battery_callback(self, msg):
        self.battery_level = msg.percentage * 100

    #criação da pose do robô
    def create_pose(self, x, y, z, w): 
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def run_patrol(self):
        # Aguardando o nav2 carregar
        self.nav.waitUntilNav2Active()

        #RE-LOCALIZAÇÃO INICIAL ---
        # Define a posição inicial (0,0) como a base de saída.
        self.get_logger().info("Definindo pose inicial para re-localização...")
        initial_pose = self.create_pose(0.0, 0.0, 0.0, 1.0)
        self.nav.setInitialPose(initial_pose)
        
        # Pausa para o AMCL processar
        time.sleep(2) 
        # ------------------------------------------------

        # 3. Definição dos Pontos de Patrulha
        waypoints = [
            self.create_pose(1.5, 0.5, 0.0, 1.0),   # Ponto A
            self.create_pose(3.0, -1.0, 0.7, 0.7),  # Ponto B
            self.create_pose(0.0, 0.0, 0.0, 1.0)    # Ponto C (Base)
        ]

        self.get_logger().info("Iniciando Patrulha em Loop...")

        # 4. Verificação de segurança: Bateria
        while rclpy.ok(): 
            for i, pose in enumerate(waypoints):
                if self.battery_level < 15.0:
                    self.get_logger().warn(f"Bateria baixa ({self.battery_level:.1f}%). Indo docar.")
                    self.nav.goToDock()
                    return 

                self.get_logger().info(f"Navegando para o Waypoint {i+1}...")
                self.nav.goToPose(pose)

                while not self.nav.isTaskComplete(): #monitora o robô entre os pontos para verificar a bateria, se ficou preso e a distância até o próximo ponto
                    feedback = self.nav.getFeedback()
                    if feedback:
                        
                        pass

                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Cheguei ao Ponto {i+1}!")
                    time.sleep(2) 
                    
                elif result == TaskResult.CANCELED:
                    self.get_logger().error("Missão cancelada.")
                    return
                else:
                    self.get_logger().error(f"Falha no Ponto {i+1}. Pulando para o próximo.")

def main():
    rclpy.init()
    node = PatrolLoopNode()
    
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info("Patrulha interrompida.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()