#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# * Neither the name of the Player Project nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE 4 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Library packages needed
from threading import Lock
import numpy as np

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

# ROS TF-related
from geometry_msgs.msg import PoseWithCovarianceStamped, Point

# Our functions
import ar_py_utils.utils as utils
from markers_msgs.msg import Markers



#importar a lista das localizações dos Becons

from lw1.lw1_landmarks_locations import beacons_wpos





class Trilateration(Node):
    '''
    Basic robot localization using trilateration.
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''
        self.lock = Lock()

        # Internal pose-related variables
        self.robot_real_pose = Pose2D()  # Store real (error-free) robot pose
        self.robot_estimated_pose = Pose2D()  # Store estimated robot pose
        self.real_pose_updated = False  # True if the real pose was updated

        # World reference frame
        self.base_frame_id = 'map'

        # Initialize the node itself
        super().__init__('lw1_localization')

        # Setup markers subscriber
        self.create_subscription(Markers, 'markers', self.markers_callback, 1)

        # Setup ground-truth subscriber
        self.create_subscription(Odometry,
                                 'base_pose_ground_truth',
                                 self.pose_gnd_truth_callback, 1)

        # Setup pose publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              'pose', 1)

    def markers_callback(self, msg: Markers):
        ''' Process received markers data '''

        self.get_logger().info(
            f'Got {msg.num_markers} beacons at time ' +
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec')

        # Display found beacons information
        for i in range(msg.num_markers):
            self.get_logger().info(
                f'Beacon {msg.id[i]:d} : (range, bearing) = (' +
                f'{msg.range[i]:.2f} m, {msg.bearing[i]:.2f} °)\n' +
                '---')

        ###
        # CHANGE THE CODE BELOW
        ###

        # A matrix, 3 x 2
       # A = np.zeros((3, 2), float)

        # b matrix, 4 x 1 (columm vector)
       # b = np.zeros((4, 1), float)

        # Store some test values in A
       # A[0, 0] = 1
       # A[0, 1] = 2
       # A[1, 0] = -1
       # A[1, 1] = 2
       # A[2, 0] = 1
       # A[2, 1] = -2
        # Add some values to A using the "append" instruction.
        # A becomes (4 x 2) has expected
       # A = np.append(A, [[2., 2.5]], axis=0)

        # Store some test values in b
       # b[0] = 1
       # b[1] = 2
       # b[2] = -1
       # b[3] = 1.5

        # Compute r = inv(A'*A)*A'*b
       # r = np.linalg.pinv(A) @ b


# Código de calculo das matrizes e posição x e y:
        n = msg.num_markers

        if n == 2:
            # temos apenas 2 becons detetados existem 2 soluções posiveis
            
            #dados dos dois beacons
            x1= beacons_wpos[msg.id[0]-1].x 
            y1 = beacons_wpos[msg.id[0]-1].y
            d1 = msg.range[0]
            x2 = beacons_wpos[msg.id[1]-1].x 
            y2 = beacons_wpos[msg.id[1]-1].y
            d2 = msg.range[1]



        #calculo dos dois pontos de interceção

            # Distância entre os dois beacons
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            # Distância do beacon 1 ao ponto médio entre as interseções
            a = (d1**2 - d2**2 + dist**2) / (2 * dist)

            # Altura do triângulo (distância do ponto médio aos pontos de interseção)
            h = np.sqrt(max(0.0, d1**2 - a**2))

            # Ponto médio entre as duas interseções
            mx = x1 + a * (x2 - x1) / dist
            my = y1 + a * (y2 - y1) / dist

            # Os dois pontos de interseção (perpendiculares ao eixo beacon1-beacon2)
            px1 = mx + h * (y2 - y1) / dist
            py1 = my - h * (x2 - x1) / dist

            px2 = mx - h * (y2 - y1) / dist
            py2 = my + h * (x2 - x1) / dist


        # Escolhe o ponto mais próximo da última estimativa conhecida
            last_x = self.robot_estimated_pose.x
            last_y = self.robot_estimated_pose.y

            dist1 = (px1 - last_x)**2 + (py1 - last_y)**2
            dist2 = (px2 - last_x)**2 + (py2 - last_y)**2

            if dist1 <= dist2:
                est_x, est_y = px1,py1
            else:
                est_x, est_y = px2,py2




            #Orientação (código igual ao de baixo):
            sum_sin = 0.0
            sum_cos = 0.0
            for i in range(2):
                xi = beacons_wpos[msg.id[i]-1].x
                yi = beacons_wpos[msg.id[i]-1].y
                bearing_i = msg.bearing[i]
                angle_to_beacon = np.arctan2(yi - est_y, xi - est_x)
                theta_i = angle_to_beacon - bearing_i
                sum_sin += np.sin(theta_i)
                sum_cos += np.cos(theta_i)
            est_theta = np.arctan2(sum_sin / 2, sum_cos / 2)

            # Debug para o caso de 2 beacons
            print(f'Caso 2 beacons:')
            print(f'  Ponto 1: ({px1:.3f}, {py1:.3f})')
            print(f'  Ponto 2: ({px2:.3f}, {py2:.3f})')
            print(f'  Escolhido: ({est_x:.3f}, {est_y:.3f})')


        else:
            #calculo da posição
            # Marco de referência (primeiro detetado)
            x1, y1 = beacons_wpos[msg.id[0]-1].x, beacons_wpos[msg.id[0]-1].y
            d1 = msg.range[0]

            A = np.empty((0, 2), float)
            b = np.empty((0, 1), float)

            #variaveis para media do angulo
            sum_sin = 0.0
            sum_cos = 0.0


            for i in range(1, n):
                # Dados do marco atual
                xi = beacons_wpos[msg.id[i]-1].x
                yi = beacons_wpos[msg.id[i]-1].y
                di = msg.range[i]
                bearing_i = np.deg2rad(msg.bearing[i]) # Sensor dá em graus, converter para radianos

                A = np.append(A, [[x1 - xi, y1 - yi]], axis=0)
                bi = 0.5 * (x1**2 + y1**2 - xi**2 - yi**2 + di**2 - d1**2)
                b = np.append(b, [[bi]], axis=0)

            # Verifica que A não é degenerada
            if np.all(np.abs(A[:, 0]) < 1e-6) or np.all(np.abs(A[:, 1]) < 1e-6):
                est_x = self.robot_estimated_pose.x
                est_y = self.robot_estimated_pose.y
            else:
                r = np.linalg.pinv(A) @ b
                est_x = float(r[0, 0])
                est_y = float(r[1, 0])




            #calculo da orientação teta
            for i in range(n):
                xi = beacons_wpos[msg.id[i]-1].x
                yi = beacons_wpos[msg.id[i]-1].y
                bearing_i = msg.bearing[i]

                # Ângulo global do robô para o marco
                #já se calculou posição e sabe-se a posição do beacon
                #usa-se arctan para calcular qual o angulo dessa direção
                angle_to_beacon = np.arctan2(yi - est_y, xi - est_x)


                # A orientação do robô é a diferença entre o ângulo global
                # e o ângulo que o sensor mediu 
                theta_i = angle_to_beacon - bearing_i
                
                # Acumula em cartesianas para evitar
                # o problema da média de ângulos (ex: média de 179° e -179°)
                sum_sin += np.sin(theta_i)
                sum_cos += np.cos(theta_i) 

            # Obter o ângulo médio final 
            est_theta = np.arctan2(sum_sin, sum_cos) 



            # Debug code
            #print('Matrix A:')
            #print(A)
            #print('Vector b:')
            #print(b)
            #print('Vector r:')
            #print(r)




        





        

# Converte o theta real de radianos para graus para ser mais fácil de ler
        real_theta_deg = np.rad2deg(self.robot_real_pose.theta)
        est_theta_deg = np.rad2deg(est_theta)

        self.get_logger().info(f'COMPARAÇÃO THETA: Calculado={est_theta_deg:.2f}°, Real={real_theta_deg:.2f}°')



        #Publicar os valores para serem utilizados no restante trabalho
        with self.lock:
            # Fill pose values using the localization results
            # CHANGE THIS: it is currently publishing the ground truth
            self.robot_estimated_pose = \
                Pose2D(x=est_x,
                       y=est_y,
                       theta=est_theta) 

            # Publish the pose message.
            pose_to_publish = PoseWithCovarianceStamped()
            pose_to_publish.header.frame_id = self.base_frame_id
            pose_to_publish.header.stamp = msg.header.stamp
            pose_to_publish.pose.pose.position = \
                Point(x=est_x,
                      y=est_y, z=0.)
            pose_to_publish.pose.pose.orientation = \
                utils.rpyToQuaternion(0., 0., est_theta)
            self.pose_pub.publish(pose_to_publish)

    def pose_gnd_truth_callback(self, msg: Odometry):
        '''
        Store internally a copy of the error-free robot pose.
        '''
        with self.lock:
            self.robot_real_pose.x = msg.pose.pose.position.x
            self.robot_real_pose.y = msg.pose.pose.position.y
            self.robot_real_pose.theta = \
                utils.quaternionToYaw(msg.pose.pose.orientation)


def main(args=None):
    '''
    Main function.
    '''
    print('Basic trilateration-based localization.\n' +
          '------------------------------------------\n')

    rclpy.init(args=args)
    loc_node = Trilateration()
    rclpy.spin(loc_node)
    loc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()