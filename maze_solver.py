#!/usr/bin/env python3

import math
from math import atan2
import random
import sys
import time
import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from sensor_msgs.msg import LaserScan
from rospy.exceptions import ROSInterruptException

# Distancia a la paret
distancia_paret = 0.3
# Marge de gir per evitar col·lisions
marge_paret = 0.6

pub = None
sub = None

# Seguirà la paret per l'esquerra: 1
# Seguirà la paret per la dreta: -1
costat = 0
# Velocitat angular
vel_angular = 0
# Velocitat lineal
vel_linear = 0.2
# Estat 0: busca paret
# Estat 1: paret trobada, va cap a la direcció
# Estat 2: segueix la paret
estat = 0
# Quan ha girat per últim cop el robot (utilitzat en l'estat 0 and 1)
ultim_gir = 0
# On es troba la paret (utilitzat en l'estat 0 and 1)
direccio_paret = None
# Simula el temps en què comença la simulació
start_time = None

# Posició
posicio_ = Point()
posicio = (-14,-14)
theta = 0
# Quadrat final on ha d'arribar el robot (centre)
quadrat_final = [(i,j) for i in range(-1,2) for j in range(-1,2)]

def clbk_odom(vel):
    global posicio_
    global theta
    global posicio
    
    # posició
    posicio_ = vel.pose.pose.position
    
    # yaw
    quaternion = (
        vel.pose.pose.orientation.x,
        vel.pose.pose.orientation.y,
        vel.pose.pose.orientation.z,
        vel.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    theta = euler[2] # yaw
    posicio = (int(posicio_.x),int(posicio_.y))
    
def update_command_vel(linear_vel, angular_vel):
    vel = Twist()
    vel.linear.x = linear_vel
    vel.angular.z = angular_vel
    pub.publish(vel)

def scan_callback(vel):
    scaner_max_valor = vel.range_max
    # Cada regió escaneja 9 graus
    regions = {
        'N':  min(min(vel.ranges[len(vel.ranges) - 4 : len(vel.ranges) - 1] + vel.ranges[0:5]), scaner_max_valor),
        'NNW':  min(min(vel.ranges[11:20]), scaner_max_valor),
        'NW':  min(min(vel.ranges[41:50]), scaner_max_valor),
        'WNW':  min(min(vel.ranges[64:73]), scaner_max_valor),
        'W':  min(min(vel.ranges[86:95]), scaner_max_valor),
        'E':  min(min(vel.ranges[266:275]), scaner_max_valor),
        'ENE':  min(min(vel.ranges[289:298]), scaner_max_valor),
        'NE':  min(min(vel.ranges[311:320]), scaner_max_valor),
        'NNE':  min(min(vel.ranges[341:350]), scaner_max_valor),
    }

    global costat, vel_angular, estat, ultim_gir, direccio_paret

    if estat == 0:  # busca paret
        # Comprova si la paret s'està detectant
        for r, v in regions.items():
            if r in ["N", "W", "E", "NW", "NE"] and v < scaner_max_valor:
                print('Will change to estat 1: drive towards wall ({})'.format(r))
                estat = 1
                direccio_paret = r
                ultim_gir = time.time()
                return

        # Gira cada 6 segons aleatòriament
        delta_time = time.time() - ultim_gir

        if delta_time > 6:
            rand = random.randrange(0, 5)
            vel_angular = math.pi / 2 - rand * math.pi / 4
            ultim_gir = time.time()

        elif delta_time > 1:
            vel_angular = 0

    elif estat == 1:  # Va cap a la paret
        # Distancia minima per començar a seguir la paret
        distancia_min = distancia_paret + 0.3

        # Comprova si la paret està prou a prop per seguir-la
        if regions['N'] < distancia_min or regions['NW'] < distancia_min or regions['NE'] < distancia_min:
            estat = 2

            if costat == 0:
                left = (regions['W'] + regions['NW']) / 2
                right = (regions['E'] + regions['NE']) / 2

                if left < scaner_max_valor or right < scaner_max_valor:
                    costat = -1 if right < left else 1
                else:
                    costat = random.randrange(-1, 2, 2)

            print('Will change to estat 2: follow wall from the {}'.format('left' if costat == 1 else 'right'))
            return

        # Gira cap a la paret
        delta_time = time.time() - ultim_gir

        if delta_time <= 1:
            if direccio_paret == 'W':
                vel_angular = math.pi / 2
            elif direccio_paret == 'NW':
                vel_angular = math.pi / 4
            elif direccio_paret == 'N':
                vel_angular = 0
            elif direccio_paret == 'NE':
                vel_angular = -math.pi / 4
            elif direccio_paret == 'E':
                vel_angular = -math.pi / 2
        else:
            vel_angular = 0

    elif estat == 2:  # seguiex la paret

        y0 = regions['E'] if costat == -1 else regions['W']
        x1 = (regions['ENE'] if costat == -1 else regions['WNW']) * math.sin(math.radians(23))
        y1 = (regions['ENE'] if costat == -1 else regions['WNW']) * math.cos(math.radians(23))

        # Si el robot està apuntant directament a la paret, gira per posar-se de costat
        if y0 >= distancia_paret * 2 and regions['N'] < scaner_max_valor:
            vel_angular = -math.pi / 4 * costat
        else:
            # Comprova informació de l'escaner per davant
            escaner_frontal = min([regions['N'], regions['NNW'] + (scaner_max_valor - regions['WNW']), regions['NNE'] + (scaner_max_valor - regions['ENE'])])

            # Si hi ha una paret pròxima al davant, ajusta la velocitat angular per evitar-la (important per cantonades)
            gir_fix = (0 if escaner_frontal >= 0.5 else 1 - escaner_frontal)

            # Calcula la velocitat angular necessària per seguir la paret
            abs_alpha = math.atan2(y1 - distancia_paret,
                                x1 + marge_paret - y0) - gir_fix * 1.5
            
            # Escull la direcció correcta per la velocitat angular
            vel_angular = costat * abs_alpha

def arriba_centre():
    global vel_linear,vel_angular
    
    angle_goal = atan2(-posicio[1], -posicio[0]) # l'angle a la meta, tenint en compte que és el punt (0,0)
    
    if abs(angle_goal - theta) > 0.1: # comprova si està apuntant a la meta
        vel_linear = 0.0
        vel_angular = 0.3
    else:
        vel_linear = 0.5
        vel_angular = 0.0

    if posicio == (0,0):   # si està al centre completa el laberint
        vel_linear = 0.0
        vel_angular = 0.0
        print("LABERINT COMPLETAT")
        sys.exit()

try:
    if __name__ == '__main__':
        
        print('Starting with values:')
        print('- linear speed: ', str(vel_linear))
        print('- distance to wall: ', str(distancia_paret))
        print('')

        rospy.init_node('laberint_solver')

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
        sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

        rate = rospy.Rate(20)

        start_time = rospy.get_time()
        print('Started at {} seconds (sim time)'.format(start_time))

        while not rospy.is_shutdown() and posicio not in quadrat_final:
            update_command_vel(vel_linear, vel_angular)
            rate.sleep()

        while not rospy.is_shutdown() and posicio in quadrat_final: # quan arriba al quadrat final
            arriba_centre()
            update_command_vel(vel_linear, vel_angular)
            rate.sleep()  

except ROSInterruptException:
    final_simulacio = rospy.get_time()
    print('Finished at {} seconds (sim time)'.format(final_simulacio))
    print('Ran for {} seconds'.format(final_simulacio - start_time))