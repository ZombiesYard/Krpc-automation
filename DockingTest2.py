import time
import krpc
import math

#宏定义
current = None
target = None
connector_separated = False
Kp = 0.05
Ki = 0.000000
Kd = 0.
Error_Of_Final_Docking = (0, 0, 0)

#连接服务器相关设置
conn = krpc.connect(name='Docking')
vessel = conn.space_center.active_vessel

current = vessel.parts.controlling.docking_port
target = conn.space_center.target_docking_port

'''vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
tgtdir = target.direction(vessel.orbital_reference_frame)
vessel.auto_pilot.target_direction = [-tgtdir[0], -tgtdir[1], -tgtdir[2]]
vessel.auto_pilot.engage()
vessel.auto_pilot.wait()
vessel.auto_pilot.target_roll = 180'''
while True:
    current = vessel.parts.controlling.docking_port
    target = conn.space_center.target_docking_port

    if current is None:

        print('等待选中指令舱连接器')

    elif connector_separated is False:
        vessel.control.activate_next_stage()
        connector_separated = True

    elif target is None:
        print('等待选中目标连接器')

    else:



        vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
        tgtdir = target.direction(vessel.orbital_reference_frame)
        #vessel.auto_pilot.target_direction = [-tgtdir[0]*-1, -tgtdir[1]*-1, -tgtdir[2]*-1]
        vessel.auto_pilot.target_direction = [-tgtdir[0], -tgtdir[1], -tgtdir[2]]
        vessel.auto_pilot.target_roll = 180
        vessel.auto_pilot.engage()
        vessel.auto_pilot.wait()
        #current_position = current.position(target.reference_frame)
        #velocity = current.part.velocity(target.reference_frame)

        #当前飞船和目标的距离
        Position = current.position(target.reference_frame)

        #当前飞船和目标的速度
        Velocity = current.part.velocity(target.reference_frame)

        #当前飞船误差范围
        Error_Of_Final_Docking = Error_Of_Final_Docking + Position

        time.sleep(0.2)

        #x轴的油门输出
        Force_Left_And_Right = (Kp * Position[0] + Kd * Velocity[0] + Ki * Error_Of_Final_Docking[0])

        #y轴的油门输出
        Force_Upward_And_Forward = (Kp * Position[1] + Kd * Velocity[1] + Ki * Error_Of_Final_Docking[1])

        #z轴的油门输出
        Force_Up_And_Down = (Kp * Position[2] + Kd * Velocity[2] + Ki * Error_Of_Final_Docking[2])

        #vessel.control.forward = 1 * Force_Upward_And_Forward

        '''print('\n前后输出：%f' % Force_Upward_And_Forward)
        print('\n前后速度：%f' % Velocity[1])
        print('\n前后距离：%f' % Position[1])'''

        #vessel.control.up = -1 * Force_Up_And_Down#待调参
        vessel.control.up = 1 * Position[2]
        '''print('\n上下输出：%f' % Force_Up_And_Down)
        print('\n上下速度：%f' % Velocity[2])
        print('\n上下距离：%f' % Position[2])'''

        #vessel.control.right = -1 * Force_Left_And_Right#待调参
        '''print('\n左右输出：%f' % Force_Left_And_Right)
        print('\n左右速度：%f' % Velocity[0])
        print('\n左右距离：%f' % Position[0])'''
        #print('\n油门大小为%f, %f, %f' % (Force_Upward_And_Forward,Force_Up_And_Down,Force_Left_And_Right))
        
        print('\n前后：%f 上下：%f 左右%f' % (vessel.control.forward, vessel.control.up ,vessel.control.right))
        








