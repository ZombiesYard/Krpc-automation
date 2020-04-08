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

def predict_landing(posx,posy,posz,velx,vely,velz):
    zf = 0
    xf = 0
    yf = 0
    return xf,yf,zf

'''current = vessel.parts.controlling.docking_port
target = conn.space_center.target_docking_port

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
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
        Position = current.position(target.reference_frame)
        Velocity = current.part.velocity(target.reference_frame)
        (posx, posy, posz) = predict_landing(Position[2], Position[1], Position[0], Velocity[2], Velocity[1],
                                            Velocity[0])



        vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
        tgtdir = target.direction(vessel.orbital_reference_frame)
        #vessel.auto_pilot.target_direction = [-tgtdir[0]*-1, -tgtdir[1]*-1, -tgtdir[2]*-1]
        vessel.auto_pilot.target_direction = [-tgtdir[0], -tgtdir[1], -tgtdir[2]]
        vessel.auto_pilot.target_roll = 180
        vessel.auto_pilot.engage()
        vessel.auto_pilot.wait()
        pos0 = (posz, posy, posx)
        pos0_vessel = conn.space_center.transform_direction(pos0, target.reference_frame, vessel.reference_frame)
        vessel.control.up = pos0_vessel[2]
        vessel.control.right = -pos0_vessel[0]
        #current_position = current.position(target.reference_frame)
        #velocity = current.part.velocity(target.reference_frame)

        #当前飞船和目标的距离
        

        #当前飞船和目标的速度
        

        
        print('\n 上下：%f 左右%f' % (vessel.control.up ,vessel.control.right))
        








