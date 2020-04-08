import time
import krpc
import math

kp = 0.02
kd = 0.5
ki = 0.00001
s = (0, 0, 0)
target_port_selected = False
# 连接服务器
conn = krpc.connect(name='Docking Test')
vessel = conn.space_center.active_vessel
vessel_port = vessel.parts.docking_ports[0]
target_vessel = conn.space_center.target_vessel
#target_port = target_vessel.parts.docking_ports[0]
#target_port = conn.space_center.target_vessel.parts.docking_ports[0]
target_port = conn.space_center.target_docking_port

#target_frame = target_port.reference_frame
target_frame = target_vessel.reference_frame
vessel_frame = vessel_port.reference_frame

time.sleep(1)
vessel.auto_pilot.engage()
v_dir = target_port.direction(vessel.auto_pilot.reference_frame)
vessel.auto_pilot.target_direction = (-v_dir[0], -v_dir[1], -v_dir[2])
vessel.auto_pilot.target_roll = 0
vessel.auto_pilot.wait()
vessel.auto_pilot.sas = True
time.sleep(0.5)

target_vessel.auto_pilot.engage()
t_dir = vessel_port.direction(target_vessel.auto_pilot.reference_frame)
target_vessel.auto_pilot.target_direction = (-t_dir[0], -t_dir[1], -t_dir[2])
target_vessel.auto_pilot.target_roll = 0
target_vessel.auto_pilot.wait()
target_vessel.auto_pilot.sas = True
time.sleep(0.5)

print('开始对接')

if vessel_port.shielded:
    vessel_port.shielded = False
    while True:
        if not target_port_selected:
            if target_port is None:
                print('选定对接目标')
                target_port_selected = True

        p = target_port.position(vessel_frame) 
        #current_position = current.position(target.reference_frame)

        v = target_port.part.velocity(vessel_frame)
        #velocity = current.part.velocity(target.reference_frame)

        s = s+p
        time.sleep(0.2)

        f0 = (2 * kp * p[0] + kd * v[0]+ki * s[0])  #x轴左右方向
        f1 = (kp*p[1]+kd*v[1]+ki*s[1])              #y轴前后方向
        f2 = (2*kp*p[2]+kd*v[2]+ki*s[2])            #z轴上下方向

        vessel.control.foward = 1*f1
        vessel.control.up = 3*f2
        vessel.control.right = -3*f0
        print('对接完成')
