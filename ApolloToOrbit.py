"""阿波罗11号的发射程序，自动将火箭发射至10km轨道"""
import math
import time
import krpc

turn_start_altitude = 250 
#程序重力转向的开始高度
turn_end_altitude = 45000 
#程序重力转向的结束高度
target_altitude = 100000 
#目标轨道高度

conn = krpc.connect(name='Launch int Orbit')
#kRPC连接服务器并设置连接的任务名称
vessel = conn.space_center.active_vessel
#选中可发射的火箭

"""以下设置遥测数据接口"""
ut = conn.add_stream(getattr, conn.space_center, 'ut')
#通用时间单位-流传输到客户端
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
#高度
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
#拱点高度
"""遥测数据接口设置完毕"""

"""进行发射前的准备"""
vessel.control.sas = False
#关闭增稳
vessel.control.rcs = False
#关闭反应控制系统
vessel.control.throttle = 1.0
#油门拉满
'''发射前准备完成'''

'''开始点火程序'''
print('准备倒数')
#开始倒数
time.sleep(1)
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('点火!')

vessel.control.activate_next_stage()
#第一级点火启动
vessel.auto_pilot.engage()
#开启自动驾驶
vessel.auto_pilot.target_pitch_and_heading(90,90)
#设置自动驾驶方向
vessel.control.sas = True

time.sleep(2)
print('起飞!')
#延时2秒显示起飞
'''点火程序结束'''

'''重力转向程序'''
turn_angle = 0
#转向角，重力转向程序处理会用到

while True:
    #在循环里处理重力转向
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        #重力转向区间内才能做转向动作，如果过了区间有可能会导致拱点太高
        frac = ((altitude() - turn_start_altitude)/
                (turn_end_altitude - turn_start_altitude) )
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)
    
    if apoapsis() > target_altitude * 0.9:
        #预测拱点位置到了目标轨道高度的百分之九十就退出循环停止重力转向
        print('即将到达轨道远点')
        break
'''重力转向程序结束'''

'''到拱点前二次加速程序'''
vessel.control.throttle = 0.05
#关小一点点油门
time.sleep(10)
vessel.control.throttle = 0.0
time.sleep(1)
#关油门，准备抛一级火箭
vessel.control.activate_next_stage()
print('抛一级火箭')
time.sleep(10)
#抛一级火箭，此时二级火箭应该已经启动
vessel.control.throttle = 0.7
#抛一级火箭后以百分之十五的油门继续前进，小油门防止拱点超过目标轨道高度
while apoapsis() < target_altitude:
    pass
#抛一级火箭后立即判断拱点是否超过期望轨道高度
vessel.control.throttle = 0.0
print("即将到达轨道远点")
#拱点到达期望轨道高度立即关油门
vessel.control.activate_next_stage()
print('逃逸塔已抛弃')
#抛逃逸塔
time.sleep(2)
vessel.control.activate_next_stage()
print('整流罩已抛弃')
#抛整流罩
print('正在漂移出大气层')
'''到拱点前二次加速程序结束'''

'''圆化轨道加速点计算程序'''
while altitude() < 70500:
    pass
#当高度超过70500的时候开始计算圆化轨道点以及dV
print('正在计算轨道圆化点')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu * ((2./r) - (1./a1)))
v2 = math.sqrt(mu * ((2./r) - (1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde = delta_v)
#首先，用vis-viva方程计算使轨道圆形化所需的dV
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate
#用齐奥尔科夫斯基火箭方程计算出达到此dV所需的燃烧时间
'''圆化轨道加速点计算程序结束'''

'''圆化轨道加速点之前的飞船姿态调整程序'''
print('正在调整飞船朝向至圆化轨道点矢量')
vessel.control.rcs = True
#开启反应控制系统，会让姿态调整更快
vessel.auto_pilot.reference_frame = node.reference_frame
#参考系选择刚刚计算出来的加速点
vessel.auto_pilot.target_direction = (0, 1, 0)
#四元数表示的调整朝向，沿着操纵节点的参考坐标系的y轴（即圆化点矢量方向）调整船的方向
vessel.auto_pilot.wait()
#开启自动驾驶等待
time.sleep(20)
#20秒延时给系统调整姿态
burn_ut = ut() +vessel.orbit.time_to_apoapsis - (burn_time / 2.)
lead_time = 5
#时间加速到启动点的前五秒
conn.space_center.warp_to(burn_ut - lead_time)
#执行时间加速
print('时间加速完成')
'''圆化轨道加速点之前的飞船姿态调整程序结束'''

'''入轨加速程序'''
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
#根据平均速度的公式，到拱点的时间=需要加速的总时间/2，就是我们开始加速的时间，而不是到了加速点再加速。
vessel.control.throttle = 1.0
print('开始加速···')
#开满油门开始加速
time.sleep(burn_time - 0.1)
print('正在进行最后微调')
#最后的0.1秒微调
vessel.control.throttle = 0.05
#百分之五的油门微调
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 0.5:
    pass
vessel.control.throttle = 0.0
node.remove()
vessel.auto_pilot.engage()
print('坎巴拉太空中心：飞船成功入轨，任务圆满完成')
while True:
    pass









