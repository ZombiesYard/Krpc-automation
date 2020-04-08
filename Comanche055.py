from __future__ import division

import collections
import curses
import math
import threading
import time

import krpc
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

import _thread
from pid import PID

##################################################################
#宏定义设置和一些标志位
##################################################################

Gravitational_Turn_Finishied = False
turn_start_altitude = 250 #程序重力转向的开始高度
turn_end_altitude = 45000 #程序重力转向的结束高度
target_altitude = 100000 #目标轨道高度

##################################################################
#curses设置
##################################################################
'''stdscr = curses.initscr()
curses.nocbreak()
stdscr.keypad(1)
curses.noecho()'''

##################################################################
#连接服务器的设置，把一些api简写，设置遥测数据
##################################################################
conn = krpc.connect(name='Launch int Orbit and Docking')#kRPC连接服务器并设置连接的任务名称
vessel = conn.space_center.active_vessel#选中目前活动的火箭
ut = conn.add_stream(getattr, conn.space_center, 'ut')#通用时间单位
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')#高度
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')#地球轨道远点高度
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')#离远点的时间

##################################################################
#主程序
##################################################################
def main():
    activeThread = 0
    while True:
        #stdscr.clear()
        Ignition_Fire()
        if activeThread == 0:
            _thread.start_new_thread(Gravitational_Turn, ('status', ))
            activeThread = 1
        if Gravitational_Turn('status') is True:
            print('当前活动线程数%d' % threading.activeCount())
            First_Seperate()
            break
    Change_Orbit_Fire()
    dock(conn)
        


        #stdscr.refresh()

##################################################################
#点火程序，设置自动驾驶以及油门
##################################################################
def Ignition_Fire():
    #stdscr.clear()
    vessel.control.sas = False#关闭增稳
    vessel.control.rcs = False#关闭反应控制系统
    vessel.control.throttle = 1.0#油门拉满
    #stdscr.addstr(0, 0, '准备倒数')
    print('准备倒数')
    time.sleep(1)
    print('3')
    #stdscr.addstr(1, 0, '3')
    time.sleep(1)
    print('2')
    #stdscr.addstr(2, 0, '2')
    time.sleep(1)
    print('1')
    #stdscr.addstr(3, 0, '1')
    time.sleep(1)
    print('点火')
    #stdscr.addstr(4, 0, '点火!')
    vessel.control.activate_next_stage()#第一级点火启动
    vessel.auto_pilot.engage()#开启自动驾驶
    vessel.auto_pilot.target_pitch_and_heading(90,90)#设置自动驾驶方向
    vessel.auto_pilot.target_roll = 0#
    vessel.control.sas = True#打开增稳
    time.sleep(2)
    print('起飞')
    #stdscr.addstr(5, 0, '起飞!')
    ##stdscr.refresh()
##################################################################
#重力转向程序
##################################################################
def Gravitational_Turn(status):
    turn_angle = 0  #转向角，重力转向程序处理会用到
    status = False
    while True:
        if altitude() > turn_start_altitude and altitude() < turn_end_altitude:#重力转向区间内才能做转向动作，如果过了区间有可能会导致拱点太高
            frac = ((altitude() - turn_start_altitude)/
                    (turn_end_altitude - turn_start_altitude) )
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)
        
        if apoapsis() > target_altitude * 0.9:#预测拱点位置到了目标轨道高度的百分之九十就退出循环停止重力转向
            #stdscr.addstr(0, 0, '即将到达轨道远点，重力转向程序结束')
            status = True
            break
    return status

##################################################################
#第一级火箭的抛离程序
##################################################################
def First_Seperate():

    vessel.control.throttle = 0.05#拱点高度即将到达设定值时关小油门，给抛离程序六时间处理
    time.sleep(10)
    vessel.control.throttle = 0.0
    time.sleep(1)#关油门，准备抛一级火箭，考虑到一级火箭内可能有剩余燃料，所以任何抛离火箭的程序前都要关油门，否则抛离的火箭有可能直接撞上下一级
    vessel.control.activate_next_stage()#抛一级火箭
    print('一级火箭抛离')
    #stdscr.addstr(0, 0, '一级火箭抛离')
    time.sleep(5)#抛一级火箭，此时二级火箭应该已经启动
    vessel.control.throttle = 0.7#抛一级火箭后以百分之十五的油门继续前进，小油门防止拱点超过目标轨道高度
    while apoapsis() < target_altitude:#抛一级火箭后立即判断拱点是否超过期望轨道高度
        pass
    vessel.control.throttle = 0.0
    print('即将到达轨道远点')
    #stdscr.addstr(0, 0, '即将到达轨道远点')#拱点到达期望轨道高度立即关油门
    vessel.control.activate_next_stage()#抛逃逸塔
    print('逃逸塔已抛弃')
    #stdscr.addstr(0, 0, '逃逸塔已抛弃')
    time.sleep(1)
    vessel.control.activate_next_stage()#抛整流罩
    print('整流罩已抛弃')
    print('正在滑出大气层')
    #stdscr.addstr(0, 0, '整流罩已抛弃')
    #stdscr.addstr(1, 0, '正在滑出大气层')

##################################################################
#圆化轨道加速点计算程序
##################################################################
def Change_Orbit_Fire():
    while altitude() < 70500:#当高度超过70500的时候开始计算圆化轨道点以及dV
        pass
    print('正在计算轨道圆化点')
    #stdscr.addstr(4,0, '正在计算轨道圆化点')

    mu = vessel.orbit.body.gravitational_parameter                  
    r = vessel.orbit.apoapsis                                       
    a1 = vessel.orbit.semi_major_axis                               
    a2 = r                                      ###########################################
    v1 = math.sqrt(mu * ((2./r) - (1./a1)))     #首先，用vis-viva方程计算使轨道圆形化所需的dV#
    v2 = math.sqrt(mu * ((2./r) - (1./a2)))     ###########################################
    delta_v = v2 - v1                                               
    node = vessel.control.add_node(                                 
        ut() + vessel.orbit.time_to_apoapsis, prograde = delta_v)   

    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82        ##################################################
    m0 = vessel.mass                            #用齐奥尔科夫斯基火箭方程计算出达到此dV所需的燃烧时间##
    m1 = m0 / math.exp(delta_v / Isp)           ##################################################
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate
    print('正在调整飞船朝向至圆化轨道点矢量')
    #stdscr.addstr(4,0, '正在调整飞船朝向至圆化轨道点矢量')
    vessel.control.rcs = True#开启反应控制系统，会让姿态调整更快
    vessel.auto_pilot.reference_frame = node.reference_frame#选择刚刚计算出来的加速点为参考系
    vessel.auto_pilot.target_direction = (0, 1, 0)#沿着操纵节点的参考坐标系的y轴（即圆化点矢量方向）调整船的方向
    vessel.auto_pilot.wait()#开启自动驾驶等待,等姿态调整完毕
    time.sleep(20)

    burn_ut = ut() +vessel.orbit.time_to_apoapsis - (burn_time / 2.)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)#时间加速到启动点的前五秒
    print('时间加速完成')
    #stdscr.addstr(4,0, '时间加速完成')
    while time_to_apoapsis() - (burn_time/2.) > 0:
        pass#根据平均速度的公式，到数前开始加速的时间=需要加速的时间/2
    vessel.control.throttle = 1.0
    print('开始加速')
    #stdscr.addstr(5,0, '开始加速')
    time.sleep(burn_time - 0.1)
    print('正在进行最后微调')
    #stdscr.addstr(5,0, '开始加速')#最后的0.1秒微调
    vessel.control.throttle = 0.05#百分之五的油门微调
    remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    while remaining_burn()[1] > 0.5:
        pass
    vessel.control.throttle = 0.0
    node.remove()
    #vessel.auto_pilot.dis engage()
    time.sleep(15)
    vessel.control.sas_mod = vessel.control.sas_mod.stability_assist
    print('飞船成功入轨')
    #stdscr.addstr(5,0, '飞船成功入轨')



##################################################################
#对接程序
##################################################################
def dock(conn, speed_limit = 1.0):
    

    upPID = PID(.75,.25,1)          ##################
    rightPID = PID(.75,.25,1)       #创建PID         #
    forwardPID = PID(.75,.2,.5)     ##################

    proceed=False   #标志位，服务舱是否已经掉头并且与登月舱对接口矢量平行
    current = None  #标志位，当前是否选中服务舱对接口
    target = None   #标志位，是否选中登月舱对接口
    connector_separated = False #标志位，服务舱和登月舱的级间连接器是否分离

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

            sc = conn.space_center
            v = sc.active_vessel
            t = sc.target_docking_port
            ap = v.auto_pilot
            rf = v.orbit.body.reference_frame

        #设置自动导航
            ap.reference_frame = rf  
            ap.target_direction = tuple(x * -1 for x in t.direction(rf))
            ap.target_roll = 0
            ap.engage()
            ap.wait()

            offset = getOffsets(v, t)  #获取数据，计算加速点
            velocity = getVelocities(v, t)
            if proceedCheck(offset):#检查是否对齐
                proceed = True
            setpoints = getSetpoints(offset, proceed, speed_limit) 

            upPID.setpoint(setpoints.up)
            rightPID.setpoint(setpoints.right)
            forwardPID.setpoint(setpoints.forward)
        
            v.control.up = (-1.0/8.0) * upPID.update(velocity.up)
            v.control.right = (-1.0/8.0) * rightPID.update(velocity.right)
            #v.control.forward = (1/8.0) * forwardPID.update(velocity.forward)

##################################################################
#pid接口程序
##################################################################
def getOffsets(v, t):
    '''
    返回xyz离对接口的距离
    '''
    return v3._make(t.part.position(v.parts.controlling.reference_frame))

def getVelocities(v, t):
    '''
    返回xyz离对接口的相对速度
    '''
    return v3._make(v.velocity(t.reference_frame))

def getSetpoints(offset, proceed, speed_limit):
    '''
    返回计算的设置点，设置点实际上是速度限制变量的偏移，越靠近对接点速度越慢
    '''
    tvup = max(min(offset.up, speed_limit), -speed_limit)
    tvright = -1 * (max(min(offset.right, speed_limit), -speed_limit))
    if proceed:
        tvforward = -.2
    else:
        tvforward = max(min((10 - offset.forward), speed_limit), -speed_limit)
    return v3(tvright, tvforward, tvup)
   
def proceedCheck(offset):
    '''
    对齐后返回True
    '''
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward)<.1)


if __name__ == "__main__":
    main()
