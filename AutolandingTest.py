# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
# Suicide Burn Autopilot
# 
# Copyright (c) 2019 Marc Seifert
# This Python script is published under the MIT license.
# The license text can be found at the end of this file.
#
# "Suicide Burn Autopilot" is a Python script using the 
# Kerbal Space Program mods KRPC and MechJeb to land a
# spacecraft by performing a suicide burn maneuver.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

from __future__ import division
import krpc
import time
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq


# ---
# constants and settings
# ---

g = 9.81335163116455    # standard gravity in KSP
MAX_STEP_IVP = 0.25      # max step size of solve_ivp in seconds
RPC_PORT = 50000        # ports used for the KRPC connection
STREAM_PORT = 50001
CONSOLE_OUTPUT = True   # enables text output
FINAL_ALTITUDE = 84     # altitude offset from surface in m

conn = krpc.connect(name='Suicide Burn Autopilot', rpc_port=RPC_PORT, stream_port=STREAM_PORT)
create_relative = conn.space_center.ReferenceFrame.create_relative
vessel = conn.space_center.active_vessel
body = vessel.orbit.body
altitude_Less_Than_25 = False

# ---
# functions
# ---

def vis_viva(r, mu, a):
    '''
    Returns the orbital velocity as a function of the radius r, the gravitational
    parameter mu, and the semi major axis of the orbit a.
    '''
    return np.sqrt(mu*(2/r - 1/a))


def delta_h(t, orbit, body, refframe):
    '''
    Calculates the altitude difference
    between orbit and surface at time t.
    '''
    pos = orbit.position_at(t, refframe)
    lat = body.latitude_at_position(pos, refframe)
    lon = body.longitude_at_position(pos, refframe)
    surface_h = body.surface_height(lat, lon)
    dh = body.altitude_at_position(pos, refframe) - surface_h
    return dh


def EOM(t, w, c, t_burn, refframes, sc):
    '''
    Defines the 3d equations of motion of a rocket in a gravitational field
    burning retrograde in the rotating frame of the body.
    '''
    #x, y, z, vx, vy, vz = w    # initial coordinates and velocity (rotated frame)
    #mu, F, m0, dm = c          # graviational parameter and rocket-defining parameters
    #body_fixed_frame, refframe_rotating = refframes   # body reference frames

    # convert velocity to rotating frame
    vr = sc.transform_velocity(w[0:3], w[3:6], refframes[0], refframes[1])
    
    # normalize velocity vector
    vr = np.array(vr)/np.linalg.norm(vr)
    
    # transform velocity direction to fixed frame -> retrograde direction
    rg = sc.transform_direction(vr, refframes[1], refframes[0])
    
    # distance between vessel and center of body
    d = np.linalg.norm(w[0:3])
    
    m = c[2]-c[3]*(t-t_burn)
    f = [w[3],
         w[4],
         w[5],
         -c[1]*rg[0]/m - c[0]*w[0]/d**3,
         -c[1]*rg[1]/m - c[0]*w[1]/d**3,
         -c[1]*rg[2]/m - c[0]*w[2]/d**3]
    return f


def cost_function(t_burn, w, constants, t0, refframes, sc, body, orbit):
    '''
    Calculates the final hight after hoverslam maneuver.
    '''
    mu, F, m0, dm = constants
    refframe_fixed, refframe_rotating = refframes   # body reference frames
    
    # position at t_burn
    pos_burn = orbit.position_at(t_burn+t0, refframe_fixed)
    
    # speed at t_burn
    speed_burn = vis_viva(orbit.radius_at(t_burn+t0), mu, orbit.semi_major_axis)
    
    # direction at t_burn
    dir_burn = np.array(orbit.position_at(t_burn+t0+0.5, refframe_fixed)) - np.array(orbit.position_at(t_burn+t0-0.5, refframe_fixed))
    dir_burn /= np.linalg.norm(dir_burn)  # normalize
    vel_burn = tuple(speed_burn*dir_burn) # velocity vector at t_burn

    w_burn = pos_burn + vel_burn
    
    t_span = (t_burn, orbit.time_to_periapsis)
    
    # The solver evaluates EOM() within the time interval t_span. If the velocity
    # drops to zero (calculated by finish_burn) the solver stops the evaluation.
    s = solve_ivp(lambda t, w: EOM(t, w, constants, t_burn, refframes, sc),
                  t_span, w_burn, max_step=MAX_STEP_IVP, events=finish_burn,
                  dense_output=True, method='LSODA')
    
    t_event = s.t_events[0][0]
    
    # position and velocity at impact or zero velocity
    f = s.sol(t_event)
    
    # current altitude above terrain at t_event
    lat = body.latitude_at_position(f[0:3], refframe_fixed)        # latitude
    lon = body.longitude_at_position(f[0:3], refframe_fixed)       # longitude
    body_rotation = t_event*body.rotational_speed*180./np.pi
    lon -= body_rotation    # compensate body rotation
    if lon < -180.:
        lon += 360.
    current_alt = body.altitude_at_position(f[0:3], refframe_fixed) - body.surface_height(lat, lon)

    if CONSOLE_OUTPUT:
        print ('%4.3f\t\t%.2f' % (t_burn, current_alt))
    return current_alt - FINAL_ALTITUDE


# -------
# initialize connection to KSP

landing_latitude = -(1.+(31./60)+(15./60/60))
landing_longitude = -(71.+(53./60)+(50./60/60))
landing_altitude = 5

landing_position = body.surface_position(
    landing_latitude, landing_longitude, body.reference_frame)
q_long = (
    0,
    np.sin(-landing_longitude * 0.5 * np.pi / 180),
    0,
    np.cos(-landing_longitude * 0.5 * np.pi / 180)
)
q_lat = (
    0,
    0,
    np.sin(landing_latitude * 0.5 * np.pi / 180),
    np.cos(landing_latitude * 0.5 * np.pi / 180)
)

landing_reference_frame = \
    create_relative(
        create_relative(
            create_relative(
                body.reference_frame,
                landing_position,
                q_long),
            (0, 0, 0),
            q_lat),
        (landing_altitude, 0, 0))



altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
velocity = conn.add_stream(vessel.velocity,landing_reference_frame)
v_speed = conn.add_stream(getattr,vessel.flight(landing_reference_frame), 'vertical_speed')
h_speed = conn.add_stream(getattr,vessel.flight(landing_reference_frame), 'horizontal_speed')
available_thrust = conn.add_stream(getattr,vessel,'available_thrust')
mass = conn.add_stream(getattr, vessel, 'mass')
G = conn.add_stream(getattr,body, 'surface_gravity')

'''
# ---
# set up UI
# ---
canvas = conn.ui.stock_canvas
screen_size = canvas.rect_transform.size
panel = canvas.add_panel()
panel_size = (200, 120)
panel.rect_transform.size = panel_size
panel.rect_transform.position = (screen_size[0]//2-panel_size[0]//2-10, 0)
title_panel = panel.add_panel()
title_panel.rect_transform.size = (panel_size[0], 24)
title_panel.rect_transform.position = (0, panel_size[1]//2-12)
title = title_panel.add_text('Suicide Burn Autopilot UI')
title.alignment = conn.ui.TextAnchor.middle_center
title.color = (1.0, 1.0, 1.0)
title.size = 13

# add a button to start the hoverslam procedure
button = panel.add_button('Start autopilot')
button.rect_transform.size = (panel_size[0]-20, 30)
button.rect_transform.position = (0, 12)

# add stream to monitor button state
button_clicked = conn.add_stream(getattr, button, 'clicked')

# add text output\
info_text = panel.add_text('')
info_text.rect_transform.position = (0, -40)
info_text.rect_transform.size = (panel_size[0]-20, 60)
info_text.alignment = conn.ui.TextAnchor.upper_left
info_text.color = (1.0, 1.0, 1.0)
info_text.size = 13'''


# ---
# main loop
# ---
while True:
    #if button_clicked():
    if True:
        #button.clicked = False  # reset button state
        time.sleep(0.1)
        
        # collect information about vessel and orbit
        # cancel action if requirements for hoverslam are not fulfilled
        sc = conn.space_center
        vessel = sc.active_vessel
        orbit = vessel.orbit
        body = orbit.body
        if vessel.situation != sc.VesselSituation.sub_orbital:
            #info_text.content = 'Spacecraft status not sub-orbital!'
            #info_text.color = (1.0, 0.501, 0.078)
            print('飞船不在亚轨道上!逆向减速将远点降到0')
        elif body.has_atmosphere:
            #info_text.content = 'Planet/moon has an atmosphere!'
            #info_text.color = (1.0, 0.501, 0.078)
            print('这个星球有大气层！')
        elif vessel.available_thrust < 1e-6:
            #info_text.content = 'No thrust available!'
            #info_text.color = (1.0, 0.501, 0.078)
            print('发动机总推力不足!')
        else:
            # conditions checked
            # collect further information about vessel and body
            body_fixed_frame = body.non_rotating_reference_frame
            body_rotating_frame = body.reference_frame
            mu = body.gravitational_parameter
            m0 = vessel.mass
            F = vessel.max_vacuum_thrust
            dm = F/(vessel.vacuum_specific_impulse*g)    # fuel consumption (kg/s)
            R = body.equatorial_radius
            
            # data streams (positions, velocities and time)
            pos = conn.add_stream(vessel.position, body_fixed_frame)
            pos_rot = conn.add_stream(vessel.position, body_rotating_frame)
            vel_fixed = conn.add_stream(vessel.velocity, body_fixed_frame)
            vel_rotating = conn.add_stream(vessel.velocity, body_rotating_frame)
            ut = conn.add_stream(getattr, sc, 'ut')    # time
            
            # current state used for calculations
            t0 = ut()
            pos0 = pos()
            vel0 = vel_fixed()
            
            # starting calculations
            #info_text.content = 'Calculating hoverslam trajectory...'
            #info_text.color = (1.0, 1.0, 1.0)
            
            # find the time of impact on the surface by finding the root of
            # the function delta_h using scipy.optimize.brentq
            # interval [t0, t1] in which delta_h changes its sign
            t1 = t0 + orbit.time_to_periapsis
            t_impact = brentq(delta_h, args=(orbit, body, body_fixed_frame),
                              a=t0, b=t1)

            # estimate time until burn
            # the factor 0.8 is empirical
            t_burn_guess = t_impact - 0.8*np.linalg.norm(vel_rotating())/(F/m0)
            
            # ---
            # NOTE: Times until now are expressed in KSP universal time (ut). For
            # the calculations it is easier to proceed in relative time starting
            # at t0 (see above). To shift to the relative time frame, t0 is 
            # subtracted from ut.
            # ---
            
            # define event functions for the optimization algorithm
            # done here to enable access to objects like sc, body and so on
            def finish_burn(t, y):
                '''
                Calculates the current speed in the rotating frame. Terminates
                the solver when reaching 1 m/s. 0 m/s is not possible as the
                function has to change sign.
                '''
                vr = sc.transform_velocity(y[0:3], y[3:6],
                                           body_fixed_frame, body_rotating_frame)
                return np.linalg.norm(vr) - 1.0
            finish_burn.terminal = True
            finish_burn.direction = -1
            
            
            # ---
            # the actual minimization process
            # ---
            
            if CONSOLE_OUTPUT:
                print ('\n正在解方程：优化接地点\n') #minimizing cost function
                print ('开始点火时间t_burn \t\t 高度altitude')
            
            # w0: initial state of the vessel
            w0 = pos0 + vel0
            
            # c: constants required for optimization
            c = (mu, F, m0, dm)
            
            # t_span: time interval in which the EOM is evaluated
            t_span_end = t_impact-t0-2.0
            
            refframes = (body_fixed_frame, body_rotating_frame)
            tt = time.time()    # measure the time of the optimization
            
            # find the root of cost_function and therefore the t_burn where
            # the final altitude is FINAL_ALTITUDE
            t_burn = brentq(cost_function, 0.0, t_span_end,
                            args=(w0, c, t0, refframes, sc, body, orbit),
                            xtol=1e-3, rtol=1e-4)

            
            # ---
            # evaluate the EOM again at t_burn
            # ---
            
            # position at t_burn
            pos_burn = orbit.position_at(t_burn+t0, body_fixed_frame)
            
            # speed at t_burn
            speed_burn = vis_viva(orbit.radius_at(t_burn+t0), mu, orbit.semi_major_axis)
            
            # direction at t_burn
            dir_burn = np.array(orbit.position_at(t_burn+t0+0.5, body_fixed_frame)) - np.array(orbit.position_at(t_burn+t0-0.5, body_fixed_frame))
            dir_burn /= np.linalg.norm(dir_burn)  # normalize
            vel_burn = tuple(speed_burn*dir_burn) # velocity vector at t_burn
        
            w_burn = pos_burn + vel_burn
            t_span = (t_burn, orbit.time_to_periapsis)
            s = solve_ivp(lambda t, w: EOM(t, w, c, t_burn, refframes, sc),
                          t_span, w_burn, max_step=MAX_STEP_IVP, events=finish_burn,
                          dense_output=True, method='LSODA')
            t_touchdown = s.t_events[0][0]
            
            # evaluate solution at end of burn (t_touchdown)
            final = s.sol(t_touchdown)  # pos and vel of vessel at end of maneuver
            posf = final[0:3]
            lat = body.latitude_at_position(posf, body_fixed_frame)
            lon = body.longitude_at_position(posf, body_fixed_frame)
            body_rotation = t_touchdown*body.rotational_speed*180./np.pi
            lon -= body_rotation    # compensate rotation of the body
            if lon < -180.0:
                lon += 360.0
            surface_h = body.surface_height(lat, lon)
            hf = body.altitude_at_position(posf, body_fixed_frame) - surface_h
            
            # final velocity
            vrf = sc.transform_velocity(posf, final[3:6],
                                        body_fixed_frame, body_rotating_frame)
            
            # required delta_v, calculated using the rocket equation
            delta_v = vessel.vacuum_specific_impulse*g*np.log(m0/(m0-dm*(t_touchdown-t_burn)))
            
            if CONSOLE_OUTPUT:
                print ('\n降落点优化计算完成')
                print ('本次计算耗时: %3.1f s' % (time.time()-tt))
                print ('\n\n优化计算结果:')
                print ('\n开始点火时间 = %4.2f s' % (t_burn)) #t_burn
                print ('结束点火时间 = %4.2f s' % t_touchdown) #t_touchdown
                print ('\n结束点火后的高度: %.1f m' % hf)
                print ('结束点火后的速度: vx=%2.1f m/s, vy=%2.1f m/s, vz=%2.1f m/s' % (vrf[0], vrf[1], vrf[2]))
                print ('结束点火后的坐标: (%3.4f, %3.4f)\n' % (lat, lon))
                print ('需要的delta_v: %3.1f m/s\n' % delta_v)
            
            #info_text.content = 'Calculation finished'
            #info_text.color = (1.0, 1.0, 1.0)
            
            # if game is paused, wait
            while conn.krpc.paused:
                time.sleep(0.2)
            
            # initialize krpc and mechjeb autopilots
            ap = vessel.auto_pilot
            ap.sas = False
            ap.reference_frame = body_rotating_frame
            control = vessel.control
            #mj = conn.mech_jeb
            
            #info_text.content = 'Starting autopilot'
            #info_text.color = (1.0, 1.0, 1.0)
            
            # approximate direction of velocity at t_burn
            pos1 = np.array(orbit.position_at(t0+t_burn, body_rotating_frame))
            pos2 = np.array(orbit.position_at(t0+t_burn+2.0, body_rotating_frame))
            dpos = pos1 - pos2
            v_burn = dpos/np.linalg.norm(dpos)
            
            # rotate vessel to retrograde direction at t_burn
            ap.engage()
            ap.target_direction = v_burn
            ap.wait()
            ap.disengage()
            ap.sas = True
            vessel.control.rcs = True
            ap.sas_mode = ap.sas_mode.stability_assist
            
            # warp to t_burn-10s
            #info_text.content = 'Warping to burn'
            #info_text.color = (1.0, 1.0, 1.0)
            sc.warp_to(t_burn + t0 - 10.0)
            
            # point vessel retrograde during burn
            ap.sas_mode = ap.sas_mode.retrograde
            # wait until t_burn, start half a physics tick early
            while ut() < t_burn + t0 - 0.02:
                time.sleep(0.01)
            
            # light the candle!
            if CONSOLE_OUTPUT:
                print ('\n自杀点火启动!')
            control.throttle = 1.0
            control.gear = True
            print('\n起落架已展开')
            #info_text.content = 'Suicide burn started'
            #info_text.color = (1.0, 1.0, 1.0)
            
            # wait until the vessel is slowed down below 3 m/s
            while np.linalg.norm(vel_rotating()) > 3.0:
                time.sleep(0.01)
            control.throttle = 0.1
            if CONSOLE_OUTPUT:
                print ('\n自杀点火结束')
            #current_alt = body.altitude_at_position([0][3], refframes)
            # switch to mechjeb landing autopilot
            print('准备进入PID控制,倒数')
            time.sleep(1)
            print('7')
            time.sleep(1)
            print('6')
            time.sleep(1)
            print('5')
            time.sleep(1)
            print('4')
            time.sleep(1)
            print('3')
            time.sleep(1)
            print('2')
            time.sleep(1)
            print('1')

            if CONSOLE_OUTPUT:
                print ('\n!!正在转换到PID控制模式!!')
                print ('\n!!正在转换到PID控制模式!!')
                print ('\n!!正在转换到PID控制模式!!')
                
                print(altitude())
                #print ('\n终止高度为：%f' % current_alt)
            landing_longitude = lon
            landing_latitude = lat
            print ('PID坐标：%f,%f' % (landing_latitude, landing_longitude))
            ap.sas = True
            ap.sas_mode = ap.sas_mode.retrograde
            while altitude() > 10:
                #ap.sas_mode = ap.sas_mode.stability_assist
                #ap.sas_mode = ap.sas_mode.retrograde
                vel0 = (velocity()[0], velocity()[1], velocity()[2])
                vel0_vessel = conn.space_center.transform_direction(vel0, landing_reference_frame, vessel.reference_frame)
                vessel.control.up = vel0_vessel[2]
                vessel.control.right = -vel0_vessel[0]

                p = 0. - altitude()
                d = 0. - v_speed()
                d_h = 0 - h_speed()
                T0 = available_thrust()
                m = mass()
                F0 = 0.93 * m * G()
                u = (F0 + 300*p + 7000*d) / T0
                vessel.control.throttle = u
                print('！！！PID控制中！！！')
                if not altitude_Less_Than_25:
                    if altitude() > 25 and altitude() < 35:
                        ap.sas_mode = ap.sas_mode.stability_assist
                        altitude_Less_Than_25 = True


            '''lap = mj.landing_autopilot
            lap.enabled = True
            lap.touchdown_speed = 1.0
            lap.land_untargeted()'''
            
            # cut throttle and enable SAS when landed
            print('PID控制结束---请接手飞船驾驶')
            while vessel.situation != sc.VesselSituation.landed:
                time.sleep(0.01)
            control.throttle = 0.0
            #lap.enabled = False
            ap.sas = True
            ap.sas_mode = ap.sas_mode.stability_assist
            
            #info_text.content = 'Somehow we landed'
            #info_text.color = (0.0, 1.0, 0.0)
            
            if CONSOLE_OUTPUT:
                print ('\n触地')
            
            # the actual landing coordinates
            lat = body.latitude_at_position(pos(), body_fixed_frame)
            lon = body.longitude_at_position(pos(), body_fixed_frame)
            if CONSOLE_OUTPUT:
                print ('\n触地坐标: (%3.4f, %3.4f)\n' % (lat, lon))
            
            conn.close()
            exit()


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
# MIT License
#
# Copyright (c) 2019 Marc Seifert
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial
# portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
# LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
