import time
import krpc
import collections
import math

from pid import PID


current = None
target = None
connector_separated = False

v3 = collections.namedtuple('v3', 'right forward up') 
conn = krpc.connect(name='Docking')
vessel = conn.space_center.active_vessel

current = vessel.parts.controlling.docking_port
target = conn.space_center.target_docking_port

##############################################################################
## Main  - only run when this file is explicitly executed
##############################################################################
def main():
    #conn = krpc.connect()
    
    dock(conn)

##############################################################################
## dock  - The actually interesting function in this file.   
## works by lining vessel up parallel, with 10m of separation between
## docking ports.  When this is confirmed, it moves forward slowly to dock.
##############################################################################
def dock(conn, speed_limit = 1.0):

    #Setup KRPC
    '''sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame
    
    #Setup Auto Pilot
    ap.reference_frame = rf   
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()'''

    #create PIDs
    upPID = PID(.75,.25,1)   
    rightPID = PID(.75,.25,1)
    forwardPID = PID(.75,.2,.5)

    proceed=False  
    current = None
    target = None
    connector_separated = False
    #'proceed' is a flag that signals that we're lined up and ready to dock.
    # Otherwise the program will try to line up 10m from the docking port.
 
    #LineUp and then dock  - in the same loop with 'proceed' controlling whether
    #we're headed for the 10m safe point, or headed forward to dock.
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
    
    #Setup Auto Pilot
            ap.reference_frame = rf   
            ap.target_direction = tuple(x * -1 for x in t.direction(rf))
            ap.target_roll = 0
            ap.engage()
            ap.wait()
            

            offset = getOffsets(v, t)  #grab data and compute setpoints
            velocity = getVelocities(v, t)
            if proceedCheck(offset):  #Check whether we're lined up and ready to dock
                proceed = True
            setpoints = getSetpoints(offset, proceed, speed_limit)  
        
            upPID.setpoint(setpoints.up)  #set PID setpoints
            rightPID.setpoint(setpoints.right)
            forwardPID.setpoint(setpoints.forward)
        
            v.control.up = (-1.0/8.0) * upPID.update(velocity.up)  #steer vessel
            v.control.right = (-1.0/8.0) * rightPID.update(velocity.right)
            #v.control.forward = (1/8.0) * forwardPID.update(velocity.forward)

            print('前后%f, 上下：%f, 左右:%f' % (v.control.forward, v.control.up, v.control.right))
     
            
             
##############################################################################
##  Helper Functions
##############################################################################
def getOffsets(v, t):
    '''
    returns the distance (right, forward, up) between docking ports.
    '''
    return v3._make(t.part.position(v.parts.controlling.reference_frame))

def getVelocities(v, t):
    '''
    returns the relative velocities (right, forward, up)
    '''
    return v3._make(v.velocity(t.reference_frame))

def getSetpoints(offset, proceed, speed_limit):
    '''
    returns the computed set points -
    set points are actually just the offset distances clamped to the
    speed_limit variable!   This way we slow down as we get closer to the right
    heading.
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
    returns true if we're lined up and ready to move forward to dock.
    '''
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward)<.1)
             

 
##This calls the main function which is at the top of the file for readability's sake!
if __name__ == '__main__':
    main()
    print('--')