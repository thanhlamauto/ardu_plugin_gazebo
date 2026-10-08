"""Goal-directed motion proposals; geometry remains the planner's safety input."""
import math
import numpy as np

def camera_motion_primitives(position,yaw,goal,horizon,dt,speed,yaw_rate_max):
    position=np.asarray(position,float);goal=np.asarray(goal,float)
    direction=goal[:2]-position[:2];length=np.linalg.norm(direction)
    direction=direction/length if length>1e-6 else np.array([math.cos(yaw),math.sin(yaw)])
    lateral=np.array([-direction[1],direction[0]])
    heading=math.atan2(direction[1],direction[0]);proposals=[]
    def actions(velocity):
        u=np.zeros((horizon,4));u[:,:2]=velocity
        predicted_yaw=yaw
        for i in range(horizon):
            delta=math.atan2(math.sin(heading-predicted_yaw),math.cos(heading-predicted_yaw))
            u[i,3]=np.clip(delta/dt,-yaw_rate_max,yaw_rate_max)
            predicted_yaw+=u[i,3]*dt
        return u
    for fraction in (.6,1.):
        for angle_deg in (-75,-60,-45,-30,0,30,45,60,75):
            a=math.radians(angle_deg);vector=direction*math.cos(a)+lateral*math.sin(a)
            proposals.append(actions(np.tile(vector*speed*fraction,(horizon,1))))
        for side in (-1.,1.):
            for phase in (.25,.5,.75):
                velocity=np.tile(direction*speed*fraction,(horizon,1))
                velocity[:max(1,round(horizon*phase))]=side*lateral*speed*fraction
                proposals.append(actions(velocity))
    return np.asarray(proposals)
