"""Evaluation-only geometry; never imported by perception or planning."""
import numpy as np

def scene_clearance(positions, boxes):
    p=np.asarray(positions,dtype=float)
    if p.shape[-1]!=3 or not np.isfinite(p).all() or not boxes:
        raise ValueError('finite XYZ positions and nonempty obstacles required')
    distances=[]
    for box in boxes:
        center=np.asarray(box['center'],float);size=np.asarray(box['size'],float)
        if center.shape!=(3,) or size.shape!=(3,) or not np.isfinite([center,size]).all() or np.any(size<=0):
            raise ValueError('invalid evaluation box')
        yaw=float(box.get('yaw',0));c,s=np.cos(yaw),np.sin(yaw)
        rel=p-center
        local=rel @ np.array([[c,-s,0],[s,c,0],[0,0,1]])
        distances.append(np.linalg.norm(np.maximum(abs(local)-size/2,0),axis=-1))
    return np.min(np.stack(distances,axis=-1),axis=-1)

def gate_crossings(positions, boxes, margin=.75):
    """Report forward crossings through an axis-aligned two-box gate."""
    p=np.asarray(positions,float).reshape(-1,3)
    if len(boxes)!=2 or any(abs(b.get('yaw',0))>1e-9 for b in boxes):
        raise ValueError('gate requires two axis-aligned boxes')
    lower,upper=sorted(boxes,key=lambda b:b['center'][1])
    if not np.isclose(lower['center'][0],upper['center'][0]):
        raise ValueError('gate boxes must share their X center')
    plane=lower['center'][0]
    bottom=lower['center'][1]+lower['size'][1]/2+margin
    top=upper['center'][1]-upper['size'][1]/2-margin
    crossing=(p[:-1,0]<plane)&(p[1:,0]>=plane)
    a=p[:-1][crossing];b=p[1:][crossing]
    y=a[:,1]+(plane-a[:,0])*(b[:,1]-a[:,1])/(b[:,0]-a[:,0]) if len(a) else np.array([])
    return dict(forward_crossings=len(y),crossing_y_m=y.tolist(),allowable_y_m=[bottom,top],through_gap=bool(np.any((y>=bottom)&(y<=top))))
