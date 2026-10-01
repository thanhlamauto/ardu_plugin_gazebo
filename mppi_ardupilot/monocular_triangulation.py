"""Metric two-view geometry from one RGB camera and timestamped body poses.

Requires scaled poses (the current odometry subsystem). No obstacle map, range
sensor, or learned-depth scale fitting. Low parallax is rejected rather than
interpreted as free space. Matching and dense surface reconstruction are separate.
"""
import numpy as np

OPTICAL_TO_FLU = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
CAMERA_OFFSET_FLU = np.array([.17,0.,.16])


def triangulate_matches(pixels_a, pixels_b, position_a, rotation_a,
                        position_b, rotation_b, intrinsics, *,
                        min_parallax_deg=1., max_reprojection_px=1.5,
                        min_range=.3, max_range=25.):
    """Return occupied world points and a validity mask for matched pixels."""
    a,b=np.asarray(pixels_a,float),np.asarray(pixels_b,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=2:
        raise ValueError('pixel correspondences must be matching Nx2 arrays')
    k=np.asarray(intrinsics,float)
    centers=[]; projections=[]; rays=[]
    for pixels,position,rotation in [(a,position_a,rotation_a),(b,position_b,rotation_b)]:
        rotation=np.asarray(rotation,float);position=np.asarray(position,float)
        center=position+rotation@CAMERA_OFFSET_FLU
        optical_to_world=rotation@OPTICAL_TO_FLU
        projection=k@optical_to_world.T@np.column_stack((np.eye(3),-center))
        ray=np.column_stack((pixels,np.ones(len(pixels))))@np.linalg.inv(k).T@optical_to_world.T
        ray/=np.linalg.norm(ray,axis=1,keepdims=True)
        centers.append(center);projections.append(projection);rays.append(ray)
    cosine=np.einsum('ij,ij->i',*rays)
    valid=np.isfinite(a).all(1)&np.isfinite(b).all(1)&(cosine<np.cos(np.deg2rad(min_parallax_deg)))
    points=np.full((len(a),3),np.nan)
    for i in np.flatnonzero(valid):
        rows=[]
        for pixel,p in zip((a[i],b[i]),projections):
            rows.extend([pixel[0]*p[2]-p[0],pixel[1]*p[2]-p[1]])
        _,_,vt=np.linalg.svd(rows)
        if abs(vt[-1,3])<1e-10:
            valid[i]=False;continue
        point=vt[-1,:3]/vt[-1,3]
        for pixel,p,center,ray in zip((a[i],b[i]),projections,centers,(rays[0][i],rays[1][i])):
            q=p@np.r_[point,1.]
            distance=np.linalg.norm(point-center)
            if q[2]<=0 or not min_range<=distance<=max_range or np.linalg.norm(q[:2]/q[2]-pixel)>max_reprojection_px or np.dot(point-center,ray)<=0:
                valid[i]=False;break
        if valid[i]:points[i]=point
    return points[valid].astype('<f4'),valid
