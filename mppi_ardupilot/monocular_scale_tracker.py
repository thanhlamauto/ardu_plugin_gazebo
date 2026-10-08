"""Estimate learned-depth scale from temporal RGB matches and scaled odometry.

No dense reference depth or obstacle geometry. This requires camera motion and
texture; absent evidence produces no scale estimate. Intrinsics must describe
the actual RGB image. Scale alone does not fix shape errors in learned depth.
"""
import numpy as np
from .monocular_triangulation import triangulate_matches,OPTICAL_TO_FLU,CAMERA_OFFSET_FLU

class MonocularScaleTracker:
    def __init__(self,intrinsics,min_tracks=12,min_baseline=.25,max_baseline=2.):
        self.k=np.asarray(intrinsics,float);self.min_tracks=min_tracks
        self.min_baseline=min_baseline;self.max_baseline=max_baseline;self.keyframe=None

    def observe(self,rgb,depth,position,rotation):
        import cv2
        cv2.setNumThreads(1)
        gray=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2GRAY)
        position=np.asarray(position,float);rotation=np.asarray(rotation,float)
        current=(gray,position.copy(),rotation.copy())
        if self.keyframe is None:
            self.keyframe=current;return dict(accepted=False,reason='need-motion')
        old,pa,ra=self.keyframe
        baseline=float(np.linalg.norm(position-pa))
        if baseline<self.min_baseline:return dict(accepted=False,reason='low-baseline',baseline_m=baseline)
        if baseline>self.max_baseline:
            self.keyframe=current;return dict(accepted=False,reason='excess-baseline',baseline_m=baseline)
        corners=cv2.goodFeaturesToTrack(old,maxCorners=600,qualityLevel=.005,minDistance=5)
        self.keyframe=current
        if corners is None:return dict(accepted=False,reason='no-texture',baseline_m=baseline)
        nxt,status,_=cv2.calcOpticalFlowPyrLK(old,gray,corners,None,winSize=(21,21),maxLevel=3)
        if nxt is None:return dict(accepted=False,reason='tracking-failed')
        back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,old,nxt,None,winSize=(21,21),maxLevel=3)
        if back is None:return dict(accepted=False,reason='reverse-tracking-failed')
        good=status[:,0].astype(bool)&reverse[:,0].astype(bool)&(np.linalg.norm(back[:,0]-corners[:,0],axis=1)<.7)
        a,b=corners[good,0],nxt[good,0]
        points,valid=triangulate_matches(a,b,pa,ra,position,rotation,self.k)
        pixels=b[valid];uv=np.rint(pixels).astype(int)
        h,w=np.asarray(depth).shape
        inside=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        points=points[inside];uv=uv[inside]
        center=position+rotation@CAMERA_OFFSET_FLU
        optical=(points-center)@rotation@OPTICAL_TO_FLU
        predicted=np.asarray(depth)[uv[:,1],uv[:,0]]
        mask=np.isfinite(predicted)&(predicted>.2)&(optical[:,2]>.3)
        ratios=optical[mask,2]/predicted[mask]
        ratios=ratios[np.isfinite(ratios)&(ratios>.1)&(ratios<10.)]
        if len(ratios)<self.min_tracks:return dict(accepted=False,reason='insufficient-metric-tracks',tracks=len(ratios),baseline_m=baseline)
        logs=np.log(ratios);median=np.median(logs);mad=float(np.median(abs(logs-median)))
        inliers=abs(logs-median)<max(.08,3*mad)
        if mad>.25 or int(inliers.sum())<self.min_tracks:
            return dict(accepted=False,reason='inconsistent-scale',tracks=len(ratios),log_mad=mad,baseline_m=baseline)
        return dict(accepted=True,scale=float(np.exp(np.median(logs[inliers]))),tracks=int(inliers.sum()),log_mad=mad,baseline_m=baseline)
