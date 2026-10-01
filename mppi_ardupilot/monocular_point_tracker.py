"""Occupied points from temporal images of a single calibrated RGB camera."""
import numpy as np
from .monocular_triangulation import triangulate_matches

class MonocularPointTracker:
    def __init__(self,intrinsics,min_baseline=.18,max_baseline=1.2,min_valid_points=12):
        if min_valid_points<1:
            raise ValueError('min_valid_points must be positive')
        self.k=np.asarray(intrinsics,float);self.min_baseline=min_baseline
        self.max_baseline=max_baseline;self.min_valid_points=min_valid_points;self.keyframe=None

    def observe(self,rgb,position,rotation):
        import cv2
        cv2.setNumThreads(1)
        gray=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2GRAY)
        position=np.asarray(position,float);rotation=np.asarray(rotation,float)
        current=(gray,position.copy(),rotation.copy())
        empty=np.empty((0,3),dtype='<f4')
        if self.keyframe is None:
            self.keyframe=current;return empty,dict(reason='need-motion')
        old,pa,ra=self.keyframe;baseline=float(np.linalg.norm(position-pa))
        if baseline<self.min_baseline:return empty,dict(reason='low-baseline',baseline_m=baseline)
        if baseline>self.max_baseline:
            self.keyframe=current
            return empty,dict(reason='excess-baseline',baseline_m=baseline)
        corners=cv2.goodFeaturesToTrack(old,maxCorners=1600,qualityLevel=.002,minDistance=3)
        if corners is None:return empty,dict(reason='no-texture',baseline_m=baseline)
        nxt,status,_=cv2.calcOpticalFlowPyrLK(old,gray,corners,None,winSize=(21,21),maxLevel=3)
        if nxt is None:return empty,dict(reason='tracking-failed')
        back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,old,nxt,None,winSize=(21,21),maxLevel=3)
        if back is None:return empty,dict(reason='reverse-tracking-failed')
        good=status[:,0].astype(bool)&reverse[:,0].astype(bool)&(np.linalg.norm(back[:,0]-corners[:,0],axis=1)<.5)
        a,b=corners[good,0],nxt[good,0]
        points,valid=triangulate_matches(a,b,pa,ra,position,rotation,self.k,min_parallax_deg=1.,max_reprojection_px=.7,max_range=20.)
        promoted=int(valid.sum())>=self.min_valid_points
        if promoted:self.keyframe=current
        return points,dict(reason='triangulated' if promoted else 'insufficient-valid-points',
                           baseline_m=baseline,features=len(corners),matched=int(good.sum()),
                           valid_points=int(valid.sum()),keyframe_promoted=promoted)
