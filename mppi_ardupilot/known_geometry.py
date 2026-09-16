"""Explicit known SDF geometry, separate from observed LiDAR. Level/yaw primitives."""
import hashlib
import numpy as np
from .global_planner import load_sdf_obstacles
class KnownGeometry:
    def __init__(self,path):
        from pathlib import Path
        self.path=Path(path).resolve()
        self.obstacles=load_sdf_obstacles(self.path)
        self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
    def clearance(self,points):
        p=np.asarray(points);out=np.full(p.shape[:-1],np.inf)
        for o in self.obstacles:
            d=p[...,:2]-o.center_xy;c,s=np.cos(o.yaw),np.sin(o.yaw)
            xy=d@np.array([[c,-s],[s,c]])
            horizontal=np.linalg.norm(np.maximum(np.abs(xy)-o.half_size_xy,0),axis=-1) if o.kind=='box' else np.maximum(np.linalg.norm(xy,axis=-1)-o.radius,0)
            z=np.maximum(np.maximum(o.z_min-p[...,2],p[...,2]-o.z_max),0)
            out=np.minimum(out,np.hypot(horizontal,z))
        return out
    def torch_clearance(self,p):
        import torch
        # Vectorize primitives to avoid one GPU/CPU dispatch per solid per step.
        boxes=[o for o in self.obstacles if o.kind=='box']
        cylinders=[o for o in self.obstacles if o.kind!='box']
        out=torch.full(p.shape[:-1],float('inf'),dtype=p.dtype,device=p.device)
        for group,box in [(boxes,True),(cylinders,False)]:
            if not group:continue
            q=torch.as_tensor([[*o.center_xy,o.z_min,o.z_max,o.yaw,*o.half_size_xy,o.radius] for o in group],dtype=p.dtype,device=p.device)
            d=p[...,None,:2]-q[:,:2];c,s=torch.cos(q[:,4]),torch.sin(q[:,4])
            xy=torch.stack((c*d[...,0]+s*d[...,1],-s*d[...,0]+c*d[...,1]),-1)
            h=torch.linalg.vector_norm((xy.abs()-q[:,5:7]).clamp_min(0),dim=-1) if box else (torch.linalg.vector_norm(xy,dim=-1)-q[:,7]).clamp_min(0)
            z=torch.maximum(q[:,2]-p[...,None,2],p[...,None,2]-q[:,3]).clamp_min(0)
            out=torch.minimum(out,torch.sqrt(h*h+z*z).min(-1).values)
        return out
    def torch_segments_safe(self,starts,ends,radius):
        """Conservative continuous segment check against radius-expanded solids.

        Boxes use a slab test against their expanded local AABB. Cylinders use
        simultaneous XY swept-disc and Z-interval overlap; that combination is
        conservative near the cylinder caps. The same predicate is used by
        sample masking and the final gate.
        """
        import torch
        a=torch.as_tensor(starts);b=torch.as_tensor(ends,dtype=a.dtype,device=a.device)
        safe=torch.ones(a.shape[:-1],dtype=torch.bool,device=a.device);eps=1e-12
        for objects,is_box in (([o for o in self.obstacles if o.kind=='box'],True),
                               ([o for o in self.obstacles if o.kind!='box'],False)):
            if not objects:continue
            q=torch.as_tensor([[*o.center_xy,o.z_min,o.z_max,o.yaw,
                                *o.half_size_xy,o.radius] for o in objects],
                              dtype=a.dtype,device=a.device)
            def local(p):
                d=p[...,None,:2]-q[:,:2]
                c,s=torch.cos(q[:,4]),torch.sin(q[:,4])
                return torch.stack((c*d[...,0]+s*d[...,1],
                    -s*d[...,0]+c*d[...,1],p[...,None,2].expand_as(d[...,0])),-1)
            la,lb=local(a),local(b);delta=lb-la
            if is_box:
                lower=torch.stack((-q[:,5]-radius,-q[:,6]-radius,q[:,2]-radius),-1)
                upper=torch.stack((q[:,5]+radius,q[:,6]+radius,q[:,3]+radius),-1)
                parallel=delta.abs()<eps
                outside_parallel=parallel&((la<lower)|(la>upper))
                inv=torch.where(parallel,torch.ones_like(delta),1/delta)
                t1=(lower-la)*inv;t2=(upper-la)*inv
                entry=torch.where(parallel,torch.full_like(t1,-torch.inf),torch.minimum(t1,t2))
                exit=torch.where(parallel,torch.full_like(t1,torch.inf),torch.maximum(t1,t2))
                hit=(~outside_parallel.any(-1)&
                     (exit.min(-1).values>=entry.max(-1).values.clamp_min(0))&
                     (entry.max(-1).values<=1))
            else:
                dxy=lb[...,:2]-la[...,:2]
                denom=(dxy*dxy).sum(-1).clamp_min(eps)
                t=(-(la[...,:2]*dxy).sum(-1)/denom).clamp(0,1)
                xy=la[...,:2]+t.unsqueeze(-1)*dxy
                xy_hit=torch.linalg.vector_norm(xy,dim=-1)<=q[:,7]+radius
                zlo=torch.minimum(la[...,2],lb[...,2]);zhi=torch.maximum(la[...,2],lb[...,2])
                hit=xy_hit&(zhi>=q[:,2]-radius)&(zlo<=q[:,3]+radius)
            safe &= ~hit.any(-1)
        return safe
    def validate(self,path,radius):
        # Distance is 1-Lipschitz: subtract half sample spacing for conservative bound.
        path=np.asarray(path);minimum=float('inf')
        if not np.isfinite(path).all():return {'valid':False,'reason':'nonfinite_map_path'}
        for a,b in zip(path[:-1],path[1:]):
            n=max(1,int(np.ceil(np.linalg.norm(b-a)/.1)))
            pts=a+np.linspace(0,1,n+1)[:,None]*(b-a)
            minimum=min(minimum,float(self.clearance(pts).min())-float(np.linalg.norm(b-a))/(2*n))
        return {'valid':minimum>radius,'min_clearance_lower_bound_m':minimum,'reason':'known_sdf_geometry', 'source':str(self.path),'sha256':self.sha256}
