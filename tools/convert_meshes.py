"""Expanded URDFs + vendor DAE/OBJ meshes -> assets/robots.json (decimated, base64). Run from repo root.

Usage: python3 tools/convert_meshes.py [robot ...]   (no args = all; named robots are merged into the existing json)
"""
import trimesh, numpy as np, base64, json, os, re, sys, xml.etree.ElementTree as ET, fast_simplification
PKGS={'ur_description':'vendor/Universal_Robots_ROS2_Description','fanuc_crx_description':'vendor/fanuc_description/fanuc_crx_description',
      'spot_description':'vendor/spot_description/spot_description'}
def resolve(fn):
    if fn.startswith('package://'):
        pkg,rest=fn[len('package://'):].split('/',1); return PKGS[pkg]+'/'+rest
    if fn.startswith('file://'): fn=fn[7:]
    return fn
def b64(a): return base64.b64encode(a.tobytes()).decode()
def obj_groups(path):
    # trimesh drops Spot's `mtllib ../../spot.mtl` (parent-relative), so read v/f/usemtl directly and colour from Kd.
    d=os.path.dirname(path); kd={}; V=[]; faces={}; cur=None
    for line in open(path):
        t=line.split()
        if not t: continue
        if t[0]=='mtllib':
            name=None
            for l in open(os.path.join(d,' '.join(t[1:]))):
                u=l.split()
                if u and u[0]=='newmtl': name=u[1]
                elif u and u[0]=='Kd': kd[name]=tuple(int(float(x)*255) for x in u[1:4])
        elif t[0]=='v': V.append([float(x) for x in t[1:4]])
        elif t[0]=='usemtl': cur=t[1]
        elif t[0]=='f':
            idx=[int(x.split('/')[0]) for x in t[1:]]; idx=[i-1 if i>0 else len(V)+i for i in idx]
            faces.setdefault(kd.get(cur,(102,102,102)),[]).extend([idx[0],idx[k],idx[k+1]] for k in range(1,len(idx)-1))
    V=np.array(V)
    return {c:[trimesh.Trimesh(V,np.array(F),process=False)] for c,F in faces.items()}
def mesh_payload(path, budget):
    if path.endswith('.obj'): groups=obj_groups(path)
    else:
        s=trimesh.load(path,force='scene')
        groups={}
        for node in s.graph.nodes_geometry:
            T,gname=s.graph[node]; g=s.geometry[gname].copy(); g.apply_transform(T)
            col=tuple(int(x) for x in g.visual.material.main_color[:3])
            groups.setdefault(col,[]).append(trimesh.Trimesh(g.vertices,g.faces,process=False))
    total=sum(sum(len(m.faces) for m in ms) for ms in groups.values())
    out=[]
    for col,ms in groups.items():
        m=trimesh.util.concatenate(ms); m.remove_unreferenced_vertices(); m.merge_vertices()
        n=len(m.faces); target=max(60,int(budget*n/total))
        V,F=m.vertices.astype(np.float32),m.faces.astype(np.int32)
        if n>target:
            V,F=fast_simplification.simplify(V,F,target_reduction=1-target/n)
        V=V.astype(np.float32); F=F.astype(np.uint16 if len(V)<65535 else np.uint32)
        out.append({'color':'#%02x%02x%02x'%col,'pos':b64(V),'idx':b64(F),'idx32':F.dtype==np.uint32})
    return out, total
def process(urdf_in, budget_per_link):
    t=ET.parse(urdf_in); r=t.getroot()
    for tag in ['ros2_control','gazebo','transmission']:
        for e in r.findall(tag): r.remove(e)
    meshes={}
    for link in r.findall('link'):
        for c in link.findall('collision')+link.findall('inertial'): link.remove(c)
        for v in link.findall('visual'):
            for m in v.iter('mesh'):
                fn=m.get('filename'); key=fn.split('/')[-3]+'/'+fn.split('/')[-1] if re.search('ur_description|fanuc|spot_description',fn) else fn
                p=resolve(fn)
                if key not in meshes:
                    meshes[key],tot=mesh_payload(p,budget_per_link); print(key,tot,'->',sum(len(base64.b64decode(g['idx']))//(4 if g['idx32'] else 2)//3 for g in meshes[key]))
                m.set('filename',key)
    return ET.tostring(r,encoding='unicode'), meshes
ROBOTS=[('ur10e','assets/urdf/ur10e.urdf'),('crx10ia','assets/urdf/crx10ia.urdf'),('spot','assets/urdf/spot.urdf')]
want=sys.argv[1:] or [n for n,_ in ROBOTS]
out=json.load(open('assets/robots.json')) if sys.argv[1:] else {}
for name,f in ROBOTS:
    if name not in want: continue
    urdf,meshes=process(f,2600)
    out[name]={'urdf':urdf,'meshes':meshes}
s=json.dumps(out); open('assets/robots.json','w').write(s); print(len(s)/1e6,'MB')
