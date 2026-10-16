"""Expanded URDFs + vendor DAE meshes -> assets/robots.json (decimated, base64). Run from repo root."""
import trimesh, numpy as np, base64, json, re, xml.etree.ElementTree as ET, fast_simplification
PKGS={'ur_description':'vendor/Universal_Robots_ROS2_Description','fanuc_crx_description':'vendor/fanuc_description/fanuc_crx_description'}
def resolve(fn):
    m=re.match(r'(?:package://|file://)?(?:.*?/)?((ur_description|fanuc_crx_description)/.*)',fn)
    if fn.startswith('package://'):
        pkg,rest=fn[len('package://'):].split('/',1); return PKGS[pkg]+'/'+rest
    if fn.startswith('file://'): fn=fn[7:]
    return fn
def b64(a): return base64.b64encode(a.tobytes()).decode()
def mesh_payload(path, budget):
    s=trimesh.load(path,force='scene')
    groups={}
    for node in s.graph.nodes_geometry:
        T,gname=s.graph[node]; g=s.geometry[gname].copy(); g.apply_transform(T)
        col=tuple(int(x) for x in g.visual.material.main_color[:3])
        groups.setdefault(col,[]).append(trimesh.Trimesh(g.vertices,g.faces,process=False))
    total=sum(sum(len(m.faces) for m in ms) for ms in groups.values())
    out=[]
    for col,ms in groups.items():
        m=trimesh.util.concatenate(ms); m.merge_vertices()
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
                fn=m.get('filename'); key=fn.split('/')[-3]+'/'+fn.split('/')[-1] if 'ur_description' in fn or 'fanuc' in fn else fn
                p=resolve(fn)
                if key not in meshes:
                    meshes[key],tot=mesh_payload(p,budget_per_link); print(key,tot,'->',sum(len(base64.b64decode(g['idx']))//(4 if g['idx32'] else 2)//3 for g in meshes[key]))
                m.set('filename',key)
    return ET.tostring(r,encoding='unicode'), meshes
out={}
for name,f in [('ur10e','assets/urdf/ur10e.urdf'),('crx10ia','assets/urdf/crx10ia.urdf')]:
    urdf,meshes=process(f,2600)
    out[name]={'urdf':urdf,'meshes':meshes}
s=json.dumps(out); open('assets/robots.json','w').write(s); print(len(s)/1e6,'MB')
