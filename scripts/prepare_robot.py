"""Regenerate MuJoCo OBJ meshes from the bundled official COLLADA originals.

Build-time only: uv pip install trimesh pycollada. Runtime uses only the OBJs.
COLLADA scene transforms and its millimeter unit scale are both respected.
"""
from pathlib import Path
import hashlib,json
import collada,trimesh

root=Path(__file__).resolve().parents[1]/'assets/mycobot_pro_630'
derived=[]
for path in sorted(root.glob('*.dae')):
    dae=collada.Collada(str(path))
    if str(dae.assetInfo.upaxis)!='Z_UP':raise ValueError('Unexpected mesh up axis; conversion needs review')
    mesh=trimesh.load(path,force='scene').to_geometry()
    mesh.apply_scale(dae.assetInfo.unitmeter or 1.)
    out=path.with_suffix('.obj');mesh.export(out)
    derived.append(dict(file=out.name,source_file=path.name,unit_to_m=dae.assetInfo.unitmeter,
                        extent_m=mesh.extents.tolist(),faces=len(mesh.faces),sha256=hashlib.sha256(out.read_bytes()).hexdigest()))
    print(out.name,mesh.extents.round(5).tolist())
(root/'mesh_conversion.json').write_text(json.dumps(derived,indent=2)+'\n')
