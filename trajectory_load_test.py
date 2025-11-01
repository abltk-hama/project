# 例: tests/test_trajectory_loader_min.py
from math import isclose
from trajectory import trajectory_loader

def _find(elem_type, data):
    return [e for e in data["elements"] if e["type"].lower()==elem_type][0]

def test_rotate_scale_translate(tmp_path):
    # サンプルJSON
    p = tmp_path/"route.json"
    p.write_text("""
    {"trajectory_id":"t1","elements":[
      {"id":"L1","type":"line","start":[1,0,0],"end":[2,0,0],"speed":0.2},
      {"id":"A1","type":"arc","center":[0,1],"radius":1.0,"start_angle":0,"end_angle":90,"direction":"ccw","speed":0.2}
    ]}
    """, encoding="utf-8")

    loader = trajectory_loader.TrajectoryLoader()

    d = loader.load_json(str(p), rotation_deg=90, scale=2.0, translate=(1.0, -1.0))
    L1 = _find("line", d)
    A1 = _find("arc", d)

    # line: (x,y)->(-y,x), その後 scale=2, 最後に + (1,-1)
    # start(1,0) -> 回転(0,1) -> スケール(0,2) -> 並進(1,1)
    sx, sy = L1["start"][:2]
    assert isclose(sx, 1.0) and isclose(sy, 1.0)
    # arc: radius も 2倍
    assert isclose(A1["radius"], 2.0)
    # 角度は+90 -> [0,360) 正規化
    assert A1["start_angle"] == 90 and A1["end_angle"] == 180
    # 方向は大文字化
    assert A1["direction"] == "CCW"

test_rotate_scale_translate(tmp_path=__import__('pathlib').Path('.'))