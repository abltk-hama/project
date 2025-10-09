# [MODULE]: sim
[PURPOSE]:
  シミュレーションおよび実機動作検証の統合管理を行う階層。  
  `simulation.py` では `VehicleInterface` を介して、  
  シミュレーション(`Physics`) と実機(`RealVehicle`)を切り替えられる。

## [FILES & RESPONSIBILITIES]  

### simulation.py  
- 経路追従・目的地点制御の実行エントリ。  
- `use_real_vehicle` フラグにより制御対象を切り替える。

#### 切替例
```python
if use_real_vehicle:
    from models.real_vehicle.real_vehicle import RealVehicle as Vehicle
else:
    from models.physics import Physics as Vehicle

vehicle = Vehicle()
for t in range(N):
    vehicle.update_state(delta, v)
    state = vehicle.get_state()
```

#### 評価関数・補正制御  
- PSO適応制御、強化学習制御を含む。  
- 誤差低減とハンドリングスムーズ化の評価を行う。

---

### sim_env.py（予定）  
- 環境シミュレーションやノイズ注入用のユーティリティを実装予定。

---

## [INTERFACE DEPENDENCY]
```
simulation.py
 ├─> models.interfaces.VehicleInterface
 ├─> models.physics.Physics
 └─> models.real_vehicle.RealVehicle
```

---

## [UPDATE LOG]
- **2025-10-09**  
  - `use_real_vehicle` フラグ説明/コード例追加。  
  - シミュレーション(`Physics`) と実機(`RealVehicle`)切り替え機能追加。  