from models.real_vehicle import RealVehicle
from models.physics import SimVehicle

# modeはconfigから指定
if MODE == "real":
    vehicle = RealVehicle(i2c_addr=0x08)
else:
    vehicle = SimVehicle()

# 経路追従側は同一呼び出し
vehicle.set_control(v=0.5, delta=0.1)
state = vehicle.get_state()