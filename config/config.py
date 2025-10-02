import configparser

class Config:
    def __init__(self, filename="C:/Users/nibgc/Documents/project/project/config.ini"):
        self.config = configparser.ConfigParser()
        # ✅ UTF-8 でファイルを開いて読み込む
        with open(filename, "r", encoding="utf-8") as file:
            self.config.read_file(file)

    def get_vehicle_params(self):
        """ 車両のパラメータを取得 """
        params = {
            "wheel_base": float(self.config["VEHICLE"]["wheel_base"]),
            "wheel_radius": float(self.config["VEHICLE"]["wheel_radius"]),
            "motor_max_rpm": float(self.config["VEHICLE"]["motor_max_rpm"]),
            "dt": float(self.config["SIMULATION"]["dt"]),
            "st": float(self.config["SIMULATION"]["st"]),
            "pt": float(self.config["SIMULATION"]["pt"])
        }
        return params
