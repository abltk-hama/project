import numpy as np

class TrajectoryLoader:
    def __init__(self, t_end=20, num_points=100):
        """ 軌道データを動的に生成 """
        self.t_end = t_end
        self.num_points = num_points
        self.t = np.linspace(0, t_end, num_points)

    def sine_wave(self, amplitude=5, frequency=0.2):
        """ サインカーブ軌道を生成 """
        x = self.t
        y = amplitude * np.sin(frequency * self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換
    
    def arc(self, scale=5, start_radians=0.0, range_radians=np.pi/2):
        """ 円弧軌道を生成 """
        t = range_radians * self.t / self.t_end
        x = scale * (1 - np.cos(t+start_radians))
        y = scale * np.sin(t+start_radians)
        return np.vstack([x, y]).T  # (N, 2) の形に変換

    def figure_eight(self, scale=5):
        """ 8の字軌道を生成 """
        x = scale * np.sin(self.t)
        y = scale * np.sin(self.t) * np.cos(self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換

    def spiral(self, scale=1):
        """ らせん軌道を生成 """
        x = scale * self.t * np.cos(self.t)
        y = scale * self.t * np.sin(self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換
    
    # ===== JSONからロードする新機能 =====
    def from_json(self, filename, num_points=50):
        """
        JSONで定義されたline/arcの集合を読み込み、座標点列を返す
        """
        with open(filename, "r", encoding="utf-8") as f:
            shapes = json.load(f)

        trajectory_points = []

        for shape in shapes:
            if shape["type"] == "line":
                start = np.array(shape["start"])
                end = np.array(shape["end"])
                for i in range(num_points):
                    t = i / (num_points - 1)
                    p = (1 - t) * start + t * end
                    trajectory_points.append(p)

            elif shape["type"] == "arc":
                cx, cy = shape["center"]
                r = shape["radius"]
                theta1 = np.radians(shape["start_angle"])
                theta2 = np.radians(shape["end_angle"])
                if shape.get("direction", "ccw") == "cw":
                    theta_values = np.linspace(theta1, theta2, num_points)[::-1]
                else:
                    theta_values = np.linspace(theta1, theta2, num_points)

                for theta in theta_values:
                    x = cx + r * np.cos(theta)
                    y = cy + r * np.sin(theta)
                    trajectory_points.append([x, y])

        return np.array(trajectory_points)
