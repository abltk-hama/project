import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
class Visualization():
    def generate_plots(self, logger):
        """
        ログデータを使用して静的なプロットを生成する。
        """
        fig, ax = plt.subplots()

        # ログデータから情報取得
        x_ref, y_ref = np.array(logger["reference_path"]).T
        x_traj, y_traj = np.array(logger["trajectory"]).T

        # プロット
        ax.plot(x_ref, y_ref, 'k--', label="Reference Path")
        ax.plot(x_traj, y_traj, 'r-', label="Actual Path")

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.legend()
        ax.grid()

        # 画像保存
        plt.savefig("trajectory_plot.png")
        plt.show()

    def create_animation(self, logger, output_path="trajectory_animation.mp4", fps=30):
        """
        ログデータを使用してアニメーションを生成する。
        """
        fig, ax = plt.subplots()
        x_ref, y_ref = np.array(logger["reference_path"]).T
        x_traj, y_traj = np.array(logger["trajectory"]).T

        ax.plot(x_ref, y_ref, 'k--', label="Reference Path")
        line, = ax.plot([], [], 'r-', label="Actual Path")

        def update(frame):
            line.set_data(x_traj[:frame], y_traj[:frame])
            return line,

        ani = animation.FuncAnimation(fig, update, frames=len(x_traj), interval=100, blit=True)

        # 動画保存
        ani.save(output_path, fps=fps, writer="ffmpeg")
        plt.show()

    def plot_simulation(self, trajectory, state_history):
        """
        シミュレーション結果をプロット
        """
        trajectory_x, trajectory_y = zip(*trajectory)
        state_x, state_y = zip(*state_history)

        plt.figure(figsize=(8, 6))
        plt.plot(trajectory_x, trajectory_y, 'k--', label="Reference Path")  # 目標経路（破線）
        plt.plot(state_x, state_y, 'r-', label="Actual Path")  # 追従した軌道
        plt.legend()
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Trajectory Tracking Simulation")
        plt.grid(True)
        plt.show()