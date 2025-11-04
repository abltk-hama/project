# app/ui_app.py（抜粋イメージ）
import streamlit as st
from runner.runner_api import RunnerConfig, RouteTransform, DebugModelParam, Termination, run_mission

st.sidebar.header("Route")
route_path = st.sidebar.text_input("Route JSON", "routes/route_square.json")
rot = st.sidebar.number_input("rotation_deg", 0.0)
scale = st.sidebar.number_input("scale", 1.0)
tx = st.sidebar.number_input("translate.x", 0.0)
ty = st.sidebar.number_input("translate.y", 0.0)
step_m = st.sidebar.number_input("step_m", 0.10)

st.sidebar.header("Model")
model = st.sidebar.selectbox("model", ["debug","physics","real"])
x0 = st.sidebar.number_input("x0", 0.0); y0 = st.sidebar.number_input("y0", 0.0); yaw0 = st.sidebar.number_input("yaw0(rad)", 0.0)
dt = st.sidebar.number_input("dt", 0.02); tau_pos = st.sidebar.number_input("tau_pos", 0.30); tau_yaw = st.sidebar.number_input("tau_yaw", 0.25)

st.sidebar.header("Termination / Log")
pos_tol = st.sidebar.number_input("pos_tol", 0.03); yaw_tol_deg = st.sidebar.number_input("yaw_tol_deg", 5.0); hold_steps = st.sidebar.number_input("hold_steps", 8)
render_xy = st.sidebar.checkbox("Plot XY", True); render_dist = st.sidebar.checkbox("Plot Distance", True); render_theta = st.sidebar.checkbox("Plot Heading", True)
save_csv = st.sidebar.text_input("save_csv (optional)", "")

if st.button("Start"):
    cfg = RunnerConfig(
        route_path=route_path,
        transform=RouteTransform(rotation_deg=rot, scale=scale, translate=(tx,ty), step_m=step_m),
        initial_pose=(x0,y0,yaw0),
        model=model,
        debug_param=DebugModelParam(dt=dt, tau_pos=tau_pos, tau_yaw=tau_yaw),
        termination=Termination(pos_tol=pos_tol, yaw_tol_deg=yaw_tol_deg, hold_steps=int(hold_steps)),
        render_xy=render_xy, render_dist=render_dist, render_theta=render_theta,
        save_csv=(save_csv or None)
    )
    result = run_mission(cfg)
    st.success("Completed")
    # result["logs"] を用いて可視化（XY/距離/姿勢）＆CSV保存
