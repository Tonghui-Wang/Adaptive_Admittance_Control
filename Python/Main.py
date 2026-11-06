"""Run the ECR5 constant-force contour and compare admittance tunings.

The tool approaches a virtual plane, holds 10 N, then keeps that force after
the surface stiffness jumps. Outputs are written as PNG files.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

import control
import robot_model

font_manager.fontManager.addfont("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc")
plt.rcParams["font.sans-serif"] = ["WenQuanYi Micro Hei"]
plt.rcParams["axes.unicode_minus"] = False

MODES = ("adaptive", "fixed", "heavy", "light")
LABELS = {
    "adaptive": "自适应",
    "fixed": "固定中等",
    "heavy": "固定偏重",
    "light": "固定偏轻",
}
COLORS = {
    "adaptive": "#0b6e4f",
    "fixed": "#1f4e79",
    "heavy": "#b85c38",
    "light": "#7a6a9a",
}


def rmse(result, start, stop):
    mask = (result["t"] >= start) & (result["t"] < stop)
    error = result["force"][mask] - result["force_des"][mask]
    return float(np.sqrt(np.mean(error ** 2)))


def run(out_dir):
    robot = robot_model.make_robot()
    reference = control.plan(robot, control.default_via_points())
    normal, plane_point = control.fit_plane(reference["p"])
    runs = {
        mode: control.simulate(robot, reference, normal, plane_point, mode=mode, seed=7)
        for mode in MODES
    }
    duration = float(reference["t"][-1])
    switch = 0.5 * duration
    out_dir.mkdir(parents=True, exist_ok=True)
    write_metrics(out_dir / "metrics.txt", runs, switch)
    plot_force(out_dir / "force.png", runs, switch)
    plot_gains(out_dir / "gains.png", runs["adaptive"], switch)
    plot_tangent(out_dir / "tangent.png", runs, switch)
    plot_path(out_dir / "path.png", runs["adaptive"], normal, plane_point)
    return runs, switch


def write_metrics(path, runs, switch):
    lines = [
        "ECR5 恒力轮廓跟踪",
        f"刚度在 t={switch:.2f}s 从 {control.K_SOFT:.0f} N/m 跳到 {control.K_STIFF:.0f} N/m",
        "",
        f"{'模式':<10}{'稳态力RMSE(N)':>16}{'跳变后1s RMSE':>16}{'切向RMSE(mm)':>16}",
    ]
    for mode in MODES:
        result = runs[mode]
        steady = rmse(result, 1.0, switch)
        recover = rmse(result, switch, switch + 1.0)
        tangent = 1000.0 * float(np.sqrt(np.mean(result["tangent"] ** 2)))
        lines.append(f"{LABELS[mode]:<10}{steady:16.3f}{recover:16.3f}{tangent:16.2f}")
    adaptive = runs["adaptive"]
    lines.append("")
    lines.append(
        f"自适应增益 起止 m={adaptive['m'][0]:.2f}->{adaptive['m'][-1]:.2f}  "
        f"b={adaptive['b'][0]:.1f}->{adaptive['b'][-1]:.1f}"
    )
    lines.append(
        f"刚度估计 接触后约 {adaptive['k_hat'][int(2.0 / control.DT)]:.0f} N/m，"
        f"结束 {adaptive['k_hat'][-1]:.0f} N/m"
    )
    text = "\n".join(lines) + "\n"
    path.write_text(text, encoding="utf-8")
    print(text, end="")


def plot_force(path, runs, switch):
    figure, axis = plt.subplots(figsize=(9, 4.2))
    desired = runs["adaptive"]
    axis.plot(desired["t"], desired["force_des"], "k--", lw=1.2, label="期望力")
    for mode in MODES:
        result = runs[mode]
        axis.plot(result["t"], result["force"], color=COLORS[mode], lw=1.3, label=LABELS[mode])
    axis.axvline(switch, color="#888888", ls=":", lw=1)
    axis.set_xlabel("时间 (s)")
    axis.set_ylabel("法向接触力 (N)")
    axis.set_title("接触力")
    axis.legend(ncol=3, frameon=False)
    axis.set_xlim(0, desired["t"][-1])
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_gains(path, result, switch):
    figure, axes = plt.subplots(3, 1, figsize=(9, 6.2), sharex=True)
    axes[0].plot(result["t"], result["m"], color=COLORS["adaptive"])
    axes[0].set_ylabel("质量 m (kg)")
    axes[1].plot(result["t"], result["b"], color=COLORS["adaptive"])
    axes[1].set_ylabel("阻尼 b (N·s/m)")
    axes[2].plot(result["t"], result["k_true"], color="#888888", ls="--", label="真实刚度")
    axes[2].plot(result["t"], result["k_hat"], color=COLORS["adaptive"], label="估计刚度")
    axes[2].set_ylabel("刚度 (N/m)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].legend(frameon=False)
    for axis in axes:
        axis.axvline(switch, color="#888888", ls=":", lw=1)
    axes[0].set_title("法向导纳参数与刚度估计")
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_tangent(path, runs, switch):
    figure, axis = plt.subplots(figsize=(9, 3.6))
    for mode in MODES:
        result = runs[mode]
        axis.plot(result["t"], result["tangent"] * 1000.0, color=COLORS[mode], lw=1.2, label=LABELS[mode])
    axis.axvline(switch, color="#888888", ls=":", lw=1)
    axis.set_xlabel("时间 (s)")
    axis.set_ylabel("切向误差 (mm)")
    axis.set_title("路径切向误差")
    axis.legend(ncol=4, frameon=False)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_path(path, result, normal, plane_point):
    figure = plt.figure(figsize=(6.4, 5.2))
    axis = figure.add_subplot(111, projection="3d")
    desired = result["p_ref"]
    actual = result["position"]
    axis.plot(desired[:, 0], desired[:, 1], desired[:, 2], color="#b85c38", ls="--", label="期望路径")
    axis.plot(actual[:, 0], actual[:, 1], actual[:, 2], color=COLORS["adaptive"], label="实际路径")
    tangent = np.eye(3) - np.outer(normal, normal)
    axis_u = tangent @ np.array([1.0, 0.0, 0.0])
    if np.linalg.norm(axis_u) < 0.2:
        axis_u = tangent @ np.array([0.0, 1.0, 0.0])
    axis_u = axis_u / np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    relative = actual - plane_point
    coord_u = relative @ axis_u
    coord_v = relative @ axis_v
    pad = 0.03
    uu, vv = np.meshgrid(
        np.linspace(coord_u.min() - pad, coord_u.max() + pad, 2),
        np.linspace(coord_v.min() - pad, coord_v.max() + pad, 2),
    )
    plane = plane_point + uu[..., None] * axis_u + vv[..., None] * axis_v
    axis.plot_surface(
        plane[:, :, 0], plane[:, :, 1], plane[:, :, 2],
        color="#9fb3c8", alpha=0.35, linewidth=0,
    )
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.set_title("末端路径与工作平面")
    axis.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description="ECR5 自适应导纳恒力仿真")
    parser.add_argument("--out", type=Path, default=Path("output"))
    args = parser.parse_args()
    run(args.out)


if __name__ == "__main__":
    main()
