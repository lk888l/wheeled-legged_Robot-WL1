import numpy as np
import matplotlib
try:
    matplotlib.use('TkAgg')
except:
    pass
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

class LegKinematicsSim:
    def __init__(self):
        # 参数来源于 LegKinematics.hpp
        self.L_CD = 40.0  
        self.L_AB = 40.0  
        self.L_BD = 20.0  
        self.L_DW = 50.0  
        self.A_X = 15.0
        self.A_Y = -20.0

    def solve_fk(self, theta_deg):
        theta_rad = np.radians(theta_deg)
        # 1. C点为原点(0,0)，计算驱动杆末端 D 点
        D = np.array([self.L_CD * np.cos(theta_rad), 
                     self.L_CD * np.sin(theta_rad)])
        
        # 2. 计算 B 点坐标
        A = np.array([self.A_X, self.A_Y])
        dx, dy = D - A
        d2 = dx**2 + dy**2
        d = np.sqrt(d2)
        
        if d > (self.L_AB + self.L_BD) or d < abs(self.L_AB - self.L_BD):
            return None
            
        a = (self.L_AB**2 - self.L_BD**2 + d2) / (2 * d)
        h = np.sqrt(max(0, self.L_AB**2 - a**2))
        P2 = A + (a / d) * (D - A)
        
        # B点取正解
        B = np.array([P2[0] + (h / d) * dy, P2[1] - (h / d) * dx])
        
        # 3. 计算轮心 W (根据 BDW 共线逻辑)
        W = D + (self.L_DW / self.L_BD) * (D - B)
        return A, B, np.array([0,0]), D, W

# --- 交互界面设置 ---
sim = LegKinematicsSim()
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.25)

# 轨迹存储与状态
trace_x, trace_y = [], []
show_trace = True

# 绘图元素
line_cd, = ax.plot([], [], 'r-o', lw=3, label='Drive (CD)')
line_ab, = ax.plot([], [], 'b-o', lw=2, label='Upper (AB)')
line_bdw, = ax.plot([], [], 'g-o', lw=2, label='Leg (BDW)')
trace_line, = ax.plot([], [], 'm--', lw=1, alpha=0.6, label='Track') 
wheel_circ = plt.Circle((0, 0), 10, color='gray', fill=False, ls='--')
ax.add_patch(wheel_circ)

def update(val):
    theta = s_theta.val
    pts = sim.solve_fk(theta)
    if pts:
        A, B, C, D, W = pts
        line_cd.set_data([C[0], D[0]], [C[1], D[1]])
        line_ab.set_data([A[0], B[0]], [A[1], B[1]])
        line_bdw.set_data([B[0], D[0], W[0]], [B[1], D[1], W[1]])
        wheel_circ.set_center((W[0], W[1]))
        
        # 更新轨迹数据
        trace_x.append(W[0])
        trace_y.append(W[1])
        if show_trace:
            trace_line.set_data(trace_x, trace_y)
            
        # 实时显示 X 和 Y 坐标
        ax.set_title(f"Theta: {theta:.1f}° | Wheel X: {W[0]:.2f} mm | Y: {W[1]:.2f} mm")
    else:
        ax.set_title("OUT OF RANGE")
    fig.canvas.draw_idle()

# 按钮功能
def reset_trace(event):
    trace_x.clear()
    trace_y.clear()
    trace_line.set_data([], [])
    fig.canvas.draw_idle()

def toggle_trace(event):
    global show_trace
    show_trace = not show_trace
    if not show_trace:
        trace_line.set_data([], [])
    else:
        trace_line.set_data(trace_x, trace_y)
    btn_toggle.label.set_text("Hide Track" if show_trace else "Show Track")
    fig.canvas.draw_idle()

# 坐标轴设置
ax.set_xlim(-60, 100)
ax.set_ylim(140, -60) 
ax.set_aspect('equal')
ax.grid(True, linestyle=':')

# 控件布置
ax_slider = plt.axes([0.2, 0.12, 0.6, 0.03])
s_theta = Slider(ax=ax_slider, label='Theta ', valmin=0.0, valmax=120.0, valinit=45.0)
s_theta.on_changed(update)

ax_reset = plt.axes([0.2, 0.04, 0.25, 0.05])
btn_reset = Button(ax_reset, 'Clear Track', color='lightgray', hovercolor='orange')
btn_reset.on_clicked(reset_trace)

ax_toggle = plt.axes([0.55, 0.04, 0.25, 0.05])
btn_toggle = Button(ax_toggle, 'Hide Track', color='lightgray', hovercolor='lightblue')
btn_toggle.on_clicked(toggle_trace)

update(45.0)
plt.legend(loc='upper right')
plt.show()