MicroMVP GUI 系统开发规范 (PyQT6)

1. 界面整体布局 (Overall Layout)

界面采用左右分栏结构，整体基于 QHBoxLayout。

右侧：主画布 (Main Canvas)

占据主要显示区域。

基于 QGraphicsView 实现，用于显示工作空间（Workspace）、小车（Cars）以及额外绘图。

要求：支持自适应缩放（始终保持 WorkspaceConfig 的比例），居中显示。

左侧：控制与监测栏 (Sidebar)

宽度固定或可调节（建议固定最小宽度）。

布局采用 QVBoxLayout，包含两个主要模块：

动态控制面板 (Custom Control Panel, 顶部)：

必须具备滚动功能：使用 QScrollArea 包装，当控件过多时支持垂直滚动。

内容由 GUI_config 动态生成。

小车状态监测器 (Car Inspector, 底部)：

固定在左侧最下方。

用于显示当前选中（Click）的小车的详细信息（CarState 的各个字段）。

2. 动态控制面板配置 (Control Panel Config)

控件通过一个嵌套的 JSON/Dict 结构进行定义。每个控件必须指定唯一标识 callback_name。

控件类型与协议

控件类型 (type)配置参数 (config)回调返回值类型 (callback_value)Labeltext: str不触发回调 (仅显示文本)Togglelabel: str, default: boolbool (选中为 True, 反选为 False)Discrete Sliderlabel: str, tiers: list, default_idx: intAny (返回 tiers 列表中对应的元素值)Continuous Sliderlabel: str, range: [min, max], default: floatfloat (滑块当前的浮点值)Options (Dropdown)label: str, options: list, default: AnyAny (返回选中项在 options 里的原始值)Inputlabel: str, placeholder: strstr (按下回车键时返回输入框内的字符串)

示例配置数据结构：

Python



GUI_config = {

    "control_panel": [

        {"type": "label", "text": "System Controls"},

        {"type": "toggle", "label": "Enable Simulation", "callback_name": "sim_toggle", "default": True},

        {"type": "discrete_slider", "label": "Speed Tier", "tiers": [1, 2, 5, 10], "callback_name": "set_speed_tier"},

        {"type": "continuous_slider", "label": "K_p Gain", "range": [0.1, 2.0], "callback_name": "kp_adjust"},

        {"type": "options", "label": "Pattern", "options": ["circle", "8_shape"], "callback_name": "change_pattern"},

        {"type": "input", "label": "Robot Name", "callback_name": "rename_robot"}

    ]

}

3. 右侧画布交互与绘图 (Canvas & Primitives)

画布使用 保留模式（Retained Mode），通过 UUID 管理对象生命周期。

3.1 坐标变换逻辑 (Coordinate Transformation)

GUI 必须维护物理坐标（Workspace Units）到屏幕像素（Pixels）的映射：

映射系数 ($k$)：$\text{scale} = \min(\text{canvas\_w}/\text{ws\_w}, \text{canvas\_h}/\text{ws\_h})$。

Y 轴翻转：Workspace 原点在左下角（Y向上），PyQt 原点在左上角（Y向下）。

$Pixels.x = k \cdot Workspace.x$

$Pixels.y = \text{Height}_{pixel} - (k \cdot Workspace.y)$

角度补偿：小车朝向 $\theta = 0^\circ$ 为 $X$ 正方向。在 PyQt 中，旋转应对应为 setRotation(90 - theta)（视坐标系定义而定）。

3.2 基于 UUID 的绘图 API

gui.update(car_states, additional_drawings)

CarState: GUI 自动在画布上渲染小车，并在小车左上方悬浮显示 ID。

Additional Drawings: 这是一个 list[dict]。

处理机制：

检查 uuid 是否在 drawing_cache 中。

若存在，修改现有 QGraphicsItem 属性。

若不存在，创建新对象并存入缓存。

若缓存中的 uuid 不在当前列表，销毁对象。

4. API 接口定义

4.1 核心方法

gui.register_callback(callback_name, func)

作用：将 UI 控件触发的动作映射到后端的函数。

参数：callback_name 必须匹配配置中的名称；func 是接收对应参数的回调函数。

gui.update_car_inspector(car_id)

作用：更新左下角 Inspector 的显示。

触发：通常由画布点击小车的事件触发。

4.2 交互回调 (From GUI to Coordinator)

on_canvas_click(x, y)：返回点击处的 Workspace 物理坐标。

on_car_click(car_id)：返回被选中的小车 ID。

on_curve_drawn(points_list)：返回用户用鼠标在屏幕上画出的点列（已转换为物理坐标）。

5. 实现提示 (Implementation Tips)

Car Inspector：建议使用 QScrollArea 配合 QFormLayout。每当 gui.update 被调用时，如果当前有选中的车，实时刷新其数值。

UUID 性能：使用 Python 字典存储 item_cache。在 update 开始时复制一份 cache.keys() 用于差分计算。

小车图标偏移：绘图时必须考虑 WorkspaceConfig.offset_w 和 offset_h。小车矩形的左下角位置应该是 $(x - offset\_w, y - offset\_h)$ 经过旋转后的坐标。

You can goto /home/omen/junshan/micromvp_push/micromvp_v4/src/micromvp/core/models.py to see some important data structure you might want to use.


-----
"""
MicroMVP 系统集成示例 - main.py
该脚本演示了如何组装环境(Env)、控制器(Controllers)、协调器(Coordinator)和 GUI。
"""

import threading
import time
from typing import Dict

# 导入自定义的数据模型
from core.models import (
    WorkspaceConfig, RobotObservation, Action, 
    CarControllerConfig, CarState, WorldState
)

# 假设这些类已经实现( actully not, we write GUI first)
from environment import RealEnv  # 或者 SimEnv
from controllers import PurePursuitController
from coordinator import MultiCarCoordinator
from gui import MVPWindow  # 基于 PyQt6 的实现

def main():
    # --- 1. 环境初始化 ---
    # Env 是物理世界的唯一来源，它决定了所有的度量单位（如：1个单位 = 1像素）
    env = RealEnv() 
    ws_config: WorkspaceConfig = env.workspace_config

    # --- 2. 控制器初始化 ---
    # 为每辆小车创建一个独立的控制器实例
    ctrl_config = CarControllerConfig(lookahead_base_ratio=1.2)
    controllers: Dict[int, PurePursuitController] = {}
    
    for robot_id in ws_config.car_ids:
        controllers[robot_id] = PurePursuitController(robot_id, ctrl_config, ws_config)
def
    # --- 3. 协调器初始化 ---
    # Coordinator 负责全局调度，比如分配任务、避障逻辑
    coordinator = MultiCarCoordinator(ws_config, controllers)

    # --- 4. GUI 配置与初始化 (GUI 程序员重点关注) ---
    # GUI_config 决定了左侧面板生成哪些控件。
    # 每一个 'callback_name' 必须在后面通过 gui.register 进行绑定。
    gui_config = {
        "canvas": {
            "draw_curve_callback": True,  # 允许用户在画布上画线
            "click_car_callback": True    # 允许用户点击小车event sent back to coordinator. (by default user can always click a car to inspect its info on inspector)
        },
        "control_panel": [
            {"type": "label", "text": "=== 任务调度 ==="},
            {
                "type": "options", 
                "label": "选择阵型", 
                "options": ["Circle", "8-Shape", "Square"], 
                "default": "Circle",
                "callback_name": "cmd_change_pattern"
            },
            {
                "type": "continuous_slider", 
                "label": "全局巡航速度", 
                "range": [0.1, 1.0], 
                "default": 0.5,
                "callback_name": "cmd_set_speed"
            },
            {"type": "toggle", "label": "紧急避障", "default": True, "callback_name": "cmd_toggle_safety"},
            {"type": "input", "label": "发送指令", "placeholder": "输入调试代码...", "callback_name": "cmd_shell"}
        ]
    }

    # 实例化 GUI
    # GUI 需要 ws_config 来设置画布比例, gui_config to setup control panel ui
    gui = MVPWindow(gui_config, ws_config)

    # --- 5. 绑定回调接口 (桥接 GUI 与业务逻辑) ---
    # 当用户操作左侧面板控件时，GUI 会自动调用对应的 handler。
    # GUI 程序员应确保：UI 触发时传递的参数类型与 handler 接收的类型一致。
    gui.register_callback("cmd_change_pattern", coordinator.set_pattern)
    gui.register_callback("cmd_set_speed", coordinator.update_global_speed)
    gui.register_callback("cmd_toggle_safety", coordinator.toggle_safety)
    
    # 画布交互回调
    gui.register_callback("on_canvas_click", coordinator.handle_map_click)
    gui.register_callback("on_curve_drawn", coordinator.handle_user_path)

    # --- 6. 核心逻辑循环 (Logic Thread) ---
    def logic_loop():
        """
        这个线程以 WorkspaceConfig.frequency (如 20Hz) 运行。
        它是系统的心跳，负责 观测 -> 决策 -> 执行。
        """
        rate = ws_config.dt
        print(f"逻辑线程已启动，频率: {ws_config.frequency}Hz")
        
        while True:
            start_time = time.time()

            # A. 从环境获取原始观测
            obs_dict: Dict[int, RobotObservation] = env.get_observations()

            # B. 协调器处理观测，更新内部状态，并计算所有车的动作
            # 在 process 内部，各个控制器的 CarState 会被更新
            actions: Dict[int, Action] = coordinator.process(obs_dict)

            # C. 获取Car state and 额外的绘图指令 (UUID 机制)
            world_state = coordinator.gather_car_info()
            # GUI 程序员注意：这里返回的 list[dict] 将驱动右侧画布的动态显示
            additional_drawings = coordinator.get_additional_drawings()

            # D. 推送数据快照给 GUI 进行渲染
            # 注意：此处 gui.update 是非阻塞的，它只是更新 GUI 的内部缓存
            gui.update(world_state, additional_drawings)

            # E. 执行动作
            env.apply_actions(actions)

            # 控制频率
            elapsed = time.time() - start_time
            time.sleep(max(0, rate - elapsed))

    # 启动逻辑子线程
    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # --- 7. 启动 GUI 主循环 (Main Thread) ---
    # 在 PyQt6 中，GUI 必须在主线程运行。
    # 它会阻塞在这里，直到用户关闭窗口。
    gui.run()

if __name__ == "__main__":
    main()