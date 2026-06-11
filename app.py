import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import math
import json
import os
import random
import time
from datetime import datetime
import pandas as pd
import plotly.graph_objects as go

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机智能监控系统", page_icon="🛰️", layout="wide")

# ==================== 南京科技职业学院坐标 ====================
CAMPUS = [32.234097, 118.749413]
OBSTACLE_CONFIG_FILE = "obstacle_config.json"
WAYPOINT_CONFIG_FILE = "waypoint_config.json"

# ==================== 初始化 ====================
if 'obstacles' not in st.session_state:
    if os.path.exists(OBSTACLE_CONFIG_FILE):
        try:
            with open(OBSTACLE_CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                st.session_state.obstacles = data.get('obstacles', [])
        except:
            st.session_state.obstacles = []
    else:
        st.session_state.obstacles = []

if 'point_a' not in st.session_state:
    if os.path.exists(WAYPOINT_CONFIG_FILE):
        try:
            with open(WAYPOINT_CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                st.session_state.point_a = data.get('point_a', [32.2323, 118.749])
                st.session_state.point_b = data.get('point_b', [32.2344, 118.749])
        except:
            st.session_state.point_a = [32.2323, 118.749]
            st.session_state.point_b = [32.2344, 118.749]
    else:
        st.session_state.point_a = [32.2323, 118.749]
        st.session_state.point_b = [32.2344, 118.749]

if 'flight_alt' not in st.session_state:
    st.session_state.flight_alt = 20
if 'safe_radius' not in st.session_state:
    st.session_state.safe_radius = 10
if 'bypass_distance' not in st.session_state:
    st.session_state.bypass_distance = 15
if 'route_plans' not in st.session_state:
    st.session_state.route_plans = []
if 'selected_plan' not in st.session_state:
    st.session_state.selected_plan = None
if 'confirmed_plan' not in st.session_state:
    st.session_state.confirmed_plan = None
if 'temp_obs' not in st.session_state:
    st.session_state.temp_obs = None
if 'temp_height' not in st.session_state:
    st.session_state.temp_height = 50
if 'temp_name' not in st.session_state:
    st.session_state.temp_name = "建筑物"
if 'show_height_panel' not in st.session_state:
    st.session_state.show_height_panel = False

# 飞行模拟相关
if "flight_sim_running" not in st.session_state:
    st.session_state.flight_sim_running = False
if "flight_sim_start_time" not in st.session_state:
    st.session_state.flight_sim_start_time = None
if "flight_sim_current_index" not in st.session_state:
    st.session_state.flight_sim_current_index = 0
if "flight_sim_speed" not in st.session_state:
    st.session_state.flight_sim_speed = 8.5
if "flight_sim_waypoints" not in st.session_state:
    st.session_state.flight_sim_waypoints = []
if "flight_sim_total_distance" not in st.session_state:
    st.session_state.flight_sim_total_distance = 0
if "flight_sim_segment_distances" not in st.session_state:
    st.session_state.flight_sim_segment_distances = []
if "flight_sim_last_wp_index" not in st.session_state:
    st.session_state.flight_sim_last_wp_index = -1

# 通信日志相关
if "comm_logs_business" not in st.session_state:
    st.session_state.comm_logs_business = []
if "comm_logs_gcs_to_fcu" not in st.session_state:
    st.session_state.comm_logs_gcs_to_fcu = []
if "comm_logs_fcu_to_gcs" not in st.session_state:
    st.session_state.comm_logs_fcu_to_gcs = []

# ==================== 保存函数 ====================
def save_waypoints():
    data = {'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
            'point_a': st.session_state.point_a, 
            'point_b': st.session_state.point_b}
    with open(WAYPOINT_CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def save_obstacles_to_file():
    data = {'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
            'obstacles': st.session_state.obstacles, 
            'count': len(st.session_state.obstacles)}
    with open(OBSTACLE_CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_CONFIG_FILE):
        with open(OBSTACLE_CONFIG_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
            st.session_state.obstacles = data.get('obstacles', [])
        return True
    return False

# ==================== 通信日志辅助函数 ====================
def add_business_log(message, source="OBC 内部", color="green"):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    st.session_state.comm_logs_business.append({
        "timestamp": timestamp,
        "message": message,
        "source": source,
        "color": color
    })

def add_gcs_to_fcu_log(message):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    st.session_state.comm_logs_gcs_to_fcu.append(f"[{timestamp}] {message}")

def add_fcu_to_gcs_log(message):
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    st.session_state.comm_logs_fcu_to_gcs.append(f"[{timestamp}] {message}")

def clear_all_logs():
    st.session_state.comm_logs_business = []
    st.session_state.comm_logs_gcs_to_fcu = []
    st.session_state.comm_logs_fcu_to_gcs = []

# ==================== 心跳模拟器 ====================
class HeartbeatSimulator:
    def __init__(self):
        self.running = False
        self.last_time = None
        self.offline = False
        self.history = []
    
    def start(self):
        self.running = True
        self.offline = False
        self.history = []
        self.last_time = time.time()
    
    def stop(self):
        self.running = False
    
    def update(self):
        if not self.running:
            return None
        current = time.time()
        elapsed = current - self.last_time
        if elapsed >= 1:
            self.last_time = current
            heartbeat = {'id': len(self.history) + 1, 
                        'time': datetime.now().strftime("%H:%M:%S"), 
                        'status': 'alive', 
                        'delay': round(random.uniform(5, 50), 2)}
            self.history.append(heartbeat)
            if len(self.history) > 50:
                self.history.pop(0)
            return heartbeat
        if elapsed > 3 and not self.offline:
            self.offline = True
            timeout = {'id': len(self.history) + 1, 
                      'time': datetime.now().strftime("%H:%M:%S"), 
                      'status': 'timeout', 
                      'delay': 0}
            self.history.append(timeout)
            return timeout
        elif elapsed <= 3 and self.offline:
            self.offline = False
        return None
    
    def get_stats(self):
        if not self.history:
            return {'total': 0, 'timeout': 0, 'rate': 100}
        total = len(self.history)
        timeout = sum(1 for h in self.history if h['status'] == 'timeout')
        return {'total': total, 'timeout': timeout, 'rate': round((total-timeout)/total*100, 1)}
    
    def get_history(self):
        return self.history.copy()

if 'heartbeat_sim' not in st.session_state:
    st.session_state.heartbeat_sim = HeartbeatSimulator()
if 'heartbeat_running' not in st.session_state:
    st.session_state.heartbeat_running = False

# ==================== 核心几何计算（修复穿障检测） ====================
def calc_distance(p1, p2):
    """计算两点球面距离(米) WGS84"""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    R = 6371000
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def point_in_polygon(point, poly):
    """判断点是否在多边形内部"""
    x, y = point[1], point[0]
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][1], poly[i][0]
        x2, y2 = poly[(i+1)%n][1], poly[(i+1)%n][0]
        if ((y1 > y) != (y2 > y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1):
            inside = not inside
    return inside

def line_intersect_polygon(start, end, poly, sample_num=100):
    """
    【修复1】超密采样线段穿障检测，杜绝漏判
    """
    for i in range(sample_num + 1):
        t = i / sample_num
        curr_lat = start[0] + (end[0] - start[0]) * t
        curr_lon = start[1] + (end[1] - start[1]) * t
        if point_in_polygon([curr_lat, curr_lon], poly):
            return True
    return False

def calculate_distances(waypoints):
    total = 0
    segment_distances = []
    for i in range(len(waypoints) - 1):
        dist = calc_distance(waypoints[i], waypoints[i+1])
        segment_distances.append(dist)
        total += dist
    return total, segment_distances

def get_polygon_bounds(points):
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    return min(lats), max(lats), min(lons), max(lons)

def meter_to_degree(meter):
    return meter / 111000.0

# ==================== 障碍物类（修复绕行点生成） ====================
class Obstacle:
    def __init__(self, points, height, name):
        self.points = points
        self.height = height
        self.name = name
        self.min_lat, self.max_lat, self.min_lon, self.max_lon = get_polygon_bounds(points)
        self.center_lat = (self.min_lat + self.max_lat) / 2
        self.center_lon = (self.min_lon + self.max_lon) / 2
        self.width_lat = self.max_lat - self.min_lat
        self.width_lon = self.max_lon - self.min_lon
        
    def to_dict(self):
        return {'points': self.points, 'height': self.height, 'name': self.name}
    
    @classmethod
    def from_dict(cls, data):
        return cls(data['points'], data['height'], data['name'])
    
    def contains(self, point):
        return point_in_polygon(point, self.points)
    
    def line_intersects(self, start, end):
        return line_intersect_polygon(start, end, self.points)

    def get_corner_bypass_points(self, start, end, safe_radius, bypass_distance):
        """
        【修复2】基于障碍物角点生成绕行点，保证在障碍物外侧
        """
        total_safe = safe_radius + bypass_distance
        offset_deg = meter_to_degree(total_safe)

        # 航线方向向量
        dx = end[1] - start[1]
        dy = end[0] - start[0]
        length = math.hypot(dx, dy)
        if length < 1e-10:
            dx, dy = 0, 1
        else:
            dx /= length
            dy /= length

        # 垂直法向量
        perp_left_x = -dy
        perp_left_y = dx
        perp_right_x = dy
        perp_right_y = -dx

        # 取障碍物四个角点，向外偏移
        corners = [
            [self.min_lat, self.min_lon],  # 西南
            [self.min_lat, self.max_lon],  # 东南
            [self.max_lat, self.max_lon],  # 东北
            [self.max_lat, self.min_lon]   # 西北
        ]

        # 为每个角点生成偏移点
        bypass_candidates = []
        for corner in corners:
            # 向四个方向偏移
            bypass_candidates.append([corner[0] + perp_left_y * offset_deg, corner[1] + perp_left_x * offset_deg])
            bypass_candidates.append([corner[0] + perp_right_y * offset_deg, corner[1] + perp_right_x * offset_deg])
            bypass_candidates.append([corner[0] + dy * offset_deg * 1.5, corner[1] + dx * offset_deg * 1.5])
            bypass_candidates.append([corner[0] - dy * offset_deg * 1.5, corner[1] - dx * offset_deg * 1.5])

        # 筛选出所有在障碍物外的点
        valid_candidates = [pt for pt in bypass_candidates if not self.contains(pt)]

        # 按到起点的距离排序，优先选离航线方向近的点
        valid_candidates.sort(key=lambda p: calc_distance(p, start))

        return {
            'left': valid_candidates[0] if valid_candidates else [self.center_lat, self.center_lon + offset_deg],
            'right': valid_candidates[1] if len(valid_candidates)>=2 else [self.center_lat, self.center_lon - offset_deg],
            'front': valid_candidates[2] if len(valid_candidates)>=3 else [self.center_lat + offset_deg, self.center_lon],
            'back': valid_candidates[3] if len(valid_candidates)>=4 else [self.center_lat - offset_deg, self.center_lon]
        }

# ==================== 路径规划（修复高度判断+绕障校验） ====================
def find_blocking_obstacles(start, end, obstacles, flight_alt):
    """
    【修复3】严格判断：只有飞行高度 < 障碍物高度 且 航线穿障，才需要绕行
    """
    blocking = []
    for obs_data in obstacles:
        obs = Obstacle.from_dict(obs_data) if isinstance(obs_data, dict) else obs_data

        # 高度足够，直接跳过
        if flight_alt >= obs.height:
            continue

        # 高度不足 + 航线穿障 → 必须绕行
        if obs.line_intersects(start, end):
            blocking.append(obs)
    return blocking

def plan_path_with_bypass(start, end, obstacles, flight_alt, safe_radius, bypass_distance, side):
    """
    【修复4】路径规划后，强制校验是否还穿障，穿障则自动修正
    """
    waypoints = [start]
    current_start = start
    current_end = end

    blocking_obstacles = find_blocking_obstacles(current_start, current_end, obstacles, flight_alt)
    
    if not blocking_obstacles:
        waypoints.append(end)
        return waypoints

    # 按距离起点远近排序
    blocking_obstacles.sort(key=lambda obs: calc_distance([obs.center_lat, obs.center_lon], start))

    for obs in blocking_obstacles:
        bypass_points = obs.get_corner_bypass_points(current_start, current_end, safe_radius, bypass_distance)
        bypass_pt = bypass_points.get(side, bypass_points['left'])
        waypoints.append(bypass_pt)
        current_start = bypass_pt

    waypoints.append(end)

    # 二次校验：如果路径仍然穿障，自动调整绕行点
    final_path = [start]
    for i in range(len(waypoints)-1):
        seg_start = waypoints[i]
        seg_end = waypoints[i+1]
        # 检查这段线段是否穿障
        still_blocking = find_blocking_obstacles(seg_start, seg_end, obstacles, flight_alt)
        if not still_blocking:
            final_path.append(seg_end)
            continue
        # 如果还穿障，强制添加额外绕行点
        obs = still_blocking[0]
        extra_pt = [obs.center_lat + meter_to_degree(50), obs.center_lon + meter_to_degree(50)]
        final_path.append(extra_pt)
        final_path.append(seg_end)

    return final_path

# ==================== 标题 ====================
st.title("🛰️ 无人机智能监控系统")
st.markdown("**南京科技职业学院** | 低于障碍物高度自动绕行 | 高于障碍物高度直接飞越")
st.markdown("---")

# ==================== 侧边栏 ====================
with st.sidebar:
    st.subheader("🌐 坐标系")
    coord_type = st.selectbox("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"])
    st.markdown("---")
    st.subheader("📊 系统状态")
    st.success("✅ 系统正常")

# ==================== 标签页 ====================
tab1, tab2 = st.tabs(["🗺️ 航线规划", "📡 飞行监控"])

# ==================== Tab 1: 航线规划 ====================
with tab1:
    col_left, col_right = st.columns([1.5, 1])
    
    with col_left:
        st.subheader("🗺️ 卫星地图")
        
        m = folium.Map(location=CAMPUS, zoom_start=17, control_scale=True)
        folium.TileLayer('https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
                         attr='高德卫星', subdomains=['1','2','3','4']).add_to(m)
        folium.TileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
                         attr='OpenStreetMap', name='街道地图').add_to(m)
        folium.Marker(CAMPUS, popup="🏫 南京科技职业学院", icon=folium.Icon(color='red')).add_to(m)
        
        alt = st.session_state.flight_alt
        for obs_data in st.session_state.obstacles:
            if isinstance(obs_data, dict):
                points = obs_data['points']
                height = obs_data['height']
                name = obs_data['name']
            else:
                points = obs_data.points
                height = obs_data.height
                name = obs_data.name
            
            color = 'red' if alt < height else 'green'
            folium.Polygon(points, color=color, weight=2, fill=True, 
                          fill_color=color, fill_opacity=0.3,
                          popup=f"{name}\n高度: {height}m\n当前飞行高度: {alt}m").add_to(m)
        
        folium.Marker(st.session_state.point_a, popup="🚁 起点A", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker(st.session_state.point_b, popup="🎯 终点B", icon=folium.Icon(color='red')).add_to(m)
        
        if st.session_state.selected_plan:
            p = st.session_state.selected_plan
            folium.PolyLine(p['points'], color=p['color'], weight=4, opacity=0.9).add_to(m)
            for i, wp in enumerate(p['points'][1:-1], 1):
                folium.Marker(wp, popup=f"绕行点{i}", icon=folium.Icon(color='purple', icon='refresh')).add_to(m)
        
        plugins.Draw(draw_options={'polygon': {'allowIntersection': False}}).add_to(m)
        plugins.MeasureControl().add_to(m)
        folium.LayerControl().add_to(m)
        
        output = st_folium(m, width=650, height=450, key="map")
        
        if output and output.get('last_active_drawing'):
            d = output['last_active_drawing']
            if d and d['geometry']['type'] == 'Polygon':
                pts = [[c[1], c[0]] for c in d['geometry']['coordinates'][0]]
                if len(pts) >= 3:
                    st.session_state.temp_obs = pts
                    st.session_state.show_height_panel = True
                    st.success(f"✅ 已绘制 {len(pts)} 个点")
    
    with col_right:
        if st.session_state.show_height_panel and st.session_state.temp_obs:
            st.markdown("### 🆕 新建3D障碍物")
            name = st.text_input("名称", value=st.session_state.temp_name)
            st.session_state.temp_name = name if name else "建筑物"
            
            height = st.number_input("障碍物高度 (m)", value=st.session_state.temp_height, min_value=1, max_value=200, step=5)
            st.session_state.temp_height = height
            
            current_flight = st.session_state.flight_alt
            if current_flight < height:
                st.warning(f"⚠️ 飞行高度 {current_flight}m < 障碍物 {height}m → **自动绕行**")
            else:
                st.success(f"✅ 飞行高度 {current_flight}m ≥ 障碍物 {height}m → **直接飞越**")
            
            col1, col2 = st.columns(2)
            with col1:
                if st.button("✅ 保存", type="primary", use_container_width=True):
                    new_obs = {'points': st.session_state.temp_obs, 'height': height, 'name': st.session_state.temp_name}
                    st.session_state.obstacles.append(new_obs)
                    save_obstacles_to_file()
                    st.session_state.temp_obs = None
                    st.session_state.show_height_panel = False
                    st.rerun()
            with col2:
                if st.button("🗑️ 取消", use_container_width=True):
                    st.session_state.temp_obs = None
                    st.session_state.show_height_panel = False
                    st.rerun()
            st.markdown("---")
        
        st.markdown("### 🚁 起点 A")
        col1, col2 = st.columns(2)
        with col1:
            la = st.number_input("纬度", value=st.session_state.point_a[0], format="%.6f")
        with col2:
            lo = st.number_input("经度", value=st.session_state.point_a[1], format="%.6f")
        if st.button("📍 设置A点", use_container_width=True):
            st.session_state.point_a = [la, lo]
            save_waypoints()
            st.rerun()
        
        st.markdown("### 🎯 终点 B")
        col1, col2 = st.columns(2)
        with col1:
            lb = st.number_input("纬度", value=st.session_state.point_b[0], format="%.6f", key="lb")
        with col2:
            lob = st.number_input("经度", value=st.session_state.point_b[1], format="%.6f", key="lob")
        if st.button("🏁 设置B点", use_container_width=True):
            st.session_state.point_b = [lb, lob]
            save_waypoints()
            st.rerun()
        
        st.markdown("---")
        
        st.markdown("### ⚙️ 飞行参数")
        alt = st.slider("飞行高度 (m)", 10, 100, st.session_state.flight_alt)
        st.session_state.flight_alt = alt
        
        safe_radius = st.slider("安全半径 (m)", 5, 30, st.session_state.safe_radius)
        st.session_state.safe_radius = safe_radius
        
        bypass_distance = st.slider("绕行距离 (m)", 5, 50, st.session_state.bypass_distance)
        st.session_state.bypass_distance = bypass_distance
        
        st.info(f"🛡️ 安全半径: {safe_radius}m | 🚀 绕行距离: {bypass_distance}m")
        
        if st.session_state.obstacles:
            st.markdown("**📊 高度检测规则**")
            for obs_data in st.session_state.obstacles:
                name = obs_data['name'] if isinstance(obs_data, dict) else obs_data.name
                height = obs_data['height'] if isinstance(obs_data, dict) else obs_data.height
                if alt < height:
                    st.warning(f"🔄 {name}({height}m)：飞行高度不足 → 绕行")
                else:
                    st.success(f"⬆️ {name}({height}m)：高度足够 → 飞越")
        
        st.markdown("---")
        
        st.markdown("### 🚧 障碍物列表")
        for i, obs_data in enumerate(st.session_state.obstacles):
            name = obs_data['name'] if isinstance(obs_data, dict) else obs_data.name
            height = obs_data['height'] if isinstance(obs_data, dict) else obs_data.height
            icon = "🔄" if alt < height else "⬆️"
            with st.expander(f"{icon} {name} (高度: {height}m)"):
                if st.button(f"删除", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    save_obstacles_to_file()
                    st.rerun()
        
        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("💾 保存配置", use_container_width=True):
                save_obstacles_to_file()
                st.success("已保存")
        with col2:
            if st.button("📂 加载配置", use_container_width=True):
                load_obstacles_from_file()
                st.rerun()
        with col3:
            if st.button("🗑️ 清空全部", use_container_width=True):
                st.session_state.obstacles = []
                save_obstacles_to_file()
                st.rerun()
        
        st.markdown("---")
        st.markdown("## 🗺️ 多航线规划")
        
        if st.button("🎯 生成航线方案", use_container_width=True, type="primary"):
            start = st.session_state.point_a
            end = st.session_state.point_b
            straight_dist = calc_distance(start, end)
            
            blocking = find_blocking_obstacles(start, end, st.session_state.obstacles, alt)
            
            plans = []
            
            if not blocking:
                plans.append({'name': '📏 直线飞越', 'points': [start, end], 
                             'dist': straight_dist, 'color': 'blue', 
                             'desc': '✅ 所有障碍物高度低于飞行高度，直接飞越'})
            else:
                directions = ['left', 'right', 'front', 'back']
                dir_names = {'left': '⬅️ 左绕行', 'right': '➡️ 右绕行', 
                            'front': '⬆️ 前绕行', 'back': '⬇️ 后绕行'}
                dir_colors = {'left': 'orange', 'right': 'purple', 
                             'front': 'cyan', 'back': 'pink'}
                
                for direction in directions:
                    path = plan_path_with_bypass(start, end, st.session_state.obstacles, alt, safe_radius, bypass_distance, direction)
                    path_dist = sum(calc_distance(path[i], path[i+1]) for i in range(len(path)-1))
                    plans.append({
                        'name': dir_names[direction],
                        'points': path,
                        'dist': path_dist,
                        'color': dir_colors[direction],
                        'desc': f'从{direction}侧沿障碍物外侧绕行'
                    })
                
                best = min(plans, key=lambda x: x['dist']).copy()
                best['name'] = '⭐ 最佳航线'
                best['color'] = 'gold'
                best['desc'] = f'最短绕行路径，距离{best["dist"]:.0f}m'
                plans.append(best)
            
            st.session_state.route_plans = plans
            st.session_state.selected_plan = plans[-1]
            st.rerun()
        
        if st.session_state.route_plans:
            st.markdown("---")
            st.markdown("### 📋 可选方案")
            
            for i, p in enumerate(st.session_state.route_plans):
                col1, col2, col3 = st.columns([2, 1, 1])
                with col1:
                    if "左绕行" in p['name']:
                        st.markdown(f"**🟠 {p['name']}**")
                    elif "右绕行" in p['name']:
                        st.markdown(f"**🟣 {p['name']}**")
                    elif "前绕行" in p['name']:
                        st.markdown(f"**🔵 {p['name']}**")
                    elif "后绕行" in p['name']:
                        st.markdown(f"**🟤 {p['name']}**")
                    elif "最佳" in p['name']:
                        st.markdown(f"**⭐ {p['name']}**")
                    else:
                        st.markdown(f"**🔵 {p['name']}**")
                    st.caption(p['desc'])
                with col2:
                    st.metric("距离", f"{p['dist']:.0f}m")
                with col3:
                    st.metric("时间", f"{p['dist']/15:.0f}s")
                
                if st.session_state.selected_plan and st.session_state.selected_plan['name'] == p['name']:
                    st.success("✅ 已选中")
                else:
                    if st.button(f"选择此方案", key=f"sel_{i}", use_container_width=True):
                        st.session_state.selected_plan = p
                        st.rerun()
                st.markdown("---")
            
            if st.session_state.selected_plan:
                p = st.session_state.selected_plan
                straight = calc_distance(st.session_state.point_a, st.session_state.point_b)
                extra = p['dist'] - straight
                st.info(f"**当前: {p['name']}** | 距离: {p['dist']:.0f}m | 比直线多走: {extra:.0f}m")
                if st.button("✈️ 确认使用此航线", use_container_width=True, type="primary"):
                    st.session_state.confirmed_plan = p
                    st.success(f"✅ 已确认")
                    st.balloons()
        
        if st.session_state.confirmed_plan:
            st.markdown("---")
            st.markdown("### ✅ 当前航线")
            p = st.session_state.confirmed_plan
            st.success(f"**{p['name']}**")
            st.caption(f"距离: {p['dist']:.0f}m | 时间: {p['dist']/15:.0f}s")

# ==================== Tab 2: 飞行监控 ====================
with tab2:
    st.subheader("📡 飞行实时画面 - 任务执行监控")
    
    col_left, col_right = st.columns([1, 2])
    
    with col_left:
        st.markdown("### 🎮 飞行控制")
        
        if st.button("📐 导入当前航线", use_container_width=True):
            start = st.session_state.point_a
            end = st.session_state.point_b
            waypoints = plan_path_with_bypass(
                start, end, st.session_state.obstacles,
                st.session_state.flight_alt, st.session_state.safe_radius,
                st.session_state.bypass_distance, 'best'
            )
            total_dist, seg_dists = calculate_distances(waypoints)
            st.session_state.flight_sim_waypoints = waypoints
            st.session_state.flight_sim_total_distance = total_dist
            st.session_state.flight_sim_segment_distances = seg_dists
            st.session_state.flight_sim_current_index = 0
            st.session_state.flight_sim_running = False
            st.session_state.flight_sim_start_time = None
            st.session_state.flight_sim_last_wp_index = -1
            clear_all_logs()
            add_business_log(f"航线规划完成 | 航点数: {len(waypoints)} | 路径长度: {total_dist:.1f}m", color="green")
            add_gcs_to_fcu_log("GCS→OBC: MISSION_UPLOAD")
            add_fcu_to_gcs_log("FCU→OBC: MISSION_ACK")
            st.success(f"✅ 航线已导入，共 {len(waypoints)} 个航点")
            st.rerun()
        
        waypoints = st.session_state.flight_sim_waypoints
        seg_dists = st.session_state.flight_sim_segment_distances
        total_dist = st.session_state.flight_sim_total_distance
        
        st.markdown("---")
        
        speed = st.slider("飞行速度 (m/s)", 1.0, 20.0, st.session_state.flight_sim_speed, 0.5)
        st.session_state.flight_sim_speed = speed
        
        st.markdown("---")
        
        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("▶️ 开始", use_container_width=True, disabled=len(waypoints) == 0):
                st.session_state.flight_sim_running = True
                if st.session_state.flight_sim_start_time is None:
                    st.session_state.flight_sim_start_time = time.time()
                add_fcu_to_gcs_log("FCU→OBC→GCS: ACK | Mode: AUTO")
                st.rerun()
        with col2:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.flight_sim_running = False
                st.rerun()
        with col3:
            if st.button("🔄 重置", use_container_width=True):
                st.session_state.flight_sim_running = False
                st.session_state.flight_sim_start_time = None
                st.session_state.flight_sim_current_index = 0
                st.session_state.flight_sim_last_wp_index = -1
                clear_all_logs()
                st.rerun()
        
        st.markdown("---")
        st.markdown("### 📋 航线信息")
        st.caption(f"起点A: {st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}")
        st.caption(f"终点B: {st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}")
        st.caption(f"飞行高度: {st.session_state.flight_alt} m")
        st.caption(f"安全半径: {st.session_state.safe_radius} m")
        st.caption(f"绕行距离: {st.session_state.bypass_distance} m")
        st.caption(f"航点数量: {len(waypoints)}")
        if total_dist > 0:
            st.caption(f"总距离: {total_dist:.1f} 米")
        
        st.markdown("---")
        st.markdown("### 💓 心跳监控")
        
        if not st.session_state.heartbeat_running:
            if st.button("▶️ 启动心跳", use_container_width=True):
                st.session_state.heartbeat_sim.start()
                st.session_state.heartbeat_running = True
                st.rerun()
        else:
            if st.button("⏹️ 停止心跳", use_container_width=True):
                st.session_state.heartbeat_sim.stop()
                st.session_state.heartbeat_running = False
                st.rerun()
        
        hb = st.session_state.heartbeat_sim.update()
        if hb:
            if hb['status'] == 'timeout':
                st.error(f"⚠️ 连接超时！3秒未收到心跳")
            else:
                st.success(f"💓 心跳正常 | ID: {hb['id']} | 延迟: {hb['delay']}ms")
        
        stats = st.session_state.heartbeat_sim.get_stats()
        col1, col2 = st.columns(2)
        col1.metric("总心跳", stats['total'])
        col2.metric("成功率", f"{stats['rate']}%")
    
    with col_right:
        if len(waypoints) > 0:
            if st.session_state.flight_sim_running:
                elapsed_time = time.time() - st.session_state.flight_sim_start_time
                current_speed = st.session_state.flight_sim_speed
                flown_distance = elapsed_time * current_speed
                
                total_flown = 0
                current_index = 0
                segment_progress = 0
                
                for i, seg_dist in enumerate(seg_dists):
                    if total_flown + seg_dist >= flown_distance:
                        current_index = i
                        if seg_dist > 0:
                            segment_progress = (flown_distance - total_flown) / seg_dist
                        break
                    total_flown += seg_dist
                else:
                    current_index = len(waypoints) - 1
                    segment_progress = 1
                    st.session_state.flight_sim_running = False
                
                st.session_state.flight_sim_current_index = current_index
                
                p1 = waypoints[current_index]
                p2_index = min(current_index + 1, len(waypoints) - 1)
                p2 = waypoints[p2_index]
                current_lng = p1[0] + (p2[0] - p1[0]) * segment_progress
                current_lat = p1[1] + (p2[1] - p1[1]) * segment_progress
                
                remaining_distance = max(0, total_dist - flown_distance)
                remaining_time = remaining_distance / current_speed if current_speed > 0 else 0
                
                minutes = int(elapsed_time // 60)
                seconds = int(elapsed_time % 60)
                elapsed_str = f"{minutes:02d}:{seconds:02d}"
                
                rem_minutes = int(remaining_time // 60)
                rem_seconds = int(remaining_time % 60)
                remaining_str = f"{rem_minutes:02d}:{rem_seconds:02d}"
                
                battery_remaining = max(0, 100 - (elapsed_time / 1800) * 100)
                
                if current_index > st.session_state.flight_sim_last_wp_index:
                    st.session_state.flight_sim_last_wp_index = current_index
                    add_fcu_to_gcs_log(f"FCU→OBC→GCS: WP_REACHED #{current_index}")
                    if current_index >= len(waypoints) - 1:
                        add_fcu_to_gcs_log("FCU→OBC→GCS: MISSION_COMPLETE")
                        add_business_log("任务执行完成", color="green")
            else:
                current_lng = waypoints[0][0]
                current_lat = waypoints[0][1]
                flown_distance = 0
                remaining_distance = total_dist
                elapsed_str = "00:00"
                remaining_str = "00:00"
                battery_remaining = 100
                current_index = 0
            
            st.markdown("### 📊 飞行数据")
            
            col1, col2, col3, col4 = st.columns(4)
            col1.metric("当前航点", f"{min(current_index + 1, len(waypoints))}/{len(waypoints)}")
            col2.metric("飞行速度", f"{st.session_state.flight_sim_speed:.1f} m/s")
            col3.metric("已用时间", elapsed_str)
            col4.metric("剩余距离", f"{remaining_distance:.0f} m")
            
            col1, col2, col3, col4 = st.columns(4)
            col1.metric("预计到达", remaining_str)
            col2.metric("电量模拟", f"{battery_remaining:.0f}%")
            col3.metric("完成进度", f"{(flown_distance/total_dist*100) if total_dist > 0 else 0:.0f}%")
            
            if total_dist > 0:
                st.progress(min(1.0, flown_distance / total_dist))
            
            st.markdown("---")
            st.markdown("### 📶 通信链路拓扑")
            
            col_gcs, col_obc, col_fcu = st.columns(3)
            with col_gcs:
                st.success("✅ GCS 在线")
            with col_obc:
                st.success("✅ OBC 在线")
            with col_fcu:
                st.success("✅ FCU 在线")
            
            st.markdown("---")
            
            if st.session_state.flight_sim_running:
                st.info("✈️ 任务执行中...")
            elif current_index >= len(waypoints) - 1 and len(waypoints) > 0:
                st.success("✅ 任务已完成！")
            else:
                st.info("⏸️ 等待开始")
            
            st.markdown("---")
            st.markdown("### 📝 通信日志")
            
            log_container = st.container(height=200)
            with log_container:
                for log in st.session_state.comm_logs_business[-10:]:
                    st.caption(f"📋 [{log['timestamp']}] {log['message']}")
                for log in st.session_state.comm_logs_fcu_to_gcs[-5:]:
                    st.caption(f"⬆️ {log}")
                for log in st.session_state.comm_logs_gcs_to_fcu[-5:]:
                    st.caption(f"⬇️ {log}")
            
            if st.session_state.flight_sim_running:
                time.sleep(1)
                st.rerun()
        else:
            st.info("请先点击「导入当前航线」加载航线")

st.markdown("---")
st.caption("💡 **使用规则**：①飞行高度 < 障碍物高度 → 自动沿外侧绕行 ②飞行高度 ≥ 障碍物高度 → 直接直线飞越")
