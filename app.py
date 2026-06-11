import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import math
import json
import os
from datetime import datetime

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机智能监控系统", page_icon="🛰️", layout="wide")

# ==================== 基础配置 ====================
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
    st.session_state.point_a = [32.2323, 118.749]
    st.session_state.point_b = [32.2344, 118.749]
    if os.path.exists(WAYPOINT_CONFIG_FILE):
        try:
            with open(WAYPOINT_CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                st.session_state.point_a = data.get('point_a', [32.2323, 118.749])
                st.session_state.point_b = data.get('point_b', [32.2344, 118.749])
        except:
            pass

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
if 'temp_obs' not in st.session_state:
    st.session_state.temp_obs = None
if 'temp_height' not in st.session_state:
    st.session_state.temp_height = 50
if 'temp_name' not in st.session_state:
    st.session_state.temp_name = "建筑物"
if 'show_height_panel' not in st.session_state:
    st.session_state.show_height_panel = False

# ==================== 保存函数 ====================
def save_waypoints():
    data = {'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
            'point_a': st.session_state.point_a, 
            'point_b': st.session_state.point_b}
    with open(WAYPOINT_CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def save_obstacles_to_file():
    data = {'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
            'obstacles': st.session_state.obstacles}
    with open(OBSTACLE_CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 核心几何函数 ====================
def calc_distance(p1, p2):
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    R = 6371000
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def point_in_polygon(point, poly):
    x, y = point[1], point[0]
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][1], poly[i][0]
        x2, y2 = poly[(i+1)%n][1], poly[(i+1)%n][0]
        if ((y1 > y) != (y2 > y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1):
            inside = not inside
    return inside

def line_intersect_polygon(start, end, poly, sample_num=200):
    for i in range(sample_num + 1):
        t = i / sample_num
        curr_lat = start[0] + (end[0] - start[0]) * t
        curr_lon = start[1] + (end[1] - start[1]) * t
        if point_in_polygon([curr_lat, curr_lon], poly):
            return True
    return False

def meter_to_degree(meter):
    return meter / 111000.0

# ==================== 障碍物类 ====================
class Obstacle:
    def __init__(self, points, height, name):
        self.points = points
        self.height = height
        self.name = name
        self.min_lat = min(p[0] for p in points)
        self.max_lat = max(p[0] for p in points)
        self.min_lon = min(p[1] for p in points)
        self.max_lon = max(p[1] for p in points)
        self.center_lat = (self.min_lat + self.max_lat) / 2
        self.center_lon = (self.min_lon + self.max_lon) / 2

    def to_dict(self):
        return {'points': self.points, 'height': self.height, 'name': self.name}
    
    @classmethod
    def from_dict(cls, data):
        return cls(data['points'], data['height'], data['name'])
    
    def contains(self, point):
        return point_in_polygon(point, self.points)
    
    def line_intersects(self, start, end):
        return line_intersect_polygon(start, end, self.points)

    # 【关键修复】固定方向的绕行点生成，左右永远不同
    def get_fixed_side_bypass(self, start, end, safe_radius, bypass_distance, side):
        offset_deg = meter_to_degree(safe_radius + bypass_distance)
        dx = end[1] - start[1]
        dy = end[0] - start[0]
        length = math.hypot(dx, dy)
        if length < 1e-10:
            dx, dy = 0, 1
        else:
            dx /= length
            dy /= length
        
        if side == 'left':
            # 左侧绕行点：强制向左偏移
            return [
                self.center_lat + dy * offset_deg * 2,
                self.center_lon - dx * offset_deg * 2
            ]
        elif side == 'right':
            # 右侧绕行点：强制向右偏移
            return [
                self.center_lat - dy * offset_deg * 2,
                self.center_lon + dx * offset_deg * 2
            ]
        return [self.center_lat, self.center_lon]

# ==================== 路径规划（只保留左/右/最佳） ====================
def find_blocking_obstacles(start, end, obstacles, flight_alt):
    blocking = []
    for obs_data in obstacles:
        obs = Obstacle.from_dict(obs_data)
        if flight_alt >= obs.height:
            continue
        if obs.line_intersects(start, end):
            blocking.append(obs)
    return blocking

def plan_path_side(start, end, obstacles, flight_alt, safe_radius, bypass_distance, side):
    waypoints = [start]
    current_start = start
    blocking = find_blocking_obstacles(start, end, obstacles, flight_alt)
    if not blocking:
        waypoints.append(end)
        return waypoints
    # 按距离起点排序，逐个绕行
    blocking.sort(key=lambda o: calc_distance([o.center_lat, o.center_lon], start))
    for obs in blocking:
        bypass = obs.get_fixed_side_bypass(current_start, end, safe_radius, bypass_distance, side)
        waypoints.append(bypass)
        current_start = bypass
    waypoints.append(end)
    return waypoints

# ==================== 标题 ====================
st.title("🛰️ 无人机智能监控系统")
st.markdown("**南京科技职业学院** | 左/右绕行 + 最佳航线")
st.markdown("---")

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
            obs = Obstacle.from_dict(obs_data)
            color = 'red' if alt < obs.height else 'green'
            folium.Polygon(obs.points, color=color, weight=2, fill=True, 
                          fill_color=color, fill_opacity=0.3,
                          popup=f"{obs.name}\n高度: {obs.height}m").add_to(m)
        
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
            st.markdown("### 🆕 新建障碍物")
            name = st.text_input("名称", value=st.session_state.temp_name)
            st.session_state.temp_name = name if name else "建筑物"
            height = st.number_input("障碍物高度 (m)", value=st.session_state.temp_height, min_value=1, max_value=200, step=5)
            st.session_state.temp_height = height
            if st.session_state.flight_alt < height:
                st.warning(f"⚠️ 飞行高度不足，将绕行")
            else:
                st.success(f"✅ 高度足够，可飞越")
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
            st.markdown("**📊 高度检测**")
            for obs_data in st.session_state.obstacles:
                obs = Obstacle.from_dict(obs_data)
                if alt < obs.height:
                    st.warning(f"🔄 {obs.name}({obs.height}m)：绕行")
                else:
                    st.success(f"⬆️ {obs.name}({obs.height}m)：飞越")
        
        st.markdown("---")
        st.markdown("### 🚧 障碍物列表")
        for i, obs_data in enumerate(st.session_state.obstacles):
            obs = Obstacle.from_dict(obs_data)
            icon = "🔄" if alt < obs.height else "⬆️"
            with st.expander(f"{icon} {obs.name} (高度: {obs.height}m)"):
                if st.button(f"删除", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    save_obstacles_to_file()
                    st.rerun()
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("💾 保存配置", use_container_width=True):
                save_obstacles_to_file()
                st.success("已保存")
        with col2:
            if st.button("🗑️ 清空全部", use_container_width=True):
                st.session_state.obstacles = []
                save_obstacles_to_file()
                st.rerun()
        
        st.markdown("---")
        st.markdown("## 🗺️ 航线方案（仅左/右/最佳）")
        
        if st.button("🎯 生成航线方案", use_container_width=True, type="primary"):
            start = st.session_state.point_a
            end = st.session_state.point_b
            straight_dist = calc_distance(start, end)
            blocking = find_blocking_obstacles(start, end, st.session_state.obstacles, alt)
            
            plans = []
            if not blocking:
                plans.append({
                    'name': '📏 直线飞越',
                    'points': [start, end],
                    'dist': straight_dist,
                    'color': 'blue',
                    'desc': '✅ 所有障碍物可飞越'
                })
            else:
                # 只保留左/右两个绕行方案
                plans.append({
                    'name': '⬅️ 左绕行',
                    'points': plan_path_side(start, end, st.session_state.obstacles, alt, safe_radius, bypass_distance, 'left'),
                    'color': 'orange',
                    'desc': '从障碍物左侧绕行'
                })
                plans.append({
                    'name': '➡️ 右绕行',
                    'points': plan_path_side(start, end, st.session_state.obstacles, alt, safe_radius, bypass_distance, 'right'),
                    'color': 'purple',
                    'desc': '从障碍物右侧绕行'
                })
                # 计算距离，生成最佳方案
                for p in plans:
                    p['dist'] = sum(calc_distance(p['points'][i], p['points'][i+1]) for i in range(len(p['points'])-1))
                best = min(plans, key=lambda x: x['dist']).copy()
                best['name'] = '⭐ 最佳航线'
                best['color'] = 'gold'
                best['desc'] = f'最短路径，距离{best["dist"]:.0f}m'
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
                    st.markdown(f"**{p['name']}**")
                    st.caption(p['desc'])
                with col2:
                    st.metric("距离", f"{p['dist']:.0f}m")
                with col3:
                    if st.session_state.selected_plan and st.session_state.selected_plan['name'] == p['name']:
                        st.success("✅ 已选中")
                    else:
                        if st.button(f"选择", key=f"sel_{i}", use_container_width=True):
                            st.session_state.selected_plan = p
                            st.rerun()
                st.markdown("---")
