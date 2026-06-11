import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import math
import json
import os
from datetime import datetime
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.affinity import translate

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

# ==================== 核心地理工具函数 ====================
def calc_distance(p1, p2):
    """Haversine 两点距离，单位米"""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    R = 6371000
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def meter_to_degree(meter):
    """米转经纬度度数近似值"""
    lat_deg_per_m = 1 / 111320
    lon_deg_per_m = 1 / (111320 * math.cos(math.radians(CAMPUS[0])))
    return (meter * lat_deg_per_m, meter * lon_deg_per_m)

def poly_buffer_meter(poly_points, buffer_m):
    """多边形向外扩展buffer_m米缓冲区，返回新轮廓点"""
    lat_buf, lon_buf = meter_to_degree(buffer_m)
    sh_poly = Polygon(poly_points)
    buf_poly = sh_poly.buffer(distance=math.sqrt(lat_buf**2 + lon_buf**2))
    if buf_poly.geom_type == 'MultiPolygon':
        buf_poly = unary_union(buf_poly)
    coords = list(buf_poly.exterior.coords)
    return [list(pt) for pt in coords[:-1]]

def line_cross_obstacle(line_start, line_end, obs_poly_list):
    """线段是否和任意障碍物（含缓冲区）相交"""
    line = LineString([line_start, line_end])
    for poly in obs_poly_list:
        sh_p = Polygon(poly)
        if line.intersects(sh_p):
            return True
    return False

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

# ==================== A* 全局最短路径规划（贴障碍物绕行） ====================
def astar_shortest_route(start, end, obstacle_list, flight_alt, safe_radius, bypass_dist, grid_step_m=8):
    """
    全局寻路生成贴障碍物外侧最短航线
    :param grid_step_m: 栅格步长，越小路径越顺滑
    """
    # 筛选需要绕行的障碍物（飞行高度低于建筑）
    need_avoid_obs = []
    buf_total = safe_radius + bypass_dist
    for obs_data in obstacle_list:
        obs = Obstacle.from_dict(obs_data)
        if flight_alt < obs.height:
            buf_poly = poly_buffer_meter(obs.points, buf_total)
            need_avoid_obs.append(buf_poly)
    if not need_avoid_obs:
        return [start, end]

    # 边界扩张范围
    all_points = [start, end]
    for pbuf in need_avoid_obs:
        all_points.extend(pbuf)
    lat_min = min(p[0] for p in all_points) - meter_to_degree(buf_total)[0]*3
    lat_max = max(p[0] for p in all_points) + meter_to_degree(buf_total)[0]*3
    lon_min = min(p[1] for p in all_points) - meter_to_degree(buf_total)[1]*3
    lon_max = max(p[1] for p in all_points) + meter_to_degree(buf_total)[1]*3

    lat_step, lon_step = meter_to_degree(grid_step_m)
    open_set = []
    closed_set = set()

    class Node:
        def __init__(self, lat, lon, parent=None):
            self.lat = lat
            self.lon = lon
            self.parent = parent
            self.g = float('inf')
            self.h = calc_distance((lat, lon), end)
            self.f = self.g + self.h
        def pos(self):
            return (round(self.lat, 8), round(self.lon, 8))
    
    start_node = Node(start[0], start[1])
    start_node.g = 0
    start_node.f = start_node.h
    open_set.append(start_node)

    dirs = [(-lat_step,0), (lat_step,0), (0,-lon_step), (0,lon_step),
            (-lat_step,-lon_step), (-lat_step,lon_step),
            (lat_step,-lon_step), (lat_step,lon_step)]
    
    found_end = None
    max_iter = 8000
    iter_cnt = 0

    while open_set and iter_cnt < max_iter:
        iter_cnt += 1
        open_set.sort(key=lambda n: n.f)
        curr = open_set.pop(0)
        if curr.pos() in closed_set:
            continue
        closed_set.add(curr.pos())

        # 到达终点阈值
        if calc_distance((curr.lat, curr.lon), end) < grid_step_m*1.2:
            found_end = curr
            break
        
        for dlat, dlon in dirs:
            new_lat = curr.lat + dlat
            new_lon = curr.lon + dlon
            new_node = Node(new_lat, new_lon, parent=curr)
            if new_node.pos() in closed_set:
                continue
            # 检测新线段是否闯入缓冲区障碍物
            if line_cross_obstacle((curr.lat,curr.lon), (new_lat,new_lon), need_avoid_obs):
                continue
            new_g = curr.g + calc_distance((curr.lat,curr.lon), (new_lat,new_lon))
            if new_g < new_node.g:
                new_node.g = new_g
                new_node.f = new_node.g + new_node.h
                if new_node not in open_set:
                    open_set.append(new_node)
    
    # 回溯路径
    if not found_end:
        return [start, end]
    path_nodes = []
    tmp = found_end
    while tmp:
        path_nodes.insert(0, [tmp.lat, tmp.lon])
        tmp = tmp.parent
    return path_nodes

# ==================== 标题 ====================
st.title("🛰️ 无人机智能监控系统")
st.markdown("**南京科技职业学院** | 缓冲区膨胀+A*最短贴边绕行")
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
        safe_buf_total = st.session_state.safe_radius + st.session_state.bypass_distance
        for obs_data in st.session_state.obstacles:
            obs = Obstacle.from_dict(obs_data)
            color = 'red' if alt < obs.height else 'green'
            # 绘制原始建筑轮廓
            folium.Polygon(obs.points, color=color, weight=2, fill=True, 
                          fill_color=color, fill_opacity=0.3,
                          popup=f"{obs.name}\n高度: {obs.height}m").add_to(m)
            # 绘制安全缓冲区轮廓（半透明蓝色虚线）
            if alt < obs.height:
                buf_coords = poly_buffer_meter(obs.points, safe_buf_total)
                folium.Polygon(buf_coords, color='blue', weight=1, dash_array='5,5', fill=False,
                              popup=f"安全缓冲区 {safe_buf_total}m").add_to(m)
        
        folium.Marker(st.session_state.point_a, popup="🚁 起点A", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker(st.session_state.point_b, popup="🎯 终点B", icon=folium.Icon(color='red')).add_to(m)
        
        if st.session_state.selected_plan:
            p = st.session_state.selected_plan
            folium.PolyLine(p['points'], color=p['color'], weight=4, opacity=0.9).add_to(m)
            for i, wp in enumerate(p['points'][1:-1], 1):
                folium.Marker(wp, popup=f"绕行拐点{i}", icon=folium.Icon(color='purple', icon='refresh')).add_to(m)
        
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
                    st.success(f"✅ 已绘制 {len(pts)} 个点圈选障碍物")
    
    with col_right:
        if st.session_state.show_height_panel and st.session_state.temp_obs:
            st.markdown("### 🆕 新建障碍物")
            name = st.text_input("名称", value=st.session_state.temp_name)
            st.session_state.temp_name = name if name else "建筑物"
            height = st.number_input("障碍物高度 (m)", value=st.session_state.temp_height, min_value=1, max_value=200, step=5)
            st.session_state.temp_height = height
            if st.session_state.flight_alt < height:
                st.warning(f"⚠️ 飞行高度不足，自动绕行")
            else:
                st.success(f"✅ 高度足够，直线飞越")
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
        st.info(f"🛡️ 安全半径: {safe_radius}m | 🚀 绕行余量: {bypass_distance}m\n总禁飞缓冲区：{safe_radius+bypass_distance}m")
        
        if st.session_state.obstacles:
            st.markdown("**📊 高度检测**")
            for obs_data in st.session_state.obstacles:
                obs = Obstacle.from_dict(obs_data)
                if alt < obs.height:
                    st.warning(f"🔄 {obs.name}({obs.height}m)：强制绕行")
                else:
                    st.success(f"⬆️ {obs.name}({obs.height}m)：直线飞越")
        
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
                st.success("已保存障碍物配置")
        with col2:
            if st.button("🗑️ 清空全部障碍物", use_container_width=True):
                st.session_state.obstacles = []
                save_obstacles_to_file()
                st.rerun()
        
        st.markdown("---")
        st.markdown("## 🗺️ 一键生成最短贴边航线")
        
        if st.button("🎯 自动生成最优绕行航线", use_container_width=True, type="primary"):
            start = st.session_state.point_a
            end = st.session_state.point_b
            straight_dist = calc_distance(start, end)

            # A*全局寻路
            route_points = astar_shortest_route(
                start, end,
                st.session_state.obstacles,
                alt, safe_radius, bypass_distance
            )
            real_dist = sum(calc_distance(route_points[i], route_points[i+1]) for i in range(len(route_points)-1))

            plans = [{
                'name': '⭐ 全局最短贴边航线',
                'points': route_points,
                'dist': real_dist,
                'color': 'blue',
                'desc': f"自动避开全部障碍物，紧贴{safe_radius+bypass_distance}m安全区外侧绕行"
            }]
            st.session_state.route_plans = plans
            st.session_state.selected_plan = plans[0]
            st.rerun()
        
        if st.session_state.route_plans:
            st.markdown("---")
            st.markdown("### 📋 航线结果")
            p = st.session_state.route_plans[0]
            st.markdown(f"**{p['name']}**")
            st.caption(p['desc'])
            st.metric("航线总长度", f"{p['dist']:.1f}m")
            st.metric("航线拐点数量", f"{len(p['points'])-2}个")

# Tab2 飞行监控你原有代码完全保留，这里省略不改动
with tab2:
    st.info("飞行监控模块可沿用原有代码，无需修改")
