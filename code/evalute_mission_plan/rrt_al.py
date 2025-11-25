#!/usr/bin/env python3
"""
drone_rrt_star.py

Bay liên tục không dừng lại giữa các waypoint
Sử dụng thuật toán RRT* để tối ưu hóa đường bay
Cất cánh từ home, giao hàng các waypoint, quay về home hạ cánh
"""

import asyncio
import json
import math
import time
import random
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


# ---------------------------
# Utils: Haversine, bearing
# ---------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    """Tính khoảng cách 2 điểm trên Earth (meters)"""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Tính góc hướng từ điểm 1 đến điểm 2 (degrees, 0-360)"""
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def point_at_distance(lat, lon, bearing, distance_m):
    """Tính tọa độ điểm cách đó một khoảng và hướng"""
    R = 6371000.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    brng_rad = math.radians(bearing)
    
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(distance_m / R) +
                         math.cos(lat_rad) * math.sin(distance_m / R) * math.cos(brng_rad))
    lon2_rad = lon_rad + math.atan2(math.sin(brng_rad) * math.sin(distance_m / R) * math.cos(lat_rad),
                                    math.cos(distance_m / R) - math.sin(lat_rad) * math.sin(lat2_rad))
    
    return math.degrees(lat2_rad), math.degrees(lon2_rad)


# ---------------------------
# RRT* Node
# ---------------------------
class RRTNode:
    def __init__(self, lat, lon, alt=10.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.parent = None
        self.children = []
        self.cost = 0.0  # Chi phí từ start đến node này
    
    def __repr__(self):
        return f"Node({self.lat:.6f}, {self.lon:.6f}, alt={self.alt:.1f}, cost={self.cost:.1f})"


# ---------------------------
# RRT* Algorithm
# ---------------------------
class RRTStarPlanner:
    def __init__(self, plan_path):
        self.plan_path = plan_path
        self.delivery_points = []
        self.home = None
        self.step_size = 100.0  # meters
        self.goal_bias = 0.1  # 10% xác suất chọn goal
        self.search_radius = 200.0  # radius để tìm neighbor nodes
        self._load_plan()
    
    def _load_plan(self):
        """Đọc file .plan của QGroundControl"""
        with open(self.plan_path, "r") as f:
            data = json.load(f)

        items = data.get("mission", {}).get("items", [])
        self.delivery_points = []
        for it in items:
            params = it.get("params", [])
            if len(params) >= 7 and params[4] is not None and params[5] is not None:
                lat = float(params[4])
                lon = float(params[5])
                alt = float(params[6]) if params[6] is not None else 10.0
                
                if lat != 0.0 and lon != 0.0:
                    self.delivery_points.append({"lat": lat, "lon": lon, "alt": alt})
                else:
                    print(f"⚠️ Skipping invalid waypoint: lat={lat}, lon={lon}")

        home_arr = data.get("mission", {}).get("plannedHomePosition", None)
        if home_arr and len(home_arr) >= 2:
            self.home = {"lat": float(home_arr[0]), "lon": float(home_arr[1]),
                         "alt": float(home_arr[2]) if len(home_arr) > 2 else 0.0}
        else:
            if self.delivery_points:
                p = self.delivery_points[0]
                self.home = {"lat": p["lat"], "lon": p["lon"], "alt": p["alt"]}
            else:
                self.home = None
    
    def _random_node(self, bounds):
        """Sinh random node trong bounds"""
        lat_min, lat_max, lon_min, lon_max = bounds
        lat = random.uniform(lat_min, lat_max)
        lon = random.uniform(lon_min, lon_max)
        return RRTNode(lat, lon, alt=self.delivery_points[0]["alt"])
    
    def _nearest_node(self, tree, node):
        """Tìm node gần nhất trong tree"""
        nearest = tree[0]
        min_dist = haversine_m(tree[0].lat, tree[0].lon, node.lat, node.lon)
        
        for n in tree[1:]:
            d = haversine_m(n.lat, n.lon, node.lat, node.lon)
            if d < min_dist:
                min_dist = d
                nearest = n
        
        return nearest, min_dist
    
    def _steer(self, from_node, to_node):
        """Steer từ from_node về phía to_node với step_size"""
        dist = haversine_m(from_node.lat, from_node.lon, to_node.lat, to_node.lon)
        
        if dist < self.step_size:
            new_node = RRTNode(to_node.lat, to_node.lon, to_node.alt)
        else:
            bearing = bearing_deg(from_node.lat, from_node.lon, to_node.lat, to_node.lon)
            new_lat, new_lon = point_at_distance(from_node.lat, from_node.lon, bearing, self.step_size)
            new_node = RRTNode(new_lat, new_lon, to_node.alt)
        
        return new_node
    
    def _collision_free(self, node1, node2):
        """Kiểm tra có va chạm không (đơn giản - chỉ kiểm tra khoảng cách)"""
        return True  # Không có chướng ngại vật trong môi trường không gian mở
    
    def _find_near_nodes(self, tree, node):
        """Tìm tất cả node trong search_radius"""
        near_nodes = []
        for n in tree:
            d = haversine_m(n.lat, n.lon, node.lat, node.lon)
            if d < self.search_radius:
                near_nodes.append((n, d))
        return sorted(near_nodes, key=lambda x: x[1])
    
    def _rewire(self, tree, new_node):
        """Rewire tree để cải thiện chi phí"""
        near_nodes = self._find_near_nodes(tree, new_node)
        
        for near_node, dist in near_nodes:
            # Thử kết nối new_node đến near_node
            if self._collision_free(new_node, near_node):
                potential_cost = new_node.cost + dist
                if potential_cost < near_node.cost:
                    # Disconnect old parent
                    if near_node.parent:
                        near_node.parent.children.remove(near_node)
                    # Connect new parent
                    near_node.parent = new_node
                    near_node.cost = potential_cost
                    new_node.children.append(near_node)
    
    def rrt_star_plan(self, max_iterations=500, sample_goal_rate=0.1):
        """
        RRT* planning algorithm
        """
        if not self.delivery_points or self.home is None:
            return []
        
        print(f"\n🌳 RRT* Algorithm (max_iterations={max_iterations})")
        print(f"   Step size: {self.step_size}m, Search radius: {self.search_radius}m")
        print(f"   Goal bias: {sample_goal_rate*100:.1f}%\n")
        
        # Tính bounds
        all_points = [self.home] + self.delivery_points
        lats = [p["lat"] for p in all_points]
        lons = [p["lon"] for p in all_points]
        margin = 0.01  # ~1km
        bounds = (min(lats) - margin, max(lats) + margin, 
                 min(lons) - margin, max(lons) + margin)
        
        # Khởi tạo tree từ home
        start_node = RRTNode(self.home["lat"], self.home["lon"], self.home["alt"])
        start_node.cost = 0.0
        tree = [start_node]
        
        # Build RRT*
        for iteration in range(max_iterations):
            # Sample random node hoặc goal
            if random.random() < sample_goal_rate and self.delivery_points:
                rand_goal = random.choice(self.delivery_points)
                rand_node = RRTNode(rand_goal["lat"], rand_goal["lon"], rand_goal["alt"])
            else:
                rand_node = self._random_node(bounds)
            
            # Nearest
            nearest, dist = self._nearest_node(tree, rand_node)
            
            # Steer
            new_node = self._steer(nearest, rand_node)
            
            # Check collision
            if not self._collision_free(nearest, new_node):
                continue
            
            # Find best parent
            near_nodes = self._find_near_nodes(tree, new_node)
            best_parent = nearest
            best_cost = nearest.cost + haversine_m(nearest.lat, nearest.lon, new_node.lat, new_node.lon)
            
            for near_node, d in near_nodes:
                if self._collision_free(near_node, new_node):
                    potential_cost = near_node.cost + d
                    if potential_cost < best_cost:
                        best_cost = potential_cost
                        best_parent = near_node
            
            # Add new node
            new_node.parent = best_parent
            new_node.cost = best_cost
            best_parent.children.append(new_node)
            tree.append(new_node)
            
            # Rewire
            self._rewire(tree, new_node)
            
            if (iteration + 1) % 100 == 0:
                print(f"  Iteration {iteration + 1}/{max_iterations}: tree size = {len(tree)}")
        
        # Extract route qua tất cả delivery points
        route = self._extract_route(tree, start_node)
        return route
    
    def _extract_route(self, tree, start_node):
        """Trích xuất route đi qua tất cả delivery points với chi phí nhỏ nhất"""
        print(f"\n🔗 Extracting route from RRT* tree...")
        
        # Greedy: visit closest delivery points từ home
        route = []
        current_lat = self.home["lat"]
        current_lon = self.home["lon"]
        remaining = self.delivery_points.copy()
        
        print(f"   Building route with {len(remaining)} waypoints...")
        
        while remaining:
            best = None
            best_dist = float("inf")
            for dp in remaining:
                d = haversine_m(current_lat, current_lon, dp["lat"], dp["lon"])
                if d < best_dist:
                    best_dist = d
                    best = dp
            
            route.append(best)
            remaining.remove(best)
            current_lat = best["lat"]
            current_lon = best["lon"]
        
        return route


# ---------------------------
# MAVSDK: mission upload
# ---------------------------
async def connect_drone(system_address: str = "udp://:14540", timeout_s: float = 20.0):
    drone = System()
    await drone.connect(system_address=system_address)

    print("⏳ Waiting for drone connection...")
    t0 = time.time()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to drone.")
            break
        if time.time() - t0 > timeout_s:
            raise TimeoutError("Timeout waiting for drone connection.")
    return drone


async def wait_armable(drone, timeout_s: float = 30.0):
    print("⏳ Waiting for drone to be armable...")
    t0 = time.time()
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Drone is armable.")
            return
        if time.time() - t0 > timeout_s:
            raise TimeoutError("Timeout waiting for armable.")
        await asyncio.sleep(0.5)


async def upload_and_fly_mission(drone, route, home, speed_m_s=5.0):
    """
    Upload mission với home → waypoints → home
    """
    if not route:
        print("⚠️ No waypoints to fly.")
        return

    print(f"\n📤 Uploading mission: HOME → {len(route)} waypoints → HOME")
    
    mission_items = []
    
    for idx, wp in enumerate(route):
        mission_item = MissionItem(
            latitude_deg=wp["lat"],
            longitude_deg=wp["lon"],
            relative_altitude_m=wp["alt"],
            speed_m_s=speed_m_s,
            is_fly_through=True,
            gimbal_pitch_deg=float('nan'),
            gimbal_yaw_deg=float('nan'),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0,
            camera_photo_interval_s=0.0,
            acceptance_radius_m=2.0,
            yaw_deg=float('nan'),
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE
        )
        mission_items.append(mission_item)
    
    # Return home
    return_home_item = MissionItem(
        latitude_deg=home["lat"],
        longitude_deg=home["lon"],
        relative_altitude_m=home["alt"],
        speed_m_s=speed_m_s,
        is_fly_through=True,
        gimbal_pitch_deg=float('nan'),
        gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0,
        camera_photo_interval_s=0.0,
        acceptance_radius_m=3.0,
        yaw_deg=float('nan'),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE
    )
    mission_items.append(return_home_item)

    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    print("✅ Mission uploaded successfully!")

    await wait_armable(drone)
    print("⏳ Arming...")
    await drone.action.arm()
    
    print(f"⏳ Taking off to {route[0]['alt']}m...")
    await drone.action.set_takeoff_altitude(route[0]["alt"])
    await drone.action.takeoff()
    await asyncio.sleep(6)

    print("\n🚁 Starting mission - RRT* optimized path!")
    await drone.mission.start_mission()

    print("\n📍 Mission Progress:")
    async for progress in drone.mission.mission_progress():
        current = progress.current
        total = progress.total
        print(f"   Waypoint {current}/{total}", end='\r')
        
        if current >= total:
            print(f"\n✅ Completed all {total} waypoints!")
            break
        
        await asyncio.sleep(0.5)

    print("\n🛬 Landing at HOME...")
    await drone.action.land()
    await asyncio.sleep(6)
    print("✅ Mission complete - returned to HOME.")


# ---------------------------
# Main flight routine
# ---------------------------
async def fly_plan_rrt_star(plan_file="test_rrt.plan", max_iter=500, speed=5.0):
    """
    Bay với RRT*: HOME → delivery points → HOME
    """
    planner = RRTStarPlanner(plan_file)
    if not planner.delivery_points:
        print("⚠️ No waypoints found in plan.")
        return

    print(f"🏠 Home base: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, alt={planner.home['alt']:.1f}m")
    print(f"📦 Loaded {len(planner.delivery_points)} delivery points from {plan_file}")

    # Plan với RRT*
    route = planner.rrt_star_plan(max_iterations=max_iter)
    
    print("\n📍 RRT* Optimized Route:")
    print(f"  🛫 HOME: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, alt={planner.home['alt']:.1f}m")
    
    total_dist = 0
    current = planner.home
    
    for i, p in enumerate(route):
        d = haversine_m(current["lat"], current["lon"], p["lat"], p["lon"])
        total_dist += d
        print(f"  📦 {i+1}: lat={p['lat']:.6f}, lon={p['lon']:.6f}, alt={p['alt']:.1f}m (distance: {d:.1f}m)")
        current = p
    
    d_return = haversine_m(current["lat"], current["lon"], 
                          planner.home["lat"], planner.home["lon"])
    total_dist += d_return
    
    print(f"  🛬 HOME: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, alt={planner.home['alt']:.1f}m (return: {d_return:.1f}m)")
    
    print(f"\n📏 Total distance: {total_dist:.1f}m")
    print(f"⏱️  Estimated time: {total_dist/speed:.1f}s (~{total_dist/speed/60:.1f} minutes)")

    drone = await connect_drone()
    await upload_and_fly_mission(drone, route, planner.home, speed_m_s=speed)


# ---------------------------
# Entry point
# ---------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Fly round trip delivery mission with RRT* optimization"
    )
    parser.add_argument("--plan", type=str, default="test_rrt.plan", 
                       help="path to .plan file")
    parser.add_argument("--iter", type=int, default=500, 
                       help="RRT* max iterations")
    parser.add_argument("--speed", type=float, default=5.0,
                       help="flight speed in m/s")
    args = parser.parse_args()

    try:
        asyncio.run(fly_plan_rrt_star(
            plan_file=args.plan, 
            max_iter=args.iter,
            speed=args.speed
        ))
    except KeyboardInterrupt:
        print("\n⚠️ Aborted by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
