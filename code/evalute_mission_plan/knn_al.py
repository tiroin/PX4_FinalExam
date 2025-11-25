#!/usr/bin/env python3
"""
drone_knn_continuous.py

Bay liên tục không dừng lại giữa các waypoint
Cất cánh từ home, giao hàng các waypoint, quay về home hạ cánh
"""

import asyncio
import json
import math
import time
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


def angle_difference(angle1, angle2):
    """Tính chênh lệch góc nhỏ nhất giữa 2 hướng (0-180 degrees)"""
    diff = abs(angle1 - angle2)
    if diff > 180.0:
        diff = 360.0 - diff
    return diff


# ---------------------------
# Planner: KNN Algorithm
# ---------------------------
class DroneRoutePlanner:
    def __init__(self, plan_path):
        self.plan_path = plan_path
        self.delivery_points = []
        self.home = None
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
                
                # ⚠️ KIỂM TRA TỌA ĐỘ HỢP LỆ
                if lat != 0.0 and lon != 0.0:  # Bỏ qua tọa độ (0,0)
                    self.delivery_points.append({"lat": lat, "lon": lon, "alt": alt})
                else:
                    print(f"⚠️ Skipping invalid waypoint: lat={lat}, lon={lon}")

        # Home position - điểm cất cánh và hạ cánh
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

    def k_nearest_neighbors_round_trip(self, k=3, w_distance=0.6, w_angle=0.3, 
                                      w_lookahead=0.1, alpha=3.0, k_lookahead=2):
        """
        THUẬT TOÁN KNN CHO ROUND TRIP
        
        - Bắt đầu từ HOME
        - Tối ưu hóa tất cả delivery points bằng KNN
        - Quay về HOME ở cuối
        """
        if not self.delivery_points or self.home is None:
            return []

        print(f"\n🏠 ROUND TRIP MODE")
        print(f"   🛫 Cất cánh từ HOME: lat={self.home['lat']:.6f}, lon={self.home['lon']:.6f}")
        print(f"   📦 Giao hàng {len(self.delivery_points)} điểm")
        print(f"   🛬 Quay về HOME hạ cánh\n")

        # Tối ưu tất cả delivery points
        remaining = self.delivery_points.copy()
        route = []
        current = self.home  # Bắt đầu từ home
        last_heading = None

        print(f"🔍 KNN Algorithm (k={k}, α={alpha})")
        print(f"   Trọng số: distance={w_distance}, angle={w_angle}, lookahead={w_lookahead}\n")

        step = 0
        while remaining:
            step += 1
            # Tìm k waypoint gần nhất
            distances = []
            for p in remaining:
                d = haversine_m(current["lat"], current["lon"], p["lat"], p["lon"])
                distances.append((d, p))
            
            distances.sort(key=lambda x: x[0])
            candidates = [p for (d, p) in distances[:min(k, len(distances))]]
            
            print(f"Bước {step}: Xét {len(candidates)} candidates")
            
            # Đánh giá từng candidate
            best = None
            best_score = float("inf")
            best_details = None
            
            for idx, wp in enumerate(candidates):
                # Distance cost
                dist = haversine_m(current["lat"], current["lon"], wp["lat"], wp["lon"])
                cost_distance = dist
                
                # Angle penalty cost
                new_heading = bearing_deg(current["lat"], current["lon"], wp["lat"], wp["lon"])
                if last_heading is None:
                    angle_diff = 0.0
                else:
                    angle_diff = angle_difference(new_heading, last_heading)
                cost_angle = alpha * angle_diff
                
                # Look-ahead cost (bao gồm cả khoảng cách về home)
                remaining_after = [p for p in remaining if p != wp]
                if len(remaining_after) > 0:
                    future_dists = []
                    for future_wp in remaining_after:
                        fd = haversine_m(wp["lat"], wp["lon"], 
                                       future_wp["lat"], future_wp["lon"])
                        future_dists.append(fd)
                    future_dists.sort()
                    lookahead_dists = future_dists[:min(k_lookahead, len(future_dists))]
                    cost_lookahead = sum(lookahead_dists) / len(lookahead_dists) if lookahead_dists else 0
                else:
                    # Waypoint cuối cùng - xét khoảng cách về home
                    cost_lookahead = haversine_m(wp["lat"], wp["lon"], 
                                                self.home["lat"], self.home["lon"]) * 0.5
                
                # TỔNG SCORE
                total_score = (w_distance * cost_distance + 
                             w_angle * cost_angle + 
                             w_lookahead * cost_lookahead)
                
                print(f"  [{idx+1}] d={dist:.1f}m, Δθ={angle_diff:.1f}°, "
                      f"lookahead={cost_lookahead:.1f}m → score={total_score:.1f}")
                
                if total_score < best_score:
                    best_score = total_score
                    best = wp
                    best_details = {
                        'dist': dist,
                        'angle': angle_diff,
                        'lookahead': cost_lookahead,
                        'heading': new_heading
                    }
            
            print(f"  ✅ Chọn: d={best_details['dist']:.1f}m, "
                  f"Δθ={best_details['angle']:.1f}°, score={best_score:.1f}\n")
            
            route.append(best)
            last_heading = best_details['heading']
            current = best
            remaining.remove(best)

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
    
    # Tạo mission items (bao gồm cả return to home)
    mission_items = []
    
    # Các delivery waypoints
    for idx, wp in enumerate(route):
        mission_item = MissionItem(
            latitude_deg=wp["lat"],
            longitude_deg=wp["lon"],
            relative_altitude_m=wp["alt"],
            speed_m_s=speed_m_s,
            is_fly_through=True,  # ✨ BAY QUA KHÔNG DỪNG
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
    
    # Thêm waypoint quay về HOME - bay nhanh như waypoint thường
    return_home_item = MissionItem(
        latitude_deg=home["lat"],
        longitude_deg=home["lon"],
        relative_altitude_m=home["alt"],
        speed_m_s=speed_m_s,  # ✨ GIỮ TỐC ĐỘ CAO
        is_fly_through=True,  # ✨ BAY NHANH, KHÔNG GIẢM TỐC
        gimbal_pitch_deg=float('nan'),
        gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0,
        camera_photo_interval_s=0.0,
        acceptance_radius_m=3.0,  # ✨ Bán kính vừa phải
        yaw_deg=float('nan'),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE
    )
    mission_items.append(return_home_item)

    # Upload mission
    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    print("✅ Mission uploaded successfully!")

    # Arm và takeoff
    await wait_armable(drone)
    print("⏳ Arming...")
    await drone.action.arm()
    
    print(f"⏳ Taking off to {route[0]['alt']}m...")
    await drone.action.set_takeoff_altitude(route[0]["alt"])
    await drone.action.takeoff()
    await asyncio.sleep(6)

    # Start mission
    print("\n🚁 Starting mission - bay liên tục không dừng!")
    await drone.mission.start_mission()

    # Monitor progress
    print("\n📍 Mission Progress:")
    async for progress in drone.mission.mission_progress():
        current = progress.current
        total = progress.total
        print(f"   Waypoint {current}/{total}", end='\r')
        
        if current >= total:
            print(f"\n✅ Completed all {total} waypoints!")
            break
        
        await asyncio.sleep(0.5)

    # Land
    print("\n🛬 Landing at HOME...")
    await drone.action.land()
    await asyncio.sleep(6)
    print("✅ Mission complete - returned to HOME.")


# ---------------------------
# Main flight routine
# ---------------------------
async def fly_plan_continuous(plan_file="test_rrt.plan", k=3, alpha=3.0, speed=5.0):
    """
    Bay round trip: HOME → delivery points → HOME
    """
    planner = DroneRoutePlanner(plan_file)
    if not planner.delivery_points:
        print("⚠️ No waypoints found in plan.")
        return

    print(f"🏠 Home base: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, alt={planner.home['alt']:.1f}m")
    print(f"📦 Loaded {len(planner.delivery_points)} delivery points from {plan_file}")

    # Tính route với KNN (round trip)
    route = planner.k_nearest_neighbors_round_trip(k=k, alpha=alpha)
    
    print("\n📍 Optimized route (KNN Round Trip):")
    print(f"  🛫 HOME: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, "
          f"alt={planner.home['alt']:.1f}m")
    
    total_dist = 0
    current = planner.home
    
    for i, p in enumerate(route):
        d = haversine_m(current["lat"], current["lon"], p["lat"], p["lon"])
        total_dist += d
        print(f"  📦 {i+1}: lat={p['lat']:.6f}, lon={p['lon']:.6f}, "
              f"alt={p['alt']:.1f}m (distance: {d:.1f}m)")
        current = p
    
    # Khoảng cách từ waypoint cuối về home
    d_return = haversine_m(current["lat"], current["lon"], 
                          planner.home["lat"], planner.home["lon"])
    total_dist += d_return
    
    print(f"  🛬 HOME: lat={planner.home['lat']:.6f}, lon={planner.home['lon']:.6f}, "
          f"alt={planner.home['alt']:.1f}m (return distance: {d_return:.1f}m)")
    
    print(f"\n📏 Total distance: {total_dist:.1f}m")
    print(f"⏱️  Estimated time: {total_dist/speed:.1f}s (~{total_dist/speed/60:.1f} minutes)")

    # Connect and fly
    drone = await connect_drone()
    await upload_and_fly_mission(drone, route, planner.home, speed_m_s=speed)


# ---------------------------
# Entry point
# ---------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Fly round trip delivery mission with KNN optimization"
    )
    parser.add_argument("--plan", type=str, default="test_rrt.plan", 
                       help="path to .plan file")
    parser.add_argument("--k", type=int, default=3, 
                       help="number of candidates (k=3-5 recommended)")
    parser.add_argument("--alpha", type=float, default=3.0,
                       help="angle penalty weight (meters per degree)")
    parser.add_argument("--speed", type=float, default=5.0,
                       help="flight speed in m/s")
    args = parser.parse_args()

    try:
        asyncio.run(fly_plan_continuous(
            plan_file=args.plan, 
            k=args.k, 
            alpha=args.alpha,
            speed=args.speed
        ))
    except KeyboardInterrupt:
        print("\n⚠️ Aborted by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
