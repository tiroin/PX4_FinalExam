#!/usr/bin/env python3

"""
Load và thực hiện mission từ file QGroundControl .plan
+ Tích hợp thuật toán Waypoint Smoothing (Chaikin Curve)
Tương thích MAVSDK-Python mới nhất
"""

import asyncio
import json
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)


# ================================
# 1) Thuật toán SMOOTH WAYPOINTS
# ================================
def smooth_waypoints(mission_items, iterations=1):
    """
    Làm mượt waypoint bằng thuật toán Chaikin (Corner Cutting)
    Chỉ làm mượt các waypoint bay (không làm mượt TAKEOFF đầu tiên nếu có).
    """

    def interp(a, b, t):
        return a * (1 - t) + b * t

    for _ in range(iterations):
        new_list = []

        # Giữ waypoint đầu tiên (để không thay đổi vị trí cất cánh)
        new_list.append(mission_items[0])

        for i in range(len(mission_items) - 1):
            A = mission_items[i]
            B = mission_items[i + 1]

            # Tạo 2 điểm Q (0.25) và R (0.75)
            lat_Q = interp(A.latitude_deg, B.latitude_deg, 0.25)
            lon_Q = interp(A.longitude_deg, B.longitude_deg, 0.25)
            alt_Q = interp(A.relative_altitude_m, B.relative_altitude_m, 0.25)

            lat_R = interp(A.latitude_deg, B.latitude_deg, 0.75)
            lon_R = interp(A.longitude_deg, B.longitude_deg, 0.75)
            alt_R = interp(A.relative_altitude_m, B.relative_altitude_m, 0.75)

            # Tạo mission items mới Q và R
            Q_item = MissionItem(
                lat_Q, lon_Q, alt_Q,
                5.0, True,
                0, 0, MissionItem.CameraAction.NONE,
                float('nan'), 1.0, 5.0,
                float('nan'), 0.0,
                MissionItem.VehicleAction.NONE
            )

            R_item = MissionItem(
                lat_R, lon_R, alt_R,
                5.0, True,
                0, 0, MissionItem.CameraAction.NONE,
                float('nan'), 1.0, 5.0,
                float('nan'), 0.0,
                MissionItem.VehicleAction.NONE
            )

            new_list.append(Q_item)
            new_list.append(R_item)

        # Giữ waypoint cuối không thay đổi
        new_list.append(mission_items[-1])

        mission_items = new_list

    return mission_items


# ================================
# 2) Load mission từ file .plan
# ================================
def load_mission_from_plan_file(plan_file_path):
    """
    Đọc file .plan của QGroundControl và chuyển thành MissionItems
    """
    with open(plan_file_path, 'r', encoding='utf-8') as f:
        plan_data = json.load(f)

    mission_items = []

    if 'mission' in plan_data and 'items' in plan_data['mission']:
        items = plan_data['mission']['items']

        for idx, item in enumerate(items):
            command = item.get('command')
            params = item.get('params', [0] * 7)

            if command in [16, 22]:  # Waypoint hoặc Takeoff
                mission_item = MissionItem(
                    latitude_deg=params[4],
                    longitude_deg=params[5],
                    relative_altitude_m=params[6],
                    speed_m_s=5.0,
                    is_fly_through=True,
                    gimbal_pitch_deg=0.0,
                    gimbal_yaw_deg=0.0,
                    camera_action=MissionItem.CameraAction.NONE,
                    loiter_time_s=float(params[0]) if params[0] > 0 else float('nan'),
                    camera_photo_interval_s=1.0,
                    acceptance_radius_m=5.0,
                    yaw_deg=float('nan'),
                    camera_photo_distance_m=0.0,
                    vehicle_action=MissionItem.VehicleAction.NONE
                )

                mission_items.append(mission_item)

                cmd_name = "TAKEOFF" if command == 22 else "WAYPOINT"
                print(f"  [{len(mission_items)}] {cmd_name}: "
                      f"Lat={mission_item.latitude_deg:.6f}, "
                      f"Lon={mission_item.longitude_deg:.6f}, "
                      f"Alt={mission_item.relative_altitude_m}m")

    return mission_items


# ================================
# 3) Chạy mission
# ================================
async def run_mission_from_file(plan_file_path):
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("=" * 50)
    print("Đang kết nối với drone...")
    print("=" * 50)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Drone đã kết nối!")
            break

    print("\nĐang chờ GPS & sensors...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ GPS OK!")
            break

    async for home in drone.telemetry.home():
        print(f"\nHome Position: "
              f"{home.latitude_deg:.6f}, {home.longitude_deg:.6f}, "
              f"Alt={home.absolute_altitude_m:.1f}m")
        break

    print(f"\n{'=' * 50}")
    print(f"Đang load mission từ file: {plan_file_path}")
    print(f"{'=' * 50}\n")

    try:
        mission_items = load_mission_from_plan_file(plan_file_path)

        if not mission_items:
            print("❌ Không có waypoint!")
            return

        print(f"✓ Đã load {len(mission_items)} waypoint gốc")

    except Exception as e:
        print(f"❌ Lỗi đọc file: {e}")
        return

    # ---------------------------
    # ÁP DỤNG SMOOTHING
    # ---------------------------
    print("\nĐang làm mượt đường bay...")
    mission_items = smooth_waypoints(mission_items, iterations=1)
    print(f"✓ Sau smoothing: {len(mission_items)} waypoint\n")

    # Upload mission
    mission_plan = MissionPlan(mission_items)
    print("Đang upload mission...")
    await drone.mission.upload_mission(mission_plan)
    print("✓ Upload thành công!")

    await drone.action.arm()
    print("✓ Đã ARM!")

    print("\n===== BẮT ĐẦU MISSION =====\n")
    await drone.mission.start_mission()

    previous = 0

    async for prog in drone.mission.mission_progress():
        if prog.current != previous:
            print(f"📍 Tiến độ: {prog.current}/{prog.total}")
            previous = prog.current

            async for pos in drone.telemetry.position():
                print(f"   Lat={pos.latitude_deg:.6f}, "
                      f"Lon={pos.longitude_deg:.6f}, "
                      f"Alt={pos.relative_altitude_m:.2f}m")
                break

        if prog.current == prog.total:
            print("\n✓ Mission hoàn thành!")
            break

        await asyncio.sleep(0.5)

    # RTL
    print("\n===== RETURN TO LAUNCH =====")
    await drone.action.return_to_launch()

    async for flying in drone.telemetry.in_air():
        if not flying:
            print("\n✓ Đã hạ cánh!")
            break
        await asyncio.sleep(1)

    print("\n===== HOÀN THÀNH =====")


# ================================
# ENTRY POINT
# ================================
if __name__ == "__main__":
    plan_file = "test_rrt.plan"

    try:
        asyncio.run(run_mission_from_file(plan_file))
    except KeyboardInterrupt:
        print("\n⚠️ Mission bị hủy bởi người dùng!")

