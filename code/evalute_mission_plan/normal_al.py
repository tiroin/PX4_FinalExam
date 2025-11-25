#!/usr/bin/env python3

"""
Load và thực hiện mission từ file QGroundControl .plan
Tương thích với MAVSDK-Python mới nhất
"""

import asyncio
import json
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)


def load_mission_from_plan_file(plan_file_path):
    """
    Đọc file .plan của QGroundControl và chuyển thành MissionItems
    
    Args:
        plan_file_path: Đường dẫn đến file .plan
        
    Returns:
        List of MissionItem objects
    """
    with open(plan_file_path, 'r', encoding='utf-8') as f:
        plan_data = json.load(f)
    
    mission_items = []
    
    # Lấy danh sách waypoints từ file plan
    if 'mission' in plan_data and 'items' in plan_data['mission']:
        items = plan_data['mission']['items']
        
        for idx, item in enumerate(items):
            command = item.get('command')
            params = item.get('params', [0] * 7)
            
            # Command 22: Takeoff
            # Command 16: Waypoint
            # Command 20: Return to Launch
            if command in [16, 22]:
                
                # Tạo MissionItem với tất cả tham số bắt buộc
                mission_item = MissionItem(
                    latitude_deg=params[4],
                    longitude_deg=params[5],
                    relative_altitude_m=params[6],
                    speed_m_s=5.0,  # Tốc độ mặc định 5 m/s
                    is_fly_through=True,  # Bay qua waypoint, không hover
                    gimbal_pitch_deg=0.0,
                    gimbal_yaw_deg=0.0,
                    camera_action=MissionItem.CameraAction.NONE,
                    loiter_time_s=float(params[0]) if params[0] > 0 else float('nan'),
                    camera_photo_interval_s=1.0,
                    acceptance_radius_m=5.0,  # Bán kính chấp nhận đến waypoint
                    yaw_deg=float('nan'),  # Hướng mặt, NaN = tự động
                    camera_photo_distance_m=0.0,  # Khoảng cách giữa các ảnh
                    vehicle_action=MissionItem.VehicleAction.NONE  # Hành động đặc biệt
                )
                
                mission_items.append(mission_item)
                
                cmd_name = "TAKEOFF" if command == 22 else "WAYPOINT"
                print(f"  [{len(mission_items)}] {cmd_name}: "
                      f"Lat={mission_item.latitude_deg:.6f}, "
                      f"Lon={mission_item.longitude_deg:.6f}, "
                      f"Alt={mission_item.relative_altitude_m}m")
    
    return mission_items


async def run_mission_from_file(plan_file_path):
    """
    Kết nối drone và thực hiện mission từ file .plan
    """
    
    # Kết nối drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("=" * 50)
    print("Đang kết nối với drone...")
    print("=" * 50)
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Drone đã kết nối!")
            break

    print("\nĐang chờ drone sẵn sàng (GPS, sensors)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ GPS và sensors sẵn sàng!")
            break

    # Hiển thị home position
    async for terrain_info in drone.telemetry.home():
        print(f"\nHome Position:")
        print(f"  Lat: {terrain_info.latitude_deg:.6f}")
        print(f"  Lon: {terrain_info.longitude_deg:.6f}")
        print(f"  Alt: {terrain_info.absolute_altitude_m:.1f}m")
        break

    # Load mission từ file
    print(f"\n{'=' * 50}")
    print(f"Đang load mission từ file: {plan_file_path}")
    print(f"{'=' * 50}\n")
    
    try:
        mission_items = load_mission_from_plan_file(plan_file_path)
        
        if not mission_items:
            print("❌ Không tìm thấy waypoints trong file!")
            return
            
        print(f"\n✓ Đã load {len(mission_items)} waypoints")
        
    except FileNotFoundError:
        print(f"❌ Không tìm thấy file: {plan_file_path}")
        return
    except Exception as e:
        print(f"❌ Lỗi khi đọc file: {e}")
        import traceback
        traceback.print_exc()
        return

    # Tạo mission plan
    mission_plan = MissionPlan(mission_items)

    # Upload mission lên drone
    print("\nĐang upload mission lên drone...")
    await drone.mission.upload_mission(mission_plan)
    print("✓ Upload mission thành công!")

    # Arm drone
    print("\nĐang arm drone...")
    await drone.action.arm()
    print("✓ Drone đã armed!")

    # Bắt đầu mission
    print("\n" + "=" * 50)
    print("BẮT ĐẦU MISSION")
    print("=" * 50 + "\n")
    
    await drone.mission.start_mission()

    # Theo dõi tiến độ mission
    print("Đang thực hiện mission...\n")
    
    previous_current = 0
    
    async for mission_progress in drone.mission.mission_progress():
        # Chỉ in khi có thay đổi waypoint
        if mission_progress.current != previous_current:
            print(f"\n📍 Tiến độ: {mission_progress.current}/{mission_progress.total} waypoints")
            previous_current = mission_progress.current
            
            # Hiển thị vị trí hiện tại
            async for position in drone.telemetry.position():
                print(f"   Vị trí: Lat={position.latitude_deg:.6f}, "
                      f"Lon={position.longitude_deg:.6f}, "
                      f"Alt={position.relative_altitude_m:.2f}m")
                break
        
        if mission_progress.current == mission_progress.total:
            print("\n✓ Mission hoàn thành!")
            break
        
        await asyncio.sleep(0.5)

    # Return to launch (RTL)
    print("\n" + "=" * 50)
    print("QUAY VỀ VỊ TRÍ XUẤT PHÁT VÀ HẠ CÁNH")
    print("=" * 50 + "\n")
    
    await drone.action.return_to_launch()
    
    print("Đang bay về home và hạ cánh...")

    # Chờ hạ cánh hoàn toàn
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("\n✓ Đã hạ cánh!")
            break
        await asyncio.sleep(1)

    print("\n" + "=" * 50)
    print("HOÀN THÀNH!")
    print("=" * 50)


if __name__ == "__main__":
    # Đường dẫn đến file .plan
    plan_file = "test_rrt.plan"
    
    # Chạy mission
    try:
        asyncio.run(run_mission_from_file(plan_file))
    except KeyboardInterrupt:
        print("\n\n⚠️  Mission bị hủy bởi người dùng!")
