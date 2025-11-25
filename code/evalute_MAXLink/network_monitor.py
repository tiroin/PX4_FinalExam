#!/usr/bin/env python3
"""
MAVSDK Network Monitoring Tool
Kiểm tra các thông số mạng khi kết nối drone
Yêu cầu: pip install mavsdk asyncio
"""

import asyncio
import time
from mavsdk import System
from mavsdk.server_utility import ServerUtility
import statistics

class NetworkMonitor:
    def __init__(self, connection_url="udp://:14540"):
        self.drone = System()
        self.connection_url = connection_url
        self.heartbeat_times = []
        self.packet_loss_count = 0
        self.total_packets = 0
        self.latency_samples = []
        self.start_time = None
        self.message_count = {}
        
    async def connect(self):
        """Kết nối tới drone"""
        print(f"🔗 Đang kết nối tới: {self.connection_url}")
        
        try:
            await self.drone.connect(system_address=self.connection_url)
            
            # Chờ heartbeat
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("✅ Kết nối thành công!")
                    self.start_time = time.time()
                    break
        except Exception as e:
            print(f"❌ Lỗi kết nối: {e}")
            return False
        
        return True
    
    async def monitor_heartbeat(self, duration=10):
        """Giám sát heartbeat"""
        print(f"\n{'='*60}")
        print(f"📡 HEARTBEAT MONITORING ({duration}s)")
        print(f"{'='*60}")
        
        heartbeat_count = 0
        last_heartbeat = time.time()
        heartbeat_intervals = []
        start = time.time()
        
        try:
            async for position in self.drone.telemetry.position():
                current_time = time.time()
                interval = current_time - last_heartbeat
                
                if interval > 0.01:
                    heartbeat_intervals.append(interval)
                
                heartbeat_count += 1
                last_heartbeat = current_time
                
                if current_time - start >= duration:
                    break
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
        
        if heartbeat_intervals:
            print(f"📊 Heartbeats nhận được: {heartbeat_count}")
            print(f"📊 Khoảng thời gian TB: {statistics.mean(heartbeat_intervals)*1000:.2f} ms")
            print(f"📊 Khoảng thời gian MIN: {min(heartbeat_intervals)*1000:.2f} ms")
            print(f"📊 Khoảng thời gian MAX: {max(heartbeat_intervals)*1000:.2f} ms")
            if len(heartbeat_intervals) > 1:
                print(f"📊 Std Dev: {statistics.stdev(heartbeat_intervals)*1000:.2f} ms")
    
    async def monitor_telemetry(self, duration=10):
        """Giám sát telemetry"""
        print(f"\n{'='*60}")
        print(f"📊 TELEMETRY MONITORING ({duration}s)")
        print(f"{'='*60}")
        
        telemetry_count = 0
        start = time.time()
        
        try:
            async for telemetry in self.drone.telemetry.position():
                telemetry_count += 1
                
                # In thông tin
                if telemetry_count % 5 == 1:
                    print(f"📍 Position: ({telemetry.latitude_deg:.6f}, {telemetry.longitude_deg:.6f})")
                
                if time.time() - start >= duration:
                    break
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
        
        print(f"📊 Telemetry count: {telemetry_count}")
    
    async def monitor_system_info(self):
        """Lấy thông tin hệ thống"""
        print(f"\n{'='*60}")
        print(f"🖥️ SYSTEM INFORMATION")
        print(f"{'='*60}")
        
        try:
            # Flight info
            async for flight_mode in self.drone.telemetry.flight_mode():
                print(f"✈️ Flight Mode: {flight_mode}")
                break
            
            # GPS Info
            async for gps_info in self.drone.telemetry.gps_info():
                print(f"📡 GPS Satellites: {gps_info.num_satellites}")
                print(f"📡 GPS Fix Type: {gps_info.fix_type}")
                break
            
            # Battery
            async for battery in self.drone.telemetry.battery():
                print(f"🔋 Battery: {battery.remaining_percent*100:.1f}%")
                print(f"🔋 Voltage: {battery.voltage_v:.2f}V")
                break
            
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
    
    async def monitor_connection_status(self):
        """Giám sát trạng thái kết nối"""
        print(f"\n{'='*60}")
        print(f"🔗 CONNECTION STATUS")
        print(f"{'='*60}")
        
        try:
            async for state in self.drone.core.connection_state():
                status = "✅ Connected" if state.is_connected else "❌ Disconnected"
                print(f"Status: {status}")
                print(f"Is Connected: {state.is_connected}")
                break  # Chỉ lấy 1 lần
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
    
    async def measure_latency(self, samples=10):
        """Đo độ trễ (latency)"""
        print(f"\n{'='*60}")
        print(f"⏱️ LATENCY MEASUREMENT ({samples} samples)")
        print(f"{'='*60}")
        
        latencies = []
        
        try:
            for i in range(samples):
                send_time = time.time()
                
                # Gửi yêu cầu và đợi phản hồi
                async for position in self.drone.telemetry.position():
                    recv_time = time.time()
                    latency = (recv_time - send_time) * 1000  # ms
                    latencies.append(latency)
                    print(f"Sample {i+1}: {latency:.2f} ms")
                    break
                
                await asyncio.sleep(0.1)
        
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
        
        if latencies:
            print(f"\n📊 Latency Statistics:")
            print(f"   Average: {statistics.mean(latencies):.2f} ms")
            print(f"   Min: {min(latencies):.2f} ms")
            print(f"   Max: {max(latencies):.2f} ms")
            print(f"   Std Dev: {statistics.stdev(latencies):.2f} ms")
    
    async def monitor_attitude(self, duration=5):
        """Giám sát attitude (roll, pitch, yaw)"""
        print(f"\n{'='*60}")
        print(f"📐 ATTITUDE MONITORING ({duration}s)")
        print(f"{'='*60}")
        
        start = time.time()
        count = 0
        
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                count += 1
                if count % 3 == 1:
                    print(f"Roll: {attitude.roll_deg:6.2f}° | "
                          f"Pitch: {attitude.pitch_deg:6.2f}° | "
                          f"Yaw: {attitude.yaw_deg:6.2f}°")
                
                if time.time() - start >= duration:
                    break
        
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
    
    async def monitor_imu(self, duration=5):
        """Giám sát IMU (gia tốc, tốc độ góc)"""
        print(f"\n{'='*60}")
        print(f"📊 IMU MONITORING ({duration}s)")
        print(f"{'='*60}")
        
        accel_values = []
        gyro_values = []
        start = time.time()
        count = 0
        
        try:
            async for imu_data in self.drone.telemetry.imu():
                # Lấy dữ liệu acceleration
                accel = imu_data.acceleration_frd
                ax, ay, az = accel.forward_m_s2, accel.right_m_s2, accel.down_m_s2
                
                # Lấy dữ liệu angular velocity - sử dụng thuộc tính đúng
                gyro = imu_data.angular_velocity_frd
                gx, gy, gz = gyro.forward_rad_s, gyro.right_rad_s, gyro.down_rad_s
                
                accel_mag = (ax**2 + ay**2 + az**2) ** 0.5
                gyro_mag = (gx**2 + gy**2 + gz**2) ** 0.5
                
                accel_values.append(accel_mag)
                gyro_values.append(gyro_mag)
                
                count += 1
                if count % 3 == 1:
                    print(f"Accel: {accel_mag:.3f} m/s² | Gyro: {gyro_mag:.3f} rad/s")
                
                if time.time() - start >= duration:
                    break
        
        except Exception as e:
            print(f"⚠️ Lỗi: {e}")
        
        if accel_values and gyro_values:
            print(f"\n📊 Statistics:")
            print(f"   Accel Avg: {statistics.mean(accel_values):.3f} m/s²")
            print(f"   Gyro Avg: {statistics.mean(gyro_values):.3f} rad/s")
    
    async def full_network_report(self):
        """Báo cáo mạng đầy đủ"""
        print("\n" + "="*60)
        print("🌐 FULL NETWORK REPORT")
        print("="*60)
        
        await self.monitor_connection_status()
        await self.monitor_system_info()
        await self.measure_latency(samples=5)
        await self.monitor_heartbeat(duration=5)
        await self.monitor_telemetry(duration=3)
        await self.monitor_attitude(duration=3)
        await self.monitor_imu(duration=3)
        
        print("\n" + "="*60)
        print("✅ Báo cáo hoàn tất!")
        print("="*60 + "\n")
    
    async def disconnect(self):
        """Ngắt kết nối"""
        print("\n🔌 Đang ngắt kết nối...")
        try:
            await self.drone.action.disarm()
        except:
            pass
        print("✅ Đã ngắt kết nối")


async def main():
    import sys
    
    # Lấy connection URL từ tham số
    connection_url = sys.argv[1] if len(sys.argv) > 1 else "udp://:14540"
    
    monitor = NetworkMonitor(connection_url)
    
    # Kết nối
    if await monitor.connect():
        try:
            # Chạy báo cáo đầy đủ
            await monitor.full_network_report()
        except KeyboardInterrupt:
            print("\n\n⚠️ Bị ngắt!")
        finally:
            await monitor.disconnect()
    else:
        print("❌ Không thể kết nối tới drone")


if __name__ == "__main__":
    print("🚁 MAVSDK Network Monitoring Tool")
    print("📌 Usage: python network_monitor.py [connection_url]")
    print("📌 Example: python network_monitor.py udp://:14540")
    print("📌 Example: python network_monitor.py tcp://localhost:5760")
    print("📌 Example: python network_monitor.py serial:///dev/ttyUSB0:921600\n")
    
    asyncio.run(main())
