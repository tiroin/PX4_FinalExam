#!/usr/bin/env python3
"""
So sánh 4 flight logs: Normal vs Smoothing vs KNN vs RRT
Yêu cầu: pip install pyulog matplotlib numpy
"""

import numpy as np
import matplotlib.pyplot as plt
from pyulog import ULog

def load_ulog(file_path):
    """Load ULog file"""
    print(f"Loading {file_path}...")
    ulog = ULog(file_path)
    return ulog

def extract_position(ulog):
    """Trích xuất vị trí X, Y, Z"""
    try:
        pos_data = ulog.get_dataset('vehicle_local_position').data
        return {
            'time': pos_data['timestamp'] / 1e6,  # Convert to seconds
            'x': pos_data['x'],
            'y': pos_data['y'],
            'z': pos_data['z']
        }
    except:
        print("⚠️ Không tìm thấy vehicle_local_position")
        return None

def extract_velocity(ulog):
    """Trích xuất vận tốc X, Y, Z"""
    try:
        pos_data = ulog.get_dataset('vehicle_local_position').data
        return {
            'time': pos_data['timestamp'] / 1e6,
            'vx': pos_data['vx'],
            'vy': pos_data['vy'],
            'vz': pos_data['vz']
        }
    except:
        print("⚠️ Không tìm thấy velocity")
        return None

def extract_attitude(ulog):
    """Trích xuất góc Roll, Pitch, Yaw"""
    try:
        att_data = ulog.get_dataset('vehicle_attitude').data
        # Convert quaternion to euler angles (simplified)
        return {
            'time': att_data['timestamp'] / 1e6,
            'roll': np.degrees(np.arctan2(
                2.0 * (att_data['q[0]'] * att_data['q[1]'] + att_data['q[2]'] * att_data['q[3]']),
                1.0 - 2.0 * (att_data['q[1]']**2 + att_data['q[2]']**2)
            )),
            'pitch': np.degrees(np.arcsin(
                2.0 * (att_data['q[0]'] * att_data['q[2]'] - att_data['q[3]'] * att_data['q[1]'])
            )),
            'yaw': np.degrees(np.arctan2(
                2.0 * (att_data['q[0]'] * att_data['q[3]'] + att_data['q[1]'] * att_data['q[2]']),
                1.0 - 2.0 * (att_data['q[2]']**2 + att_data['q[3]']**2)
            ))
        }
    except:
        print("⚠️ Không tìm thấy vehicle_attitude")
        return None

def extract_power(ulog):
    """Trích xuất điện áp, dòng điện, công suất"""
    try:
        battery_data = ulog.get_dataset('battery_status').data
        return {
            'time': battery_data['timestamp'] / 1e6,
            'voltage': battery_data['voltage_v'],
            'current': battery_data['current_a'],
            'power': battery_data['voltage_v'] * battery_data['current_a']
        }
    except:
        print("⚠️ Không tìm thấy battery_status")
        return None

def calculate_metrics(ulog, name):
    """Tính toán metrics so sánh"""
    print(f"\n{'='*50}")
    print(f"Metrics cho: {name}")
    print(f"{'='*50}")
    
    metrics = {}
    
    # 1. Position - tổng quãng đường
    pos = extract_position(ulog)
    if pos:
        distances = np.sqrt(np.diff(pos['x'])**2 + np.diff(pos['y'])**2 + np.diff(pos['z'])**2)
        total_distance = np.sum(distances)
        metrics['total_distance'] = total_distance
        print(f"📏 Tổng quãng đường: {total_distance:.1f}m")
    
    # 2. Velocity - vận tốc trung bình
    vel = extract_velocity(ulog)
    if vel:
        speed = np.sqrt(vel['vx']**2 + vel['vy']**2 + vel['vz']**2)
        avg_speed = np.mean(speed)
        max_speed = np.max(speed)
        metrics['avg_speed'] = avg_speed
        metrics['max_speed'] = max_speed
        print(f"🚁 Vận tốc TB: {avg_speed:.2f}m/s")
        print(f"🚁 Vận tốc MAX: {max_speed:.2f}m/s")
        
        # Độ mượt (standard deviation thấp = mượt)
        speed_std = np.std(speed)
        metrics['speed_smoothness'] = speed_std
        print(f"📊 Độ mượt (std): {speed_std:.2f} (thấp = mượt)")
    
    # 3. Attitude - góc nghiêng
    att = extract_attitude(ulog)
    if att:
        roll_std = np.std(att['roll'])
        pitch_std = np.std(att['pitch'])
        yaw_changes = np.sum(np.abs(np.diff(att['yaw'])))
        
        metrics['roll_std'] = roll_std
        metrics['pitch_std'] = pitch_std
        metrics['yaw_changes'] = yaw_changes
        
        print(f"🔄 Roll std: {roll_std:.2f}° (thấp = ổn định)")
        print(f"🔄 Pitch std: {pitch_std:.2f}°")
        print(f"🔄 Tổng góc xoay Yaw: {yaw_changes:.1f}° (thấp = ít rẽ)")
    
    # 4. Power - năng lượng
    power = extract_power(ulog)
    if power:
        avg_current = np.mean(power['current'])
        avg_power = np.mean(power['power'])
        total_energy = np.trapz(power['power'], power['time']) / 3600  # Wh
        
        metrics['avg_current'] = avg_current
        metrics['avg_power'] = avg_power
        metrics['total_energy'] = total_energy
        
        print(f"⚡ Dòng điện TB: {avg_current:.2f}A")
        print(f"⚡ Công suất TB: {avg_power:.2f}W")
        print(f"⚡ Tổng năng lượng: {total_energy:.2f}Wh")
    
    # 5. Thời gian bay
    if pos:
        flight_time = pos['time'][-1] - pos['time'][0]
        metrics['flight_time'] = flight_time
        print(f"⏱️  Thời gian bay: {flight_time:.1f}s ({flight_time/60:.1f} phút)")
    
    return metrics

def plot_comparison(ulog_list, name_list):
    """Vẽ đồ thị so sánh"""
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f'So sánh: {" vs ".join(name_list)}', fontsize=16, fontweight='bold')
    
    colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']
    
    # 1. Position X-Y (Quỹ đạo 2D)
    positions = [extract_position(ulog) for ulog in ulog_list]
    if all(pos for pos in positions):
        for i, pos in enumerate(positions):
            axes[0, 0].plot(pos['y'], pos['x'], linewidth=2, label=name_list[i], color=colors[i])
        axes[0, 0].set_xlabel('Y (m)')
        axes[0, 0].set_ylabel('X (m)')
        axes[0, 0].set_title('Quỹ đạo bay (2D)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].axis('equal')
    
    # 2. Altitude
    if all(pos for pos in positions):
        for i, pos in enumerate(positions):
            axes[0, 1].plot(pos['time'], -pos['z'], linewidth=2, label=name_list[i], color=colors[i])
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Altitude (m)')
        axes[0, 1].set_title('Độ cao theo thời gian')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
    
    # 3. Velocity
    velocities = [extract_velocity(ulog) for ulog in ulog_list]
    if all(vel for vel in velocities):
        for i, vel in enumerate(velocities):
            speed = np.sqrt(vel['vx']**2 + vel['vy']**2 + vel['vz']**2)
            axes[1, 0].plot(vel['time'], speed, linewidth=2, label=name_list[i], color=colors[i])
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Speed (m/s)')
        axes[1, 0].set_title('Vận tốc')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
    
    # 4. Roll Angle
    attitudes = [extract_attitude(ulog) for ulog in ulog_list]
    if all(att for att in attitudes):
        for i, att in enumerate(attitudes):
            axes[1, 1].plot(att['time'], att['roll'], linewidth=2, label=name_list[i], color=colors[i])
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Roll (deg)')
        axes[1, 1].set_title('Góc Roll')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
    
    # 5. Pitch Angle
    if all(att for att in attitudes):
        for i, att in enumerate(attitudes):
            axes[2, 0].plot(att['time'], att['pitch'], linewidth=2, label=name_list[i], color=colors[i])
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Pitch (deg)')
        axes[2, 0].set_title('Góc Pitch')
        axes[2, 0].legend()
        axes[2, 0].grid(True)
    
    # 6. Power
    powers = [extract_power(ulog) for ulog in ulog_list]
    if all(power for power in powers):
        for i, power in enumerate(powers):
            axes[2, 1].plot(power['time'], power['current'], linewidth=2, label=name_list[i], color=colors[i])
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_ylabel('Current (A)')
        axes[2, 1].set_title('Dòng điện')
        axes[2, 1].legend()
        axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('flight_comparison.png', dpi=300, bbox_inches='tight')
    print("\n✅ Đã lưu đồ thị: flight_comparison.png")
    plt.show()

def compare_metrics(metrics_list, name_list):
    """So sánh metrics và tính % cải thiện"""
    print(f"\n{'='*80}")
    print(f"SO SÁNH TỔNG QUAN: {' vs '.join(name_list)}")
    print(f"{'='*80}\n")
    
    comparisons = {
        'total_distance': ('Quãng đường', 'm', 'lower'),
        'flight_time': ('Thời gian bay', 's', 'lower'),
        'avg_speed': ('Vận tốc TB', 'm/s', 'higher'),
        'speed_smoothness': ('Độ mượt', '', 'lower'),
        'roll_std': ('Roll std', '°', 'lower'),
        'pitch_std': ('Pitch std', '°', 'lower'),
        'yaw_changes': ('Góc xoay Yaw', '°', 'lower'),
        'avg_current': ('Dòng điện TB', 'A', 'lower'),
        'avg_power': ('Công suất TB', 'W', 'lower'),
        'total_energy': ('Năng lượng', 'Wh', 'lower')
    }
    
    for key, (label, unit, better) in comparisons.items():
        values = []
        for metrics in metrics_list:
            if key in metrics:
                values.append(metrics[key])
            else:
                values.append(None)
        
        if all(v is not None for v in values):
            print(f"\n{label}:")
            for i, (name, val) in enumerate(zip(name_list, values)):
                print(f"   {name}: {val:.2f}{unit}")
            
            if better == 'lower':
                best_idx = values.index(min(values))
                symbol = '✅' if best_idx == 0 else '⚠️'
            else:
                best_idx = values.index(max(values))
                symbol = '✅' if best_idx == 0 else '⚠️'
            
            print(f"   {symbol} Tốt nhất: {name_list[best_idx]}")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 5:
        print("Usage: python compare_flight_logs.py <normal.ulg> <smoothing.ulg> <knn.ulg> <rrt.ulg>")
        print("\nExample:")
        print("  python compare_flight_logs.py normal.ulg smoothing.ulg knn.ulg rrt.ulg")
        sys.exit(1)
    
    file_paths = sys.argv[1:5]
    names = ["Normal", "Smoothing", "KNN", "RRT"]
    
    try:
        # Load logs
        ulogs = [load_ulog(file_path) for file_path in file_paths]
        
        # Calculate metrics
        metrics_list = [calculate_metrics(ulog, name) for ulog, name in zip(ulogs, names)]
        
        # Compare
        compare_metrics(metrics_list, names)
        
        # Plot
        plot_comparison(ulogs, names)
        
    except FileNotFoundError as e:
        print(f"❌ Không tìm thấy file: {e}")
    except Exception as e:
        print(f"❌ Lỗi: {e}")
        import traceback
        traceback.print_exc()
