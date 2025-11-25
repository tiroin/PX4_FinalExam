# PX4_FinalExam
Đây là dự án của Quý + Tâm về đề tài:  Mô phỏng UAV và so sánh thuật toán hoạch định  đường bay trên PX4 – QGroundControl – Gazebo dựa trên MAVLink
Sau đây là 1 số hướng dẫn cách cài và 1 số lệnh chính trên terminal về việc mô phỏng của hệ thống này:

# Các công cụ cần thiết
1. PX4 Autopilot (Firmware)
Dùng để chạy SITL (Software In The Loop).
Bao gồm bộ điều khiển bay, mô hình động lực học UAV, các thuật toán điều hướng.
2. Gazebo / Ignition Gazebo
Môi trường mô phỏng 3D.
3. QGroundControl (QGC)
Giao diện điều khiển UAV.
Cấu hình tham số, gửi mission, xem bản đồ & telemetry.
4. MAVSDK / MAVLink
Giao thức truyền thông giữa PX4 ↔ QGC ↔ Scripts Python.
MAVSDK dùng để viết mã tự động hóa, thu thập dữ liệu, giám sát telemetry.
5. Python 3 + MAVSDK-Python
Dùng để chạy script kiểm thử MAVLink và so sánh thuật toán.
6. Ubuntu 20.04 / 22.04
Hệ điều hành chuẩn để chạy PX4 + Gazebo.

# Một số lệnh chính trên các terminal
Mở QRC
cd ~/Downloads
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage

//////////////////////////////////////////////////////////
Khởi tạo Drone
cd ~/Desktop/PX4-Autopilot
make px4_sitl gz_x500

//////////////////////////////////////////////////////////
Chạy network_monitor để kiểm tra MAVLink
cd ~/Desktop/
source mavsdk_venv/bin/activate
python network_monitor.py udp://:14540

//////////////////////////////////////////////////////////
Chạy 4 thuật toán và so sánh chúng
cd ~/Desktop/
source mavsdk_venv/bin/activate
python normal_al.py
python smoothing_al.py
python knn_al.py
python rrt_al.py

python compare_al.py normal.ulg smoothing.ulg knn.ulg rrt.ulg
