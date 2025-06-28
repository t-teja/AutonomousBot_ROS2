# Step-by-Step Launch Guide

Since the full system launch is having issues, let's launch components individually to test the web interface properly.

## 🔧 **Step 1: Launch Individual Nodes**

### **Terminal 1: Motor Control**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch motor_control_pkg motor_control.launch.py
```

### **Terminal 2: Battery Monitor**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch battery_monitor_pkg battery_monitor.launch.py
```

### **Terminal 3: OLED Display**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch oled_display_pkg oled_display.launch.py
```

### **Terminal 4: RPLidar (Optional)**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyACM1
```

### **Terminal 5: Web Interface**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch web_interface_pkg web_interface.launch.py
```

## 🔍 **Step 2: Verify Each Component**

### **Check Nodes Are Running:**
```bash
ros2 node list
# Should show:
# /motor_control_node
# /battery_monitor_node
# /oled_display_node
# /rplidar_node (if launched)
# /web_server
# /ros_bridge
```

### **Check Topics:**
```bash
ros2 topic list
# Should show battery, motor, and web topics
```

### **Test Individual Components:**

#### **Battery Data:**
```bash
ros2 topic echo /battery/voltage --once
ros2 topic echo /battery/percentage --once
```

#### **Motor Control:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' --once
```

#### **OLED Display:**
```bash
ros2 topic pub /oled_display_message std_msgs/String '{data: "Test Message"}' --once
```

## 🌐 **Step 3: Test Web Interface**

### **Access Web Interface:**
1. Open browser: http://localhost:8080
2. Check connection status (should show "Connected")
3. Test each section:
   - Battery data should show real values
   - Robot status should show IP address
   - Joystick should send cmd_vel when moved
   - OLED message should work

### **Debug Web Interface:**
```bash
# Check web server
curl http://localhost:8080/api/robot_info

# Check WebSocket (if wscat installed)
# npm install -g wscat
# wscat -c ws://localhost:8765
```

## 🔧 **Step 4: Troubleshooting**

### **If No Battery Data in Web:**
1. Check battery node is publishing:
   ```bash
   ros2 topic hz /battery/voltage
   ```
2. Check WebSocket connection in browser console (F12)
3. Restart ros_bridge node

### **If Motor Control Not Working:**
1. Check cmd_vel is being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```
2. Move joystick in web interface
3. Should see Twist messages

### **If Web Interface Won't Load:**
1. Check web server is running:
   ```bash
   netstat -tulpn | grep :8080
   ```
2. Try different browser
3. Check firewall settings

## 🎯 **Expected Results**

When everything is working correctly:

### **✅ Web Interface Should Show:**
- **Battery**: Real voltage (11-12V), percentage (0-100%)
- **Robot Status**: Actual robot IP address
- **System Info**: CPU, memory usage
- **Lidar**: Real-time scan visualization (if lidar running)
- **Connection**: Green "Connected" status

### **✅ Controls Should Work:**
- **Joystick**: Moves robot when dragged
- **Stop Button**: Immediately stops robot
- **OLED Message**: Appears on physical display
- **Emergency Features**: Auto-stop when page closed

### **✅ RQT Graph Should Show:**
```bash
rqt_graph
```
- All nodes connected with proper topic flows
- ros_bridge connected to motor_control_node via /cmd_vel
- ros_bridge connected to battery topics

## 🚀 **Once Working: Create Simple Launch**

After verifying everything works individually, we can create a simpler launch file or use a script to start all components.

### **Simple Startup Script:**
```bash
#!/bin/bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Start all nodes in background
ros2 launch motor_control_pkg motor_control.launch.py &
sleep 2
ros2 launch battery_monitor_pkg battery_monitor.launch.py &
sleep 2
ros2 launch oled_display_pkg oled_display.launch.py &
sleep 2
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyACM1 &
sleep 3
ros2 launch web_interface_pkg web_interface.launch.py &

echo "🚀 All nodes started! Web interface: http://localhost:8080"
wait
```

This step-by-step approach will help identify exactly where the issue is and get the web interface working with real data! 🤖📊
