# JetRacer Full System Test Guide

## 🚀 **Quick Launch - All Real Nodes**

### **Single Command Launch (Recommended)**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch jetracer_full_system.launch.py
```

### **Alternative: Detailed Launch**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch web_interface_pkg jetracer_complete_system.launch.py
```

### **Launch Options**
```bash
# Without lidar
ros2 launch jetracer_full_system.launch.py use_lidar:=false

# With debug logging
ros2 launch jetracer_full_system.launch.py log_level:=debug

# With RViz (detailed launch only)
ros2 launch web_interface_pkg jetracer_complete_system.launch.py use_rviz:=true
```

---

## 📋 **What Gets Started**

### **✅ Real Robot Nodes:**
1. **motor_control_node** - RP2040 motor/servo control
2. **oled_display_node** - OLED display with IP address
3. **battery_monitor_node** - INA219 battery monitoring
4. **rplidar_node** - RPLidar A1 laser scanner
5. **web_server** - HTTP server for web interface
6. **ros_bridge** - WebSocket bridge for real-time data

### **🌐 Web Interface Access:**
- **Local**: http://localhost:8080
- **Network**: http://\<robot-ip\>:8080
- **Mobile**: Same URL on phone/tablet

---

## 🔍 **Expected Results**

### **✅ In Terminal:**
```
🤖 Starting JetRacer Full System...
📋 Components: Motor Control + OLED + Battery + Lidar + Web Interface
🌐 Web Access: http://localhost:8080

[INFO] [motor_control_node]: Motor control initialized
[INFO] [oled_display_node]: OLED display ready
[INFO] [battery_monitor_node]: Battery monitor started
[INFO] [rplidar_node]: RPLidar A1 connected
[INFO] [web_server]: Web server started on port 8080
[INFO] [ros_bridge]: WebSocket server started on port 8765

✅ JetRacer System Ready!
🌐 Web Interface: http://localhost:8080
📱 Mobile Access: http://<robot-ip>:8080
🎮 Use web interface to control robot
```

### **✅ In Web Interface:**
- **Battery Section**: Real voltage (11-12V), percentage, current, power
- **Robot Status**: Shows actual robot IP and system status
- **Lidar Visualization**: Real-time laser scan points
- **System Info**: CPU usage, memory, temperature
- **Controls**: Joystick responds and sends commands

### **✅ In RQT Graph:**
```bash
# In another terminal
rqt_graph
```
Should show all nodes connected with proper topic flows.

---

## 🧪 **Testing Checklist**

### **Hardware Verification:**
- [ ] **RP2040**: Connected on /dev/ttyACM0 (motor control)
- [ ] **RPLidar**: Connected on /dev/ttyACM1 (laser scanner)
- [ ] **OLED**: Working on I2C bus 7 (shows IP address)
- [ ] **INA219**: Working on I2C bus 7 (battery monitoring)

### **Web Interface Testing:**
- [ ] **Access**: Can open http://localhost:8080
- [ ] **Connection**: Green "Connected" status in header
- [ ] **Battery Data**: Shows real voltage/percentage values
- [ ] **Robot IP**: Displays actual robot IP address
- [ ] **Lidar Data**: Shows moving laser scan points
- [ ] **Joystick Control**: Moves robot when used
- [ ] **Emergency Stop**: Stops robot immediately
- [ ] **OLED Messages**: Appear on physical OLED display

### **Mobile Testing:**
- [ ] **Network Access**: Works from phone/tablet on same WiFi
- [ ] **Touch Controls**: Joystick works with touch
- [ ] **Responsive Design**: Interface adapts to screen size
- [ ] **Performance**: Smooth operation on mobile device

---

## 🔧 **Troubleshooting**

### **Issue: No Web Data**
**Symptoms**: Web interface shows "---" for all values
**Solutions**:
1. Check WebSocket connection (should show "Connected")
2. Verify nodes are running: `ros2 node list`
3. Check topics: `ros2 topic list`
4. Restart launch file

### **Issue: Motor Control Not Working**
**Symptoms**: Joystick doesn't move robot
**Solutions**:
1. Check RP2040 connection: `ls /dev/ttyACM*`
2. Verify motor node: `ros2 topic echo /cmd_vel`
3. Check hardware: Encoder LEDs should be ON
4. Test manually: `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'`

### **Issue: No Lidar Data**
**Symptoms**: Lidar visualization shows "No Data"
**Solutions**:
1. Check RPLidar connection: `ls /dev/ttyACM*`
2. Verify lidar node: `ros2 topic echo /scan --once`
3. Check hardware: RPLidar should be spinning
4. Try manual launch: `ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyACM1`

### **Issue: No Battery Data**
**Symptoms**: Battery shows "---V" and "---%" 
**Solutions**:
1. Check I2C connection: `i2cdetect -y 7`
2. Should see device at 0x41 (INA219)
3. Verify battery node: `ros2 topic echo /battery/voltage`
4. Check hardware connections

### **Issue: Web Interface Won't Load**
**Symptoms**: Browser can't connect to http://localhost:8080
**Solutions**:
1. Check web server is running: `netstat -tulpn | grep :8080`
2. Try different port: Add `web_port:=8081` to launch command
3. Check firewall settings
4. Try from different device on network

---

## 📊 **Performance Monitoring**

### **Check System Resources:**
```bash
# CPU and memory usage
htop

# Network connections
netstat -tulpn | grep -E ":(8080|8765)"

# ROS2 node status
ros2 node list
ros2 topic hz /cmd_vel
ros2 topic hz /scan
```

### **Expected Performance:**
- **Web Interface**: Responsive, <100ms latency
- **Lidar**: ~8Hz scan rate
- **Battery**: 1Hz update rate
- **Motor Commands**: Real-time response
- **CPU Usage**: <50% on Jetson Orin Nano

---

## 🎯 **Success Criteria**

The system is working correctly when:

1. **✅ All Nodes Running**: 6 nodes active in `ros2 node list`
2. **✅ Web Interface Loads**: Accessible at http://localhost:8080
3. **✅ Real Data Flowing**: Battery, lidar, status show real values
4. **✅ Robot Control Works**: Joystick moves robot
5. **✅ Mobile Compatible**: Works on phone/tablet
6. **✅ Safety Features**: Emergency stop works

---

## 🎉 **Next Steps After Success**

Once everything is working:

1. **🗺️ Add SLAM**: Integrate Cartographer for mapping
2. **📷 Camera Stream**: Add camera feed to web interface  
3. **🎯 Navigation**: Add goal setting and path planning
4. **🏷️ AprilTags**: Add marker detection for docking
5. **📊 Advanced Features**: More sensors and capabilities

---

**Ready to test the complete system! 🤖🚀**
