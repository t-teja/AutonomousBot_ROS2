# Web Interface Testing Guide

## 🧪 **Testing the Web Interface Communication**

### **Step 1: Test ROS2 Communication**

#### **Terminal 1: Launch Test Node**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
python3 test_web_communication.py
```

#### **Terminal 2: Launch Web Interface**
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
ros2 launch web_interface_pkg web_interface.launch.py
```

#### **Terminal 3: Check RQT Graph**
```bash
rqt_graph
```

### **Step 2: Test Web Interface**

1. **Open Browser**: Go to `http://localhost:8080`
2. **Test Joystick**: Move the virtual joystick
3. **Check Terminal 1**: Should see cmd_vel messages
4. **Test OLED**: Send a message via the OLED input
5. **Check Terminal 1**: Should see OLED messages
6. **Monitor Data**: Battery and status should update in real-time

### **Expected Results**

#### **✅ RQT Graph Should Show:**
- `web_comm_test` node connected to `/cmd_vel` and `/oled_display_message`
- `ros_bridge` node connected to battery topics and `/cmd_vel`
- `web_server` node (may appear isolated - this is normal)

#### **✅ Terminal 1 Should Show:**
```
Received cmd_vel #1: linear.x=0.30, angular.z=0.50
Received OLED message #1: 'Hello Robot!'
```

#### **✅ Web Interface Should Show:**
- Battery voltage: 11.5-12.0V (changing)
- Battery percentage: 75-95% (changing)
- Robot status: "Test Mode - Commands: X, OLED: Y"
- Robot IP: 192.168.1.100

---

## 🔧 **Troubleshooting**

### **Issue 1: No cmd_vel Messages**
**Symptoms**: Joystick moves but no messages in terminal
**Solutions**:
1. Check WebSocket connection status in browser console (F12)
2. Verify ros_bridge_node is running: `ros2 node list | grep ros_bridge`
3. Check topic: `ros2 topic echo /cmd_vel`

### **Issue 2: No Battery Data in Web**
**Symptoms**: Battery shows "---" values
**Solutions**:
1. Check test node is publishing: `ros2 topic echo /battery/voltage`
2. Verify WebSocket connection in browser
3. Check browser console for errors

### **Issue 3: Joystick Not Centering**
**Symptoms**: Joystick knob doesn't return to center
**Solutions**:
1. Refresh the web page
2. Check browser console for JavaScript errors
3. Try on different device/browser

### **Issue 4: Laggy Interface**
**Symptoms**: Slow response to controls
**Solutions**:
1. Check network connection
2. Reduce browser tabs/applications
3. Try on different device
4. Check robot CPU usage

---

## 🚀 **Testing with Real Robot Nodes**

### **Replace Test Node with Real Nodes**

#### **Terminal 1: Motor Control**
```bash
ros2 launch motor_control_pkg motor_control.launch.py
```

#### **Terminal 2: Battery Monitor**
```bash
ros2 launch battery_monitor_pkg battery_monitor.launch.py
```

#### **Terminal 3: OLED Display**
```bash
ros2 launch oled_display_pkg oled_display.launch.py
```

#### **Terminal 4: Web Interface**
```bash
ros2 launch web_interface_pkg web_interface.launch.py
```

### **Real Robot Testing Checklist**

- [ ] **Motor Control**: Joystick moves robot
- [ ] **Battery Data**: Real voltage/percentage displayed
- [ ] **OLED Messages**: Messages appear on physical display
- [ ] **Robot Status**: Shows actual robot state
- [ ] **Safety**: Emergency stop works
- [ ] **Mobile**: Interface works on phone/tablet

---

## 📱 **Mobile/Tablet Testing**

### **Network Setup**
1. **Connect robot and device to same WiFi**
2. **Find robot IP**: `hostname -I`
3. **Access from device**: `http://<robot-ip>:8080`

### **Mobile Testing Checklist**
- [ ] **Responsive Design**: Interface adapts to screen size
- [ ] **Touch Controls**: Joystick works with touch
- [ ] **PWA Install**: "Add to Home Screen" option available
- [ ] **Performance**: Smooth operation on mobile
- [ ] **Safety**: Auto-stop when page closed

### **Performance Optimization**
- **Command Throttling**: 50ms between robot commands
- **UI Throttling**: 100ms between UI updates
- **WebSocket**: 10Hz data rate
- **Lidar Downsampling**: Every 10th point for visualization

---

## 🔍 **Debug Commands**

### **Check Node Status**
```bash
ros2 node list
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic echo /battery/voltage --once
```

### **Check Web Servers**
```bash
# Test HTTP server
curl http://localhost:8080/api/robot_info

# Check WebSocket (requires wscat)
# npm install -g wscat
# wscat -c ws://localhost:8765
```

### **Monitor Performance**
```bash
# Check CPU usage
htop

# Check network
netstat -tulpn | grep :8080
netstat -tulpn | grep :8765
```

---

## ✅ **Success Criteria**

The web interface is working correctly when:

1. **✅ ROS2 Communication**: Commands flow from web to robot
2. **✅ Real-time Data**: Battery/status updates in real-time
3. **✅ Responsive UI**: Smooth joystick and button interactions
4. **✅ Mobile Compatible**: Works well on phone/tablet
5. **✅ Safety Features**: Emergency stop and auto-stop work
6. **✅ Network Access**: Accessible from other devices on network

---

## 🎯 **Next Steps After Testing**

Once web interface is confirmed working:

1. **🗺️ SLAM Integration**: Add Cartographer for mapping
2. **📷 Camera Feed**: Integrate camera stream
3. **🎯 Navigation**: Add goal setting via web interface
4. **🏷️ AprilTags**: Add marker detection for docking
5. **📊 Advanced Monitoring**: Add more system metrics

---

**Happy Testing! 🤖📱**
