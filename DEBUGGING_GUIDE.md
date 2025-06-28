# JetRacer System Debugging Guide

## 🔍 **Current Status Analysis**

Based on testing, here's what's confirmed working:

### ✅ **Navigation Commands - WORKING**
- WebSocket messages are being sent from web interface
- ROS bridge is receiving and processing commands
- `/cmd_vel` topic is receiving proper Twist messages
- Movement commands confirmed: forward (0.3), backward (-0.3), turns (±0.5)

### ✅ **Data Transmission - WORKING**  
- WebSocket connections established
- Data broadcasting every 5 seconds
- Lidar data: 108 points being transmitted
- Battery data: Real voltage values (11.36V)

---

## 🚨 **Troubleshooting Steps**

### **1. Check Physical Robot Movement**

If robot is not moving despite commands being sent:

```bash
# Monitor motor control node output
ros2 topic echo /cmd_vel

# Check motor control node logs
ros2 node info /motor_control_node

# Test direct motor commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

**Possible Issues:**
- RP2040 not receiving commands properly
- Motor drivers not responding
- Power issues with drive system
- Firmware communication problems

### **2. Check Lidar Visualization**

If lidar map not showing in web interface:

**A. Verify Lidar Data:**
```bash
# Check lidar data is publishing
ros2 topic echo /scan --once

# Check lidar health
ros2 topic echo /lidar/health_status
```

**B. Check Web Interface Console:**
1. Open browser developer tools (F12)
2. Go to Console tab
3. Look for JavaScript errors
4. Check if lidar data is being received

**C. Check WebSocket Data:**
```bash
# Monitor ROS bridge logs for data transmission
# Look for "Broadcasting data" messages with lidar points
```

### **3. Browser Developer Console Checks**

Open browser console (F12) and check for:

**Navigation Issues:**
- Look for "Forward button clicked" messages
- Check "Sending WebSocket message" logs
- Verify WebSocket connection status

**Lidar Issues:**
- Check if robot_data messages are being received
- Look for lidar ranges array in data
- Check for canvas rendering errors

---

## 🔧 **Quick Fixes**

### **Fix 1: Restart Web Interface Only**
```bash
# If only web interface has issues
ros2 launch web_interface_pkg web_interface.launch.py
```

### **Fix 2: Reset Motor Control**
```bash
# If robot not moving
ros2 launch motor_control_pkg motor_control.launch.py
```

### **Fix 3: Check Hardware Connections**
```bash
# Verify devices
ls /dev/ttyACM*
# Should show: /dev/ttyACM0 (RP2040) and /dev/ttyACM1 (RPLidar)

# Check I2C devices
i2cdetect -y 7
# Should show devices at 0x3c (OLED) and 0x41 (INA219)
```

### **Fix 4: Browser Cache Clear**
- Clear browser cache and reload page
- Try incognito/private browsing mode
- Try different browser

---

## 📊 **Verification Commands**

### **Check All Nodes Running:**
```bash
ros2 node list
# Expected: motor_control_node, battery_monitor_node, oled_display_node, 
#          rplidar_node, web_server, ros_bridge
```

### **Check All Topics:**
```bash
ros2 topic list | grep -E "(cmd_vel|scan|battery)"
# Should show: /cmd_vel, /scan, /battery/voltage, /battery/percentage
```

### **Test Web APIs:**
```bash
# Test battery API
curl http://localhost:8080/api/battery_data

# Test system status
curl http://localhost:8080/api/system_status
```

---

## 🎯 **Expected Behavior**

### **Navigation:**
- Clicking buttons should show console messages
- `/cmd_vel` topic should receive Twist messages
- Robot should move physically (wheels/servo)

### **Lidar Visualization:**
- Canvas should show green dots representing obstacles
- Red dot in center representing robot
- Concentric circles showing range rings
- Real-time updates as robot moves

### **Battery Display:**
- Voltage: ~11.4V (real-time)
- Percentage: ~68% (calculated)
- Battery bar should reflect percentage
- Values should update every 2 seconds

---

## 🚀 **Next Steps**

1. **Run the debugging commands above**
2. **Check browser console for errors**
3. **Verify physical connections**
4. **Test individual components**

If issues persist, the problem is likely:
- **Hardware**: RP2040 communication or motor drivers
- **Browser**: JavaScript errors or WebSocket issues
- **Network**: Firewall blocking WebSocket connections
