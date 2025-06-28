# RPLidar A1 Troubleshooting Guide - JetRacer

**🎯 CRITICAL INFORMATION: This document contains solutions for common RPLidar A1 issues on JetRacer with Jetson Orin Nano.**

## ⚠️ CRITICAL ISSUE: Web Interface Lidar Data Not Displaying

### Issue Description
- Web interface shows "Waiting for lidar data..."
- Lidar Data Log shows JSON parse errors
- ROS system shows lidar working correctly
- Error: `Unexpected token 'N', ...""charge": NaN, "perc"... is not valid JSON`

### Root Cause
**INA219 battery monitor sends `NaN` values** which break JSON serialization in the web interface. JSON doesn't support `NaN` values, causing the entire message to fail parsing.

### Solution (CONFIRMED WORKING)
Add data cleaning function in `ros_bridge_node.py`:

```python
def clean_data_for_json(self, data):
    """Clean data to ensure JSON serialization compatibility"""
    import math
    
    if isinstance(data, dict):
        cleaned = {}
        for key, value in data.items():
            cleaned[key] = self.clean_data_for_json(value)
        return cleaned
    elif isinstance(data, list):
        return [self.clean_data_for_json(item) for item in data]
    elif isinstance(data, float):
        if math.isnan(data) or math.isinf(data):
            return None  # Convert NaN/Inf to None (null in JSON)
        return data
    else:
        return data

def broadcast_data(self):
    # Clean data before JSON serialization
    cleaned_data = self.clean_data_for_json(self.latest_data)
    data_package = {
        'type': 'robot_data',
        'data': cleaned_data,
        'timestamp': time.time()
    }
```

### Verification Steps
1. **Check ROS logs**: `ros2 topic echo /scan` should show lidar data
2. **Check web interface log**: Should show real-time lidar scan data
3. **No JSON errors**: Browser console should be clean
4. **Data flow**: Log should show: `📡 Scan: 108 points | Valid: 108 | Range: X.XX-XX.XXm`

## 🔧 Hardware Troubleshooting

### RPLidar A1 Connection Issues
- **Device Path**: `/dev/ttyACM1` (confirmed working)
- **Permissions**: Ensure user is in `dialout` group
- **Power**: RPLidar should spin and emit red laser
- **USB Cable**: Use high-quality USB cable, avoid extensions

### RViz Visualization Issues
- **Fixed Frame**: Set to `laser` (not `map`)
- **Topic**: `/scan` for laser scan data
- **Display Type**: LaserScan display
- **TF**: Ensure `base_link` to `laser` transform is published

## 📊 Diagnostic Commands

### Check Lidar Hardware
```bash
# Check device detection
ls -la /dev/ttyACM*

# Check permissions
groups $USER

# Test lidar node
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyACM1
```

### Check Data Flow
```bash
# Check scan topic
ros2 topic echo /scan

# Check transform
ros2 run tf2_tools view_frames

# Check node status
ros2 node list
ros2 node info /rplidar_node
```

### Web Interface Debugging
```bash
# Check WebSocket connection
netstat -an | grep 8765

# Check web server
curl http://localhost:8080

# Check browser console for errors
# Open Developer Tools > Console
```

## 🚀 Prevention Guidelines

### Code Best Practices
1. **Always validate sensor data** before JSON serialization
2. **Use `math.isnan()` and `math.isinf()`** checks for float values
3. **Implement data cleaning** in all web interface bridges
4. **Add comprehensive logging** for debugging

### System Monitoring
1. **Monitor battery sensor** for NaN values
2. **Check lidar data quality** regularly
3. **Verify WebSocket connections** are stable
4. **Test web interface** after system updates

## 📝 Status Log

- **2025-06-29**: ✅ JSON parse error resolved with data cleaning function
- **2025-06-29**: ✅ Lidar data flowing correctly to web interface
- **2025-06-29**: ✅ Enhanced debugging with Lidar Data Log Window

## 🔗 Related Documents

- `RP2040_MOTOR_CONTROL_PROTOCOL.md` - Motor control troubleshooting
- `ROS_TOPICS_DOCUMENTATION.md` - System communication overview
- Web interface source: `src/web_interface_pkg/`
