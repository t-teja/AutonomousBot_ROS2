/**
 * JetRacer Robot Web Interface
 * Modern JavaScript application for robot control and monitoring
 */

class JetRacerInterface {
    constructor() {
        this.websocket = null;
        this.isConnected = false;
        this.joystickActive = false;
        this.joystickCenter = { x: 0, y: 0 };
        this.maxJoystickDistance = 55; // pixels from center

        // Performance optimization
        this.lastCommandTime = 0;
        this.commandThrottle = 50; // ms between commands
        this.lastUpdateTime = 0;
        this.updateThrottle = 100; // ms between UI updates

        // Robot state
        this.robotData = {
            battery: {},
            lidar: {},
            odometry: {},
            system: {}
        };

        // Lidar log state
        this.logPaused = false;
        this.logAutoScroll = true;
        this.logMaxLines = 100;

        this.init();
    }
    
    init() {
        this.setupWebSocket();
        this.setupEventListeners();
        this.setupJoystick();
        this.setupLidarLog();
        this.startDataRefresh();
        this.hideLoadingOverlay();
    }
    
    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8765`;

        console.log('🔌 Setting up WebSocket connection to:', wsUrl);

        try {
            this.websocket = new WebSocket(wsUrl);

            this.websocket.onopen = () => {
                console.log('✅ WebSocket connected successfully at:', new Date().toISOString());
                this.connectionAttempts = 0; // Reset connection attempts
                this.setConnectionStatus(true);
            };

            this.websocket.onmessage = (event) => {
                this.handleWebSocketMessage(event.data);
            };

            this.websocket.onclose = (event) => {
                console.log('❌ WebSocket disconnected. Code:', event.code, 'Reason:', event.reason, 'Time:', new Date().toISOString());
                this.setConnectionStatus(false);

                // Implement exponential backoff for reconnection
                this.connectionAttempts = (this.connectionAttempts || 0) + 1;
                const delay = Math.min(1000 * Math.pow(2, this.connectionAttempts), 10000); // Max 10 seconds
                console.log(`🔄 Attempting to reconnect in ${delay}ms (attempt ${this.connectionAttempts})`);

                setTimeout(() => this.setupWebSocket(), delay);
            };

            this.websocket.onerror = (error) => {
                console.error('🚨 WebSocket error:', error);
                this.setConnectionStatus(false);
            };

        } catch (error) {
            console.error('❌ Failed to create WebSocket:', error);
            this.setConnectionStatus(false);
        }
    }
    
    handleWebSocketMessage(data) {
        try {
            // First, log that we received ANY message
            console.log('🔄 RAW WebSocket message received at:', new Date().toISOString());
            console.log('📦 Raw message length:', data.length);
            console.log('📦 Raw message preview:', data.substring(0, 200) + '...');

            this.addToLidarLog('🔄 WebSocket message received', '#00aaff');

            const message = JSON.parse(data);
            console.log('✅ Message parsed successfully');
            console.log('📋 Message type:', message.type);

            this.addToLidarLog(`📋 Message type: ${message.type}`, '#00aaff');

            if (message.type === 'robot_data') {
                console.log('🔄 Processing robot_data message at:', new Date().toISOString());
                console.log('📊 Full message data keys:', Object.keys(message.data));
                console.log('📡 Lidar data in message:', message.data.lidar);

                this.addToLidarLog(`📊 Data keys: ${Object.keys(message.data).join(', ')}`, '#00aaff');

                // Check if lidar data exists and has ranges
                if (message.data.lidar && message.data.lidar.ranges) {
                    console.log('✅ LIDAR DATA FOUND:', message.data.lidar.ranges.length, 'points');
                    console.log('📍 First 3 ranges:', message.data.lidar.ranges.slice(0, 3));

                    // Add to lidar log
                    const lidar = message.data.lidar;
                    const validRanges = lidar.ranges.filter(r => r !== null && r !== undefined && !isNaN(r));
                    const minRange = validRanges.length > 0 ? Math.min(...validRanges).toFixed(2) : 'N/A';
                    const maxRange = validRanges.length > 0 ? Math.max(...validRanges).toFixed(2) : 'N/A';
                    const avgRange = validRanges.length > 0 ? (validRanges.reduce((a, b) => a + b, 0) / validRanges.length).toFixed(2) : 'N/A';

                    this.addToLidarLog(
                        `📡 Scan: ${lidar.ranges.length} points | Valid: ${validRanges.length} | Range: ${minRange}-${maxRange}m | Avg: ${avgRange}m`,
                        '#00ff00'
                    );

                    // Log first few ranges for debugging
                    const firstRanges = lidar.ranges.slice(0, 5).map(r => r !== null ? r.toFixed(2) : 'null').join(', ');
                    this.addToLidarLog(`📍 First 5 ranges: [${firstRanges}]`, '#00aaff');

                } else {
                    console.log('❌ NO LIDAR RANGES FOUND');
                    console.log('🔍 Lidar object keys:', message.data.lidar ? Object.keys(message.data.lidar) : 'no lidar object');

                    this.addToLidarLog('❌ No lidar ranges found in message', '#ff4444');
                    if (message.data.lidar) {
                        this.addToLidarLog(`🔍 Lidar keys: ${Object.keys(message.data.lidar).join(', ')}`, '#ffaa00');
                    } else {
                        this.addToLidarLog('🔍 No lidar object in message', '#ffaa00');
                    }
                }

                this.robotData = message.data;
                this.updateUI();
            } else {
                this.addToLidarLog(`❓ Unknown message type: ${message.type}`, '#ffaa00');
            }
        } catch (error) {
            console.error('❌ Error parsing WebSocket message:', error);
            this.addToLidarLog(`❌ Parse error: ${error.message}`, '#ff4444');
        }
    }
    
    sendWebSocketMessage(message) {
        console.log('Sending WebSocket message:', message);
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(message));
            console.log('Message sent successfully');
        } else {
            console.warn('WebSocket not connected, readyState:', this.websocket ? this.websocket.readyState : 'null');
        }
    }
    
    setConnectionStatus(connected) {
        this.isConnected = connected;
        const statusElement = document.getElementById('connectionStatus');
        const statusDot = statusElement.querySelector('.status-dot');
        const statusText = statusElement.querySelector('.status-text');
        
        if (connected) {
            statusDot.className = 'status-dot online';
            statusText.textContent = 'Connected';
        } else {
            statusDot.className = 'status-dot offline';
            statusText.textContent = 'Disconnected';
        }
    }
    
    setupEventListeners() {
        // Stop button
        document.getElementById('stopBtn').addEventListener('click', () => {
            this.stopRobot();
        });
        
        // Direction buttons
        document.getElementById('forwardBtn').addEventListener('click', () => {
            console.log('Forward button clicked');
            this.moveRobot(0.3, 0);
        });

        document.getElementById('backwardBtn').addEventListener('click', () => {
            console.log('Backward button clicked');
            this.moveRobot(-0.3, 0);
        });

        document.getElementById('leftBtn').addEventListener('click', () => {
            console.log('Left button clicked');
            this.moveRobot(0, 0.5);
        });

        document.getElementById('rightBtn').addEventListener('click', () => {
            console.log('Right button clicked');
            this.moveRobot(0, -0.5);
        });
        
        // Lidar controls
        document.getElementById('startLidarBtn').addEventListener('click', () => {
            this.sendWebSocketMessage({ command: 'start_lidar' });
        });
        
        document.getElementById('stopLidarBtn').addEventListener('click', () => {
            this.sendWebSocketMessage({ command: 'stop_lidar' });
        });
        
        // OLED message
        document.getElementById('sendOledBtn').addEventListener('click', () => {
            const message = document.getElementById('oledMessage').value;
            if (message.trim()) {
                this.sendWebSocketMessage({
                    command: 'send_oled_message',
                    message: message
                });
                document.getElementById('oledMessage').value = '';
            }
        });
        
        // Enter key for OLED message
        document.getElementById('oledMessage').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                document.getElementById('sendOledBtn').click();
            }
        });
    }

    setupLidarLog() {
        // Clear log button
        document.getElementById('clearLogBtn').addEventListener('click', () => {
            const logElement = document.getElementById('lidarDataLog');
            logElement.innerHTML = '<div style="color: #888;">Log cleared...</div>';
        });

        // Pause/Resume log button
        const pauseBtn = document.getElementById('pauseLogBtn');
        pauseBtn.addEventListener('click', () => {
            this.logPaused = !this.logPaused;
            pauseBtn.textContent = this.logPaused ? 'Resume' : 'Pause';
            pauseBtn.style.backgroundColor = this.logPaused ? '#4caf50' : '#ff9800';

            if (!this.logPaused) {
                this.addToLidarLog('📝 Log resumed', '#00ff00');
            } else {
                this.addToLidarLog('⏸️ Log paused', '#ffaa00');
            }
        });
    }

    addToLidarLog(message, color = '#00ff00') {
        if (this.logPaused) return;

        const logElement = document.getElementById('lidarDataLog');
        const timestamp = new Date().toLocaleTimeString();

        const logEntry = document.createElement('div');
        logEntry.style.color = color;
        logEntry.style.marginBottom = '2px';
        logEntry.innerHTML = `<span style="color: #666;">[${timestamp}]</span> ${message}`;

        logElement.appendChild(logEntry);

        // Limit log lines
        const lines = logElement.children;
        if (lines.length > this.logMaxLines) {
            logElement.removeChild(lines[0]);
        }

        // Auto-scroll to bottom
        if (this.logAutoScroll) {
            logElement.scrollTop = logElement.scrollHeight;
        }
    }

    setupJoystick() {
        const joystick = document.getElementById('joystick');
        const knob = document.getElementById('joystickKnob');
        
        // Get joystick center position
        const rect = joystick.getBoundingClientRect();
        this.joystickCenter = {
            x: rect.width / 2,
            y: rect.height / 2
        };
        
        // Mouse events
        knob.addEventListener('mousedown', this.startJoystickDrag.bind(this));
        document.addEventListener('mousemove', this.updateJoystickDrag.bind(this));
        document.addEventListener('mouseup', this.endJoystickDrag.bind(this));
        
        // Touch events for mobile
        knob.addEventListener('touchstart', this.startJoystickDrag.bind(this), { passive: false });
        document.addEventListener('touchmove', this.updateJoystickDrag.bind(this), { passive: false });
        document.addEventListener('touchend', this.endJoystickDrag.bind(this));
        
        // Click on joystick area
        joystick.addEventListener('click', this.clickJoystick.bind(this));
    }
    
    startJoystickDrag(event) {
        event.preventDefault();
        this.joystickActive = true;
        document.body.style.userSelect = 'none';
    }
    
    updateJoystickDrag(event) {
        if (!this.joystickActive) return;
        
        event.preventDefault();
        
        const joystick = document.getElementById('joystick');
        const knob = document.getElementById('joystickKnob');
        const rect = joystick.getBoundingClientRect();
        
        // Get mouse/touch position
        let clientX, clientY;
        if (event.type.startsWith('touch')) {
            clientX = event.touches[0].clientX;
            clientY = event.touches[0].clientY;
        } else {
            clientX = event.clientX;
            clientY = event.clientY;
        }
        
        // Calculate position relative to joystick center
        const x = clientX - rect.left - this.joystickCenter.x;
        const y = clientY - rect.top - this.joystickCenter.y;
        
        // Limit to circle
        const distance = Math.sqrt(x * x + y * y);
        let finalX = x;
        let finalY = y;
        
        if (distance > this.maxJoystickDistance) {
            finalX = (x / distance) * this.maxJoystickDistance;
            finalY = (y / distance) * this.maxJoystickDistance;
        }
        
        // Update knob position
        knob.style.transform = `translate(${finalX}px, ${finalY}px)`;
        
        // Calculate normalized values (-1 to 1)
        const normalizedX = finalX / this.maxJoystickDistance;
        const normalizedY = -finalY / this.maxJoystickDistance; // Invert Y axis
        
        // Update display
        document.getElementById('joystickX').textContent = normalizedX.toFixed(2);
        document.getElementById('joystickY').textContent = normalizedY.toFixed(2);
        
        // Send robot movement command
        this.moveRobot(normalizedY * 0.5, normalizedX * 1.0); // Scale for robot speeds
    }
    
    endJoystickDrag(event) {
        if (!this.joystickActive) return;

        this.joystickActive = false;
        document.body.style.userSelect = '';

        // Return knob to center with smooth animation
        const knob = document.getElementById('joystickKnob');
        knob.style.transition = 'transform 0.2s ease-out';
        knob.style.transform = 'translate(0px, 0px)';

        // Remove transition after animation
        setTimeout(() => {
            knob.style.transition = '';
        }, 200);

        // Reset display
        document.getElementById('joystickX').textContent = '0.00';
        document.getElementById('joystickY').textContent = '0.00';

        // Stop robot
        this.stopRobot();
    }
    
    clickJoystick(event) {
        if (this.joystickActive) return;
        
        const joystick = document.getElementById('joystick');
        const knob = document.getElementById('joystickKnob');
        const rect = joystick.getBoundingClientRect();
        
        // Calculate click position
        const x = event.clientX - rect.left - this.joystickCenter.x;
        const y = event.clientY - rect.top - this.joystickCenter.y;
        
        // Limit to circle
        const distance = Math.sqrt(x * x + y * y);
        let finalX = x;
        let finalY = y;
        
        if (distance > this.maxJoystickDistance) {
            finalX = (x / distance) * this.maxJoystickDistance;
            finalY = (y / distance) * this.maxJoystickDistance;
        }
        
        // Move knob
        knob.style.transform = `translate(${finalX}px, ${finalY}px)`;
        
        // Calculate normalized values
        const normalizedX = finalX / this.maxJoystickDistance;
        const normalizedY = -finalY / this.maxJoystickDistance;
        
        // Update display
        document.getElementById('joystickX').textContent = normalizedX.toFixed(2);
        document.getElementById('joystickY').textContent = normalizedY.toFixed(2);
        
        // Send movement command briefly
        this.moveRobot(normalizedY * 0.5, normalizedX * 1.0);
        
        // Return to center after 300ms with smooth animation
        setTimeout(() => {
            knob.style.transition = 'transform 0.2s ease-out';
            knob.style.transform = 'translate(0px, 0px)';
            document.getElementById('joystickX').textContent = '0.00';
            document.getElementById('joystickY').textContent = '0.00';
            this.stopRobot();

            // Remove transition after animation
            setTimeout(() => {
                knob.style.transition = '';
            }, 200);
        }, 300);
    }
    
    moveRobot(linearX, angularZ) {
        // Throttle commands to prevent overwhelming the robot
        const now = Date.now();
        if (now - this.lastCommandTime < this.commandThrottle) {
            return;
        }
        this.lastCommandTime = now;

        this.sendWebSocketMessage({
            command: 'move_robot',
            linear_x: linearX,
            linear_y: 0.0,
            angular_z: angularZ
        });
    }
    
    stopRobot() {
        this.sendWebSocketMessage({
            command: 'stop_robot'
        });
    }
    
    updateUI() {
        // Throttle UI updates for better performance
        const now = Date.now();
        if (now - this.lastUpdateTime < this.updateThrottle) {
            return;
        }
        this.lastUpdateTime = now;

        this.updateBatteryInfo();
        this.updateSystemInfo();
        this.updateLidarInfo();
        this.updateLidarVisualization();
    }
    
    updateBatteryInfo() {
        const battery = this.robotData.battery || {};
        
        // Battery percentage
        const percentage = battery.percentage_raw || battery.jetracer_percentage || 0;
        document.getElementById('batteryLevel').textContent = `${percentage.toFixed(1)}%`;
        
        // Battery bar
        const batteryFill = document.getElementById('batteryFill');
        batteryFill.style.width = `${Math.max(0, Math.min(100, percentage))}%`;
        
        // Voltage
        const voltage = battery.voltage_raw || battery.jetracer_voltage || 0;
        document.getElementById('batteryVoltage').textContent = `${voltage.toFixed(2)}V`;
        
        // Current
        const current = battery.jetracer_current || 0;
        document.getElementById('batteryCurrent').textContent = `${current.toFixed(2)}A`;
        
        // Power
        const power = battery.jetracer_power || (voltage * current);
        document.getElementById('batteryPower').textContent = `${power.toFixed(1)}W`;
    }
    
    updateSystemInfo() {
        const robotStatus = this.robotData.robot_status || {};
        
        // Robot IP
        document.getElementById('robotIP').textContent = robotStatus.ip_address || '---';
        
        // System status
        document.getElementById('systemStatus').textContent = robotStatus.status || 'Unknown';
    }
    
    updateLidarInfo() {
        const lidar = this.robotData.lidar || {};

        // Only update lidar status (which still exists in the controls section)
        const health = lidar.health ? 'Healthy' : 'Offline';
        const statusElement = document.getElementById('lidarStatus');
        if (statusElement) {
            statusElement.textContent = health;
        }
    }
    
    updateLidarVisualization() {
        const canvas = document.getElementById('lidarCanvas');
        if (!canvas) {
            console.error('❌ Lidar canvas not found!');
            return;
        }

        const ctx = canvas.getContext('2d');
        if (!ctx) {
            console.error('❌ Canvas context not available!');
            return;
        }

        const lidar = this.robotData.lidar || {};

        // Clear canvas first
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (!lidar.ranges || lidar.ranges.length === 0) {
            // Draw "No Data" message with debugging info
            ctx.fillStyle = '#6c757d';
            ctx.font = '14px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('No Lidar Data', canvas.width / 2, canvas.height / 2 - 40);

            // Show debugging info
            ctx.font = '10px Arial';
            ctx.fillText(`Robot data keys: ${Object.keys(this.robotData).join(', ')}`, canvas.width / 2, canvas.height / 2 - 20);
            if (this.robotData.lidar) {
                ctx.fillText(`Lidar keys: ${Object.keys(this.robotData.lidar).join(', ')}`, canvas.width / 2, canvas.height / 2);
                ctx.fillText(`Ranges: ${this.robotData.lidar.ranges ? this.robotData.lidar.ranges.length : 'undefined'}`, canvas.width / 2, canvas.height / 2 + 20);
            } else {
                ctx.fillText('No lidar object in robot data', canvas.width / 2, canvas.height / 2);
            }

            // Also update the page title to show debug info
            document.title = `JetRacer - No Lidar Data (Keys: ${Object.keys(this.robotData).join(',')})`;
            console.log('No lidar data available');
            return;
        } else {
            // Update page title to show we have data
            document.title = `JetRacer - Lidar OK (${lidar.ranges.length} points)`;
        }
        
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(canvas.width, canvas.height) / 2 - 30; // Increased margin for larger canvas
        const scale = maxRadius / 12; // 12m max range with better scaling

        // Draw range circles - more rings for better scale reference
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        for (let r = 1; r <= 12; r += 1) {
            const radius = r * scale;
            ctx.beginPath();
            ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
            ctx.stroke();

            // Add range labels for better reference
            if (r % 2 === 0) {
                ctx.fillStyle = '#666';
                ctx.font = '10px Arial';
                ctx.textAlign = 'center';
                ctx.fillText(`${r}m`, centerX + radius - 15, centerY - 5);
            }
        }

        // Draw cross lines for orientation
        ctx.strokeStyle = '#444';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(centerX - maxRadius, centerY);
        ctx.lineTo(centerX + maxRadius, centerY);
        ctx.moveTo(centerX, centerY - maxRadius);
        ctx.lineTo(centerX, centerY + maxRadius);
        ctx.stroke();

        // Draw robot (larger for better visibility)
        ctx.fillStyle = '#e94560';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 6, 0, 2 * Math.PI);
        ctx.fill();

        // Draw lidar points (larger for better visibility)
        ctx.fillStyle = '#27ae60';
        const ranges = lidar.ranges;
        const angleMin = lidar.angle_min || 0;
        const angleIncrement = lidar.angle_increment || (2 * Math.PI / ranges.length);

        let pointsDrawn = 0;
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];
            if (range !== null && range !== undefined && range > 0 && range < 12) { // Valid range
                const angle = angleMin + i * angleIncrement;
                const x = centerX + range * scale * Math.cos(angle);
                const y = centerY + range * scale * Math.sin(angle);

                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI); // Larger points for better visibility
                ctx.fill();
                pointsDrawn++;
            }
        }
    }
    
    startDataRefresh() {
        // Refresh system info every 5 seconds
        setInterval(() => {
            this.fetchSystemStatus();
        }, 5000);

        // Refresh battery data every 2 seconds
        setInterval(() => {
            this.fetchBatteryData();
        }, 2000);
    }

    async fetchBatteryData() {
        // Fetch battery data from API
        try {
            const response = await fetch('/api/battery_data');
            const data = await response.json();

            if (data.status === 'active') {
                // Update battery display
                document.getElementById('batteryLevel').textContent = `${data.percentage.toFixed(1)}%`;
                document.getElementById('batteryVoltage').textContent = `${data.voltage.toFixed(2)}V`;

                // Update battery bar
                const batteryFill = document.getElementById('batteryFill');
                batteryFill.style.width = `${Math.max(0, Math.min(100, data.percentage))}%`;

                // Update connection status
                this.setConnectionStatus(true);
            } else {
                console.warn('Battery data not available:', data);
            }

        } catch (error) {
            console.error('Failed to fetch battery data:', error);
        }
    }
    
    async fetchSystemStatus() {
        try {
            const response = await fetch('/api/system_status');
            const data = await response.json();
            
            if (data.cpu_percent !== undefined) {
                document.getElementById('cpuUsage').textContent = `${data.cpu_percent.toFixed(1)}%`;
                document.getElementById('cpuProgress').style.width = `${data.cpu_percent}%`;
            }
            
            if (data.memory && data.memory.percent !== undefined) {
                document.getElementById('memoryUsage').textContent = `${data.memory.percent.toFixed(1)}%`;
                document.getElementById('memoryProgress').style.width = `${data.memory.percent}%`;
            }
            
            if (data.temperature !== undefined && data.temperature !== null) {
                document.getElementById('systemTemp').textContent = `${data.temperature.toFixed(1)}°C`;
            }
            
            if (data.uptime !== undefined) {
                const hours = Math.floor(data.uptime / 3600);
                const minutes = Math.floor((data.uptime % 3600) / 60);
                document.getElementById('systemUptime').textContent = `${hours}h ${minutes}m`;
            }
            
        } catch (error) {
            console.error('Failed to fetch system status:', error);
        }
    }
    
    hideLoadingOverlay() {
        setTimeout(() => {
            const overlay = document.getElementById('loadingOverlay');
            overlay.style.opacity = '0';
            setTimeout(() => {
                overlay.style.display = 'none';
            }, 300);
        }, 1000);
    }
}

// Initialize the application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.jetracerInterface = new JetRacerInterface();
});

// Handle page visibility changes
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        // Page is hidden, stop robot for safety
        if (window.jetracerInterface) {
            window.jetracerInterface.stopRobot();
        }
    }
});

// Handle page unload
window.addEventListener('beforeunload', () => {
    if (window.jetracerInterface) {
        window.jetracerInterface.stopRobot();
    }
});
