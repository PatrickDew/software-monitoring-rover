import React, { useState, useEffect, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, AreaChart, Area } from 'recharts';
import { Activity, Compass, Cpu, Radio, Camera, Navigation, Terminal, Gauge, Satellite, Zap, TrendingUp, AlertTriangle } from 'lucide-react';

const RoverMissionControl = () => {
  const [activeMode, setActiveMode] = useState('telemetry');
  const [connected, setConnected] = useState(false);
  const [command, setCommand] = useState('');
  const [commandHistory, setCommandHistory] = useState([]);
  const [imuData, setImuData] = useState([]);
  const [gpsData, setGpsData] = useState({ lat: 38.4063, lon: -110.7918, alt: 1372 }); // Utah coordinates
  const [lidarData, setLidarData] = useState([]);
  const [systemHealth, setSystemHealth] = useState({
    battery: 87,
    cpu: 45,
    memory: 62,
    temperature: 38,
    signal: 92
  });
  const [missionMode, setMissionMode] = useState('object'); // New state for mission mode
  const terminalRef = useRef(null);
  
  // --- PLACEHOLDER DATA FOR SENSOR VISUALIZATIONS ---

  // Placeholder for 2D LiDAR Point Cloud
  // Simulates 36 points in a circle with varying distances
  const lidarPoints = Array.from({ length: 36 }).map((_, i) => {
    const angle = (i * 10) * (Math.PI / 180); // 10 degrees increment
    const baseDistance = 100 + Math.sin(angle * 5 + Date.now() / 1000) * 50;
    const distance = Math.min(200, Math.max(50, baseDistance + Math.random() * 20));
    return {
      x: distance * Math.cos(angle),
      y: distance * Math.sin(angle),
      distance: distance,
    };
  });

  // Placeholder for Occupancy Grid Map (40x40 grid)
  const occupancyGrid = Array.from({ length: 40 }, (_, y) => 
    Array.from({ length: 40 }, (_, x) => {
      // Simulate an obstacle 'wall'
      if (y > 10 && y < 30 && x === 25) return 0.8;
      // Simulate another, smaller obstacle
      if (Math.sqrt((x-30)**2 + (y-10)**2) < 3) return 0.9;
      return 0;
    })
  );

  // --- END PLACEHOLDER DATA ---

  // Simulate real-time data updates
  useEffect(() => {
    const interval = setInterval(() => {
      // IMU data simulation
      setImuData(prev => {
        const newData = [...prev, {
          time: new Date().toLocaleTimeString(),
          pitch: Math.sin(Date.now() / 1000) * 15 + Math.random() * 5,
          roll: Math.cos(Date.now() / 1000) * 10 + Math.random() * 3,
          yaw: Math.sin(Date.now() / 2000) * 20 + Math.random() * 4,
          accelX: Math.random() * 2 - 1,
          accelY: Math.random() * 2 - 1,
          accelZ: 9.8 + Math.random() * 0.5
        }];
        return newData.slice(-20);
      });

      // LiDAR data simulation
      setLidarData(prev => {
        const newData = [...prev, {
          time: new Date().toLocaleTimeString(),
          distance: 150 + Math.random() * 100,
          points: Math.floor(50000 + Math.random() * 10000)
        }];
        return newData.slice(-20);
      });

      // System health updates
      setSystemHealth(prev => ({
        battery: Math.max(0, prev.battery - Math.random() * 0.1),
        cpu: 30 + Math.random() * 40,
        memory: 50 + Math.random() * 30,
        temperature: 35 + Math.random() * 10,
        signal: 80 + Math.random() * 20
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const sendCommand = () => {
    if (command.trim()) {
      const timestamp = new Date().toLocaleTimeString();
      setCommandHistory(prev => [...prev, { cmd: command, time: timestamp, status: 'sent' }]);
      
      // Simulate command execution
      setTimeout(() => {
        setCommandHistory(prev => prev.map((item, idx) => 
          idx === prev.length - 1 ? { ...item, status: 'executed', response: 'OK' } : item
        ));
      }, 500);
      
      setCommand('');
      
      if (terminalRef.current) {
        terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
      }
    }
  };

  const handleMissionStagePublish = () => {
    const timestamp = new Date().toLocaleTimeString();
    setCommandHistory(prev => [...prev, { cmd: `ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: ${gpsData.lat}, y: ${gpsData.lon}, z: ${gpsData.alt}}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}'`, time: timestamp, status: 'sent' }]);
    setTimeout(() => {
      setCommandHistory(prev => prev.map((item, idx) => 
        idx === prev.length - 1 ? { ...item, status: 'executed', response: 'Goal published' } : item
      ));
    }, 500);
  };

  const ModeButton = ({ mode, icon: Icon, label }) => (
    <button
      onClick={() => setActiveMode(mode)}
      className={`flex items-center gap-2 px-6 py-3 rounded-lg transition-all border ${
        activeMode === mode
          ? 'bg-red-600 text-white shadow-lg shadow-red-500/50 border-red-700'
          : 'bg-gray-900 text-gray-400 hover:bg-gray-800 border-red-900/30'
      }`}
    >
      <Icon size={20} />
      <span className="font-semibold">{label}</span>
    </button>
  );

  const MetricCard = ({ icon: Icon, label, value, unit, color }) => (
    <div className="bg-gray-900 rounded-xl p-4 border border-red-900/40">
      <div className="flex items-center justify-between mb-2">
        <Icon className={`${color}`} size={24} />
        <span className="text-gray-400 text-sm">{label}</span>
      </div>
      <div className="text-3xl font-bold text-white">
        {typeof value === 'number' ? value.toFixed(1) : value}
        <span className="text-lg text-gray-400 ml-1">{unit}</span>
      </div>
    </div>
  );

  return (
    <div className="min-h-screen bg-gradient-to-br from-red-950 via-black to-gray-900 text-white p-6">
    {/* Header */}
      <div className="mb-8">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-4xl font-bold bg-gradient-to-r from-red-500 to-orange-500 bg-clip-text text-transparent">
              ARES9 MARS ROVER MISSION CONTROL
            </h1>
            <p className="text-gray-400 mt-2">University Rover Challenge 2026 - @Utah Field Operations</p>
          </div>
          <div className="flex items-center gap-4">
            <div className={`flex items-center gap-2 px-4 py-2 rounded-lg border ${connected ? 'bg-green-500/20 text-green-400 border-green-800' : 'bg-red-500/20 text-red-400 border-red-800'}`}>
              <Radio className={connected ? 'animate-pulse' : ''} size={20} />
              <span className="font-semibold">{connected ? 'CONNECTED' : 'DISCONNECTED'}</span>
            </div>
            <button
                onClick={() => setConnected(!connected)}
                className="px-6 py-2 bg-gradient-to-r from-red-600 to-red-700 rounded-lg font-semibold hover:shadow-lg hover:shadow-red-500/50 transition-all border border-red-800"
              >
              {connected ? 'Disconnect' : 'Connect ROS2'}
            </button>
          </div>
        </div>

        {/* Mode Selection */}
        <div className="flex gap-3 flex-wrap">
          <ModeButton mode="telemetry" icon={Activity} label="Telemetry" />
          <ModeButton mode="navigation" icon={Navigation} label="Navigation" />
          <ModeButton mode="sensors" icon={Satellite} label="Sensors" />
          <ModeButton mode="camera" icon={Camera} label="Camera Feed" />
          <ModeButton mode="terminal" icon={Terminal} label="Command Terminal" />
          <ModeButton mode="diagnostics" icon={AlertTriangle} label="Diagnostics" />
        </div>
      </div>

      {/* System Health Bar */}
      <div className="grid grid-cols-5 gap-4 mb-6">
        <MetricCard icon={Zap} label="Battery" value={systemHealth.battery} unit="%" color="text-green-400" />
        <MetricCard icon={Cpu} label="CPU Usage" value={systemHealth.cpu} unit="%" color="text-red-400" />
        <MetricCard icon={Gauge} label="Memory" value={systemHealth.memory} unit="%" color="text-orange-400" />
        <MetricCard icon={TrendingUp} label="Temp" value={systemHealth.temperature} unit="°C" color="text-orange-400" />
        <MetricCard icon={Radio} label="Signal" value={systemHealth.signal} unit="%" color="text-red-300" />
      </div>

      {/* Main Content Area */}
      <div className="grid grid-cols-3 gap-6">
        {/* Left Column - Charts */}
        <div className="col-span-2 space-y-6">
          {activeMode === 'telemetry' && (
            <>
              <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Compass className="text-red-400" />
                  IMU Data - Orientation
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <LineChart data={imuData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#450a0a" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Line type="monotone" dataKey="pitch" stroke="#ef4444" strokeWidth={2} dot={false} />
                    <Line type="monotone" dataKey="roll" stroke="#f97316" strokeWidth={2} dot={false} />
                    <Line type="monotone" dataKey="yaw" stroke="#fb923c" strokeWidth={2} dot={false} />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Activity className="text-orange-400" />
                  Accelerometer Data
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <AreaChart data={imuData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#450a0a" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Area type="monotone" dataKey="accelX" stackId="1" stroke="#dc2626" fill="#dc262650" />
                    <Area type="monotone" dataKey="accelY" stackId="1" stroke="#ea580c" fill="#ea580c50" />
                    <Area type="monotone" dataKey="accelZ" stackId="1" stroke="#f59e0b" fill="#f59e0b50" />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
            </>
          )}

          {activeMode === 'sensors' && (
            <>
              <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Satellite className="text-purple-400" />
                  LiDAR Distance Measurements
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <LineChart data={lidarData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#450a0a" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Line type="monotone" dataKey="distance" stroke="#A855F7" strokeWidth={3} dot={false} />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Real-time LiDAR Point Cloud Visualization */}
              <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Satellite className="text-cyan-400" />
                  2D LiDAR Point Cloud (Top View)
                </h3>
                <div className="bg-black rounded-lg p-4 relative border border-red-900/30">
                  <svg width="100%" height="400" viewBox="-220 -220 440 440" className="bg-black rounded">
                    {/* Grid lines */}
                    {[-200, -100, 0, 100, 200].map(pos => (
                      <g key={pos}>
                        <line x1={pos} y1="-200" x2={pos} y2="200" stroke="#1f2937" strokeWidth="1" />
                        <line x1="-200" y1={pos} x2="200" y2={pos} stroke="#1f2937" strokeWidth="1" />
                      </g>
                    ))}
                    
                    {/* Distance circles */}
                    {[50, 100, 150, 200].map(r => (
                      <circle key={r} cx="0" cy="0" r={r} fill="none" stroke="#374151" strokeWidth="1" strokeDasharray="4,4" />
                    ))}
                    
                    {/* LiDAR points */}
                    {lidarPoints.map((point, idx) => (
                      <circle
                        key={idx}
                        cx={point.x}
                        cy={-point.y} // Invert Y for typical map view (Y-up)
                        r="2"
                        fill={`hsl(${280 - (point.distance / 250) * 80}, 100%, 60%)`}
                        opacity="0.8"
                      />
                    ))}
                    
                    {/* Rover position (center) */}
                    <g>
                      <circle cx="0" cy="0" r="8" fill="#10B981" />
                      <line x1="0" y1="0" x2="0" y2="-15" stroke="#10B981" strokeWidth="3" />
                    </g>
                    
                    {/* Distance labels */}
                    <text x="5" y="-50" fill="#6B7280" fontSize="10">50m</text>
                    <text x="5" y="-100" fill="#6B7280" fontSize="10">100m</text>
                    <text x="5" y="-150" fill="#6B7280" fontSize="10">150m</text>
                  </svg>
                  <div className="text-xs text-gray-400 mt-2 text-center">
                    Green dot = Rover position | Points colored by distance (purple=close, blue=far)
                  </div>
                </div>
              </div>

              {/* Occupancy Grid Map */}
              <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Satellite className="text-green-400" />
                  Occupancy Grid Map
                </h3>
                <div className="bg-gray-900 rounded-lg p-4">
                  <svg width="100%" height="400" viewBox="0 0 400 400" className="bg-black rounded">
                    {occupancyGrid.map((row, y) =>
                      row.map((cell, x) => (
                        <rect
                          key={`${x}-${y}`}
                          x={x * 10}
                          y={y * 10}
                          width="10"
                          height="10"
                          fill={cell > 0 ? `rgba(168, 85, 247, ${cell})` : '#000'}
                          stroke="#1f2937"
                          strokeWidth="0.5"
                        />
                      ))
                    )}
                    {/* Rover position on grid (Assuming center: 40*10/2 = 200) */}
                    <circle cx="200" cy="200" r="6" fill="#10B981" />
                  </svg>
                  <div className="text-xs text-gray-400 mt-2 text-center">
                    Cell brightness indicates obstacle probability | Green = Rover
                  </div>
                </div>
              </div>

              <div className="grid grid-cols-2 gap-4">
                <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                  <h4 className="font-semibold text-gray-400 mb-2">Point Cloud Density</h4>
                  <div className="text-4xl font-bold text-purple-400">
                    {lidarData[lidarData.length - 1]?.points.toLocaleString() || '0'}
                  </div>
                  <div className="text-sm text-gray-500 mt-1">points/scan</div>
                </div>
                <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                  <h4 className="font-semibold text-gray-400 mb-2">Average Distance</h4>
                  <div className="text-4xl font-bold text-cyan-400">
                    {lidarData[lidarData.length - 1]?.distance.toFixed(1) || '0'} m
                  </div>
                  <div className="text-sm text-gray-500 mt-1">to nearest obstacle</div>
                </div>
              </div>
            </>
          )}

          {activeMode === 'navigation' && (
            <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                <Navigation className="text-blue-400" />
                GPS Position & Map
              </h3>
              <div className="bg-gray-900 rounded-lg p-6 mb-4">
                <div className="grid grid-cols-3 gap-4 text-center">
                  <div>
                    <div className="text-sm text-gray-400 mb-1">Latitude</div>
                    <div className="text-2xl font-bold text-blue-400">{gpsData.lat.toFixed(4)}°</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-400 mb-1">Longitude</div>
                    <div className="text-2xl font-bold text-blue-400">{gpsData.lon.toFixed(4)}°</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-400 mb-1">Altitude</div>
                    <div className="text-2xl font-bold text-blue-400">{gpsData.alt} m</div>
                  </div>
                </div>
              </div>
              <div className="bg-gray-700 rounded-lg h-96 flex items-center justify-center">
                <div className="text-center">
                  <Navigation className="mx-auto mb-4 text-blue-400" size={48} />
                  <p className="text-gray-400">Map Integration: Google Maps / Mapbox API</p>
                  <p className="text-sm text-gray-500 mt-2">Add your API key to enable live mapping</p>
                </div>
              </div>
            </div>
          )}

          {activeMode === 'camera' && (
            <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                <Camera className="text-green-400" />
                Camera Feeds
              </h3>
              <div className="grid grid-cols-2 gap-4">
                <div className="bg-gray-700 rounded-lg h-64 flex items-center justify-center">
                  <div className="text-center">
                    <Camera className="mx-auto mb-2 text-green-400" size={48} />
                    <p className="text-gray-400">Front Camera</p>
                  </div>
                </div>
                <div className="bg-gray-700 rounded-lg h-64 flex items-center justify-center">
                  <div className="text-center">
                    <Camera className="mx-auto mb-2 text-blue-400" size={48} />
                    <p className="text-gray-400">Arm Camera</p>
                  </div>
                </div>
              </div>
              <p className="text-sm text-gray-500 mt-4 text-center">
                Connect ROS2 video streams via WebRTC or MJPEG
              </p>
            </div>
          )}

          {/* Diagnostics Mode - Placeholder for future implementation */}
          {activeMode === 'diagnostics' && (
            <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                <AlertTriangle className="text-yellow-400" />
                System Diagnostics
              </h3>
              <div className="bg-gray-900 rounded-lg p-6 h-96 flex items-center justify-center">
                <div className="text-center">
                  <Gauge className="mx-auto mb-4 text-yellow-400" size={48} />
                  <p className="text-gray-400">Real-time Node/Topic Latency and Error Monitoring</p>
                  <p className="text-sm text-gray-500 mt-2">Feature under development...</p>
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Right Column - Terminal & Status */}
        {activeMode === 'terminal' ? (
          <>
            {/* Left Panel - ROS2 System Monitoring (Takes 2/3 width) */}
            <div className="col-span-2 bg-black rounded-xl p-6 border-2 border-red-800 shadow-xl">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2 text-red-500">
                <Radio className="text-red-500" />
                ROS2 System Monitoring
              </h3>
              
              {/* Enhanced ROS2 Graph Visualization */}
              <div className="bg-gradient-to-br from-red-950/60 to-gray-900/60 rounded-lg p-4 border border-red-900">
                <h4 className="text-sm font-bold text-red-300 mb-3 flex items-center gap-2">
                  <Radio className="text-red-400" size={16} />
                  ROS2 COMPUTATION GRAPH
                </h4>
                
                {/* Publishers Section */}
                <div className="mb-4 pl-2">
                  <div className="flex items-center gap-2 mb-2">
                    <TrendingUp className="text-green-400" size={14} />
                    <span className="text-xs font-bold text-green-300">PUBLISHERS</span>
                  </div>
                  <div className="pl-4 border-l-2 border-green-900/50 space-y-2">
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-green-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-green-400 font-mono">/rover/sensors/imu</div>
                        <div className="text-xs text-gray-500 ml-2">→ sensor_msgs/Imu @ 50Hz</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /imu_driver</div>
                      </div>
                    </div>
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-green-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-green-400 font-mono">/rover/sensors/lidar/scan</div>
                        <div className="text-xs text-gray-500 ml-2">→ sensor_msgs/LaserScan @ 10Hz</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /lidar_processor</div>
                      </div>
                    </div>
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-green-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-green-400 font-mono">/rover/gps/fix</div>
                        <div className="text-xs text-gray-500 ml-2">→ sensor_msgs/NavSatFix @ 1Hz</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /gps_driver</div>
                      </div>
                    </div>
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-green-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-green-400 font-mono">/rover/odometry</div>
                        <div className="text-xs text-gray-500 ml-2">→ nav_msgs/Odometry @ 20Hz</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /state_estimator</div>
                      </div>
                    </div>
                  </div>
                </div>

                {/* Subscribers Section */}
                <div className="mb-4 pl-2">
                  <div className="flex items-center gap-2 mb-2">
                    <Activity className="text-blue-400" size={14} />
                    <span className="text-xs font-bold text-blue-300">SUBSCRIBERS</span>
                  </div>
                  <div className="pl-4 border-l-2 border-blue-900/50 space-y-2">
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-blue-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-blue-400 font-mono">/rover/cmd_vel</div>
                        <div className="text-xs text-gray-500 ml-2">← geometry_msgs/Twist</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /motor_controller</div>
                      </div>
                    </div>
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-blue-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-blue-400 font-mono">/rover/goal_pose</div>
                        <div className="text-xs text-gray-500 ml-2">← geometry_msgs/PoseStamped</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /navigation_planner</div>
                      </div>
                    </div>
                    <div className="flex items-start gap-2">
                      <div className="w-1.5 h-1.5 bg-blue-400 rounded-full mt-1.5"></div>
                      <div className="flex-1">
                        <div className="text-xs text-blue-400 font-mono">/rover/arm/joint_states</div>
                        <div className="text-xs text-gray-500 ml-2">← sensor_msgs/JointState</div>
                        <div className="text-xs text-gray-600 ml-2">Node: /arm_controller</div>
                      </div>
                    </div>
                  </div>
                </div>

                {/* Active Nodes Section */}
                <div className="pl-2">
                  <div className="flex items-center gap-2 mb-2">
                    <Cpu className="text-purple-400" size={14} />
                    <span className="text-xs font-bold text-purple-300">ACTIVE NODES (6)</span>
                  </div>
                  <div className="pl-4 border-l-2 border-purple-900/50">
                    <div className="flex flex-wrap gap-2">
                      {["/imu_driver", "/lidar_processor", "/gps_driver", "/state_estimator", "/motor_controller", "/navigation_planner"].map(node => (
                        <span key={node} className="text-xs bg-purple-900/40 text-purple-300 px-2 py-1 rounded border border-purple-800 font-mono">
                          {node}
                        </span>
                      ))}
                    </div>
                  </div>
                </div>

                <div className="text-xs text-gray-500 mt-3 pt-3 border-t border-red-900/30 italic">
                  Real-time ROS2 DDS communication layer visualization
                </div>
              </div>
            </div>

            {/* Right Panel - Command Terminal & Mission Control (Takes 1/3 width) */}
            <div className="col-span-1 space-y-6">
              {/* Command Terminal Section */}
              <div className="bg-black rounded-xl p-6 border-2 border-red-800 shadow-xl">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2 text-red-500">
                  <Terminal className="text-red-500" />
                  Command Terminal
                </h3>
                
                {/* Terminal Output */}
                <div 
                  ref={terminalRef}
                  className="bg-black rounded-lg border border-red-900 p-4 h-64 overflow-y-auto font-mono text-sm mb-4 text-white"
                >
                  {commandHistory.map((item, idx) => (
                    <div key={idx} className="mb-2">
                      <div className="text-red-400">
                        [{item.time}] $ {item.cmd}
                      </div>
                      {item.status === 'executed' && (
                        <div className="text-green-400 ml-4">→ {item.response}</div>
                      )}
                    </div>
                  ))}
                </div>

                {/* Command Input */}
                <div className="flex gap-2 mb-4">
                  <input
                    type="text"
                    value={command}
                    onChange={(e) => setCommand(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && sendCommand()}
                    placeholder="ros2 topic pub /cmd_vel..."
                    className="flex-1 bg-black border border-red-700 rounded-lg px-4 py-2 text-white focus:outline-none focus:border-red-500"
                  />
                  <button
                    onClick={sendCommand}
                    className="px-6 py-2 bg-red-600 rounded-lg font-semibold hover:bg-red-700 transition-all text-white"
                  >
                    Send
                  </button>
                </div>

                {/* Quick Commands */}
                <div className="space-y-2">
                  <p className="text-xs text-gray-400">Quick Commands:</p>
                  <div className="flex flex-wrap gap-2">
                    {["ros2 node list", "ros2 topic list", "ros2 service list"].map(cmd => (
                      <button
                        key={cmd}
                        onClick={() => setCommand(cmd)}
                        className="text-xs px-3 py-1 bg-red-900 text-white rounded hover:bg-red-800"
                      >
                        {cmd}
                      </button>
                    ))}
                  </div>
                </div>
              </div>

              {/* Mission Control Section */}
              <div className="bg-gray-900 rounded-xl p-6 border-2 border-red-800">
                <h4 className="text-lg font-bold mb-4 text-red-400 flex items-center gap-2">
                  <Compass className="text-red-400" size={20} /> Mission Control
                </h4>
                
                <div className="space-y-3">
                  <div className="flex flex-col">
                    <label className="text-xs text-gray-300 mb-1">Latitude</label>
                    <input
                      type="number"
                      step="0.0001"
                      className="bg-black border border-red-900 rounded px-3 py-2 text-white"
                      value={gpsData.lat}
                      onChange={e => setGpsData(d => ({...d, lat: parseFloat(e.target.value)}))}
                    />
                  </div>
                  
                  <div className="flex flex-col">
                    <label className="text-xs text-gray-300 mb-1">Longitude</label>
                    <input
                      type="number"
                      step="0.0001"
                      className="bg-black border border-red-900 rounded px-3 py-2 text-white"
                      value={gpsData.lon}
                      onChange={e => setGpsData(d => ({...d, lon: parseFloat(e.target.value)}))}
                    />
                  </div>
                  
                  <div className="flex flex-col">
                    <label className="text-xs text-gray-300 mb-1">Altitude (m)</label>
                    <input
                      type="number"
                      className="bg-black border border-red-900 rounded px-3 py-2 text-white"
                      value={gpsData.alt}
                      onChange={e => setGpsData(d => ({...d, alt: parseFloat(e.target.value)}))}
                    />
                  </div>
                  
                  <div className="flex flex-col">
                    <label className="text-xs text-gray-300 mb-1">Mission Mode</label>
                    <select
                      className="bg-black border border-red-900 rounded px-3 py-2 text-white"
                      value={missionMode}
                      onChange={e => setMissionMode(e.target.value)}
                    >
                      <option value="object">Object</option>
                      <option value="hammer">Hammer</option>
                      <option value="bottle">Bottle</option>
                    </select>
                  </div>
                  
                  <button
                    className="w-full mt-2 px-4 py-2.5 bg-red-700 hover:bg-red-800 rounded-lg font-bold text-white uppercase text-sm"
                    onClick={handleMissionStagePublish}
                  >
                    Publish Goal
                  </button>
                </div>
                
                <div className="text-xs text-gray-400 mt-4 pt-4 border-t border-red-900/30">
                  Configure mission parameters and publish navigation goals to ROS2
                </div>
              </div>
            </div>
          </>
        ) : (
          <div className="col-span-1 space-y-6">
            <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
              <h3 className="text-xl font-bold mb-4">Mission Log</h3>
              <div className="space-y-3">
                {[
                  { time: '14:23:45', msg: 'System initialized', type: 'success' },
                  { time: '14:24:12', msg: 'GPS lock acquired', type: 'success' },
                  { time: '14:25:03', msg: 'LiDAR scan complete', type: 'info' },
                  { time: '14:25:45', msg: 'Obstacle detected at 2.3m', type: 'warning' },
                ].map((log, idx) => {
                  let color;
                  if (log.type === 'success') color = 'text-green-400';
                  else if (log.type === 'warning') color = 'text-yellow-400';
                  else if (log.type === 'error') color = 'text-red-400';
                  else color = 'text-gray-300';

                  return (
                    <div key={idx} className="flex text-sm">
                      <span className="text-gray-500 mr-4">[{log.time}]</span>
                      <span className={color}>{log.msg}</span>
                    </div>
                  );
                })}
              </div>
            </div>

            <div className="bg-gray-900 rounded-xl p-6 border border-red-900/40">
                <h3 className="text-xl font-bold mb-4">Current Status</h3>
                <div className="space-y-4">
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400">Mission Mode:</span>
                    <span className="font-semibold text-cyan-400 capitalize">{missionMode} Search</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400">Last Command:</span>
                    <span className="font-semibold text-gray-300">
                      {commandHistory[commandHistory.length - 1]?.cmd || 'None'}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400">Estimated Velocity:</span>
                    <span className="font-semibold text-green-400">0.4 m/s</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400">Frame Rate (Cam):</span>
                    <span className="font-semibold text-purple-400">15 FPS</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400">System Uptime:</span>
                    <span className="font-semibold text-orange-400">1h 45m 32s</span>
                  </div>
                </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default RoverMissionControl;