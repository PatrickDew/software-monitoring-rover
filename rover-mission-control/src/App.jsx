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
  const terminalRef = useRef(null);

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

  const ModeButton = ({ mode, icon: Icon, label }) => (
    <button
      onClick={() => setActiveMode(mode)}
      className={`flex items-center gap-2 px-6 py-3 rounded-lg transition-all ${
        activeMode === mode
          ? 'bg-cyan-500 text-white shadow-lg shadow-cyan-500/50'
          : 'bg-gray-800 text-gray-400 hover:bg-gray-700'
      }`}
    >
      <Icon size={20} />
      <span className="font-semibold">{label}</span>
    </button>
  );

  const MetricCard = ({ icon: Icon, label, value, unit, color }) => (
    <div className="bg-gray-800 rounded-xl p-4 border border-gray-700">
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
    <div className="min-h-screen bg-gradient-to-br from-gray-900 via-black to-gray-900 text-white p-6">
      {/* Header */}
      <div className="mb-8">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-4xl font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">
              MARS ROVER MISSION CONTROL
            </h1>
            <p className="text-gray-400 mt-2">University Rover Challenge - Utah Field Operations</p>
          </div>
          <div className="flex items-center gap-4">
            <div className={`flex items-center gap-2 px-4 py-2 rounded-lg ${connected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
              <Radio className={connected ? 'animate-pulse' : ''} size={20} />
              <span className="font-semibold">{connected ? 'CONNECTED' : 'DISCONNECTED'}</span>
            </div>
            <button
              onClick={() => setConnected(!connected)}
              className="px-6 py-2 bg-gradient-to-r from-cyan-500 to-blue-500 rounded-lg font-semibold hover:shadow-lg hover:shadow-cyan-500/50 transition-all"
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
        <MetricCard icon={Cpu} label="CPU Usage" value={systemHealth.cpu} unit="%" color="text-blue-400" />
        <MetricCard icon={Gauge} label="Memory" value={systemHealth.memory} unit="%" color="text-purple-400" />
        <MetricCard icon={TrendingUp} label="Temp" value={systemHealth.temperature} unit="°C" color="text-orange-400" />
        <MetricCard icon={Radio} label="Signal" value={systemHealth.signal} unit="%" color="text-cyan-400" />
      </div>

      {/* Main Content Area */}
      <div className="grid grid-cols-3 gap-6">
        {/* Left Column - Charts */}
        <div className="col-span-2 space-y-6">
          {activeMode === 'telemetry' && (
            <>
              <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Compass className="text-cyan-400" />
                  IMU Data - Orientation
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <LineChart data={imuData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Line type="monotone" dataKey="pitch" stroke="#06B6D4" strokeWidth={2} dot={false} />
                    <Line type="monotone" dataKey="roll" stroke="#3B82F6" strokeWidth={2} dot={false} />
                    <Line type="monotone" dataKey="yaw" stroke="#8B5CF6" strokeWidth={2} dot={false} />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Activity className="text-green-400" />
                  Accelerometer Data
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <AreaChart data={imuData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Area type="monotone" dataKey="accelX" stackId="1" stroke="#EF4444" fill="#EF444450" />
                    <Area type="monotone" dataKey="accelY" stackId="1" stroke="#10B981" fill="#10B98150" />
                    <Area type="monotone" dataKey="accelZ" stackId="1" stroke="#F59E0B" fill="#F59E0B50" />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
            </>
          )}

          {activeMode === 'sensors' && (
            <>
              <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
                <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                  <Satellite className="text-purple-400" />
                  LiDAR Distance Measurements
                </h3>
                <ResponsiveContainer width="100%" height={250}>
                  <LineChart data={lidarData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
                    <XAxis dataKey="time" stroke="#9CA3AF" />
                    <YAxis stroke="#9CA3AF" />
                    <Tooltip contentStyle={{ backgroundColor: '#1F2937', border: 'none', borderRadius: '8px' }} />
                    <Legend />
                    <Line type="monotone" dataKey="distance" stroke="#A855F7" strokeWidth={3} dot={false} />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <div className="grid grid-cols-2 gap-4">
                <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
                  <h4 className="font-semibold text-gray-400 mb-2">Point Cloud Density</h4>
                  <div className="text-4xl font-bold text-purple-400">
                    {lidarData[lidarData.length - 1]?.points.toLocaleString() || '0'}
                  </div>
                  <div className="text-sm text-gray-500 mt-1">points/scan</div>
                </div>
                <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
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
            <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
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
            <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
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
        </div>

        {/* Right Column - Terminal & Status */}
        <div className="space-y-6">
          {activeMode === 'terminal' && (
            <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
              <h3 className="text-xl font-bold mb-4 flex items-center gap-2">
                <Terminal className="text-cyan-400" />
                ROS2 Command Terminal
              </h3>
              <div 
                ref={terminalRef}
                className="bg-black rounded-lg p-4 h-96 overflow-y-auto font-mono text-sm mb-4"
              >
                {commandHistory.map((item, idx) => (
                  <div key={idx} className="mb-2">
                    <div className="text-cyan-400">
                      [{item.time}] $ {item.cmd}
                    </div>
                    {item.status === 'executed' && (
                      <div className="text-green-400 ml-4">→ {item.response}</div>
                    )}
                  </div>
                ))}
              </div>
              <div className="flex gap-2">
                <input
                  type="text"
                  value={command}
                  onChange={(e) => setCommand(e.target.value)}
                  onKeyPress={(e) => e.key === 'Enter' && sendCommand()}
                  placeholder="ros2 topic pub /cmd_vel..."
                  className="flex-1 bg-gray-900 border border-gray-700 rounded-lg px-4 py-2 focus:outline-none focus:border-cyan-500"
                />
                <button
                  onClick={sendCommand}
                  className="px-6 py-2 bg-cyan-500 rounded-lg font-semibold hover:bg-cyan-600 transition-all"
                >
                  Send
                </button>
              </div>
              <div className="mt-4 space-y-2">
                <p className="text-xs text-gray-500">Quick Commands:</p>
                <div className="flex flex-wrap gap-2">
                  {['ros2 node list', 'ros2 topic list', 'ros2 service list'].map(cmd => (
                    <button
                      key={cmd}
                      onClick={() => setCommand(cmd)}
                      className="text-xs px-3 py-1 bg-gray-700 rounded hover:bg-gray-600"
                    >
                      {cmd}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          )}

          {activeMode !== 'terminal' && (
            <div className="bg-gray-800 rounded-xl p-6 border border-gray-700">
              <h3 className="text-xl font-bold mb-4">Mission Log</h3>
              <div className="space-y-3">
                {[
                  { time: '14:23:45', msg: 'System initialized', type: 'success' },
                  { time: '14:24:12', msg: 'GPS lock acquired', type: 'success' },
                  { time: '14:25:03', msg: 'LiDAR scan complete', type: 'info' },
                  { time: '14:25:45', msg: 'Obstacle detected at 2.3m', type: 'warning' },
                ].map((log, idx) => (
                  <div key={idx} className="flex items-start gap-3">
                    <div className={`w-2 h-2 rounded-full mt-1.5 ${
                      log.type === 'success' ? 'bg-green-400' : 
                      log.type === 'warning' ? 'bg-yellow-400' : 'bg-blue-400'
                    }`} />
                    <div className="flex-1">
                      <div className="text-xs text-gray-500">{log.time}</div>
                      <div className="text-sm">{log.msg}</div>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default RoverMissionControl;