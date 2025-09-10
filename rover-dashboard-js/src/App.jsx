// import { useState } from 'react'
// import reactLogo from './assets/react.svg'
// import viteLogo from '/vite.svg'
// import './App.css'

// function App() {
//   const [count, setCount] = useState(0)

//   return (
//     <>
//       <div>
//         <a href="https://vite.dev" target="_blank">
//           <img src={viteLogo} className="logo" alt="Vite logo" />
//         </a>
//         <a href="https://react.dev" target="_blank">
//           <img src={reactLogo} className="logo react" alt="React logo" />
//         </a>
//       </div>
//       <h1>Vite + React</h1>
//       <div className="card">
//         <button onClick={() => setCount((count) => count + 1)}>
//           count is {count}
//         </button>
//         <p>
//           Edit <code>src/App.jsx</code> and save to test HMR
//         </p>
//       </div>
//       <p className="read-the-docs">
//         Click on the Vite and React logos to learn more
//       </p>
//     </>
//   )
// }

// export default App

import React, { useState, useEffect, useRef } from "react";
import {
  MapPin,
  Cpu,
  Settings,
  Joystick,
  Clock,
  Battery,
  Terminal,
  Thermometer,
  Camera,
  Zap,
  Gauge,
  Radio,
  Satellite,
  AlertTriangle,
  CheckCircle,
  XCircle
} from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "./components/ui/card";
import { Button } from "./components/ui/button";
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend, Filler, ArcElement } from "chart.js";
import { Line, Doughnut } from "react-chartjs-2";
import { cn } from "@/lib/utils";

// Register Chart.js components
ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend, Filler, ArcElement);

// Enhanced Chart.js configuration for dark theme
ChartJS.defaults.color = "#e5e7eb";
ChartJS.defaults.borderColor = "rgba(239, 68, 68, 0.3)";

// Complex data generation with realistic patterns
const generateComplexHistory = (length, baseValue, variation, trend = 0, noise = 0.1) => {
  const data = [];
  let currentValue = baseValue;
  
  for (let i = 0; i < length; i++) {
    // Add trend component
    currentValue += trend;
    // Add seasonal/cyclic component
    const cyclic = Math.sin(i * 0.3) * variation * 0.3;
    // Add noise
    const randomNoise = (Math.random() - 0.5) * variation * noise;
    // Combine all components
    const value = currentValue + cyclic + randomNoise + (Math.random() - 0.5) * variation;
    data.push(Math.max(0, Math.min(100, value))); // Clamp between 0-100 for percentages
    
    // Slight drift back to base
    currentValue = currentValue * 0.99 + baseValue * 0.01;
  }
  return data;
};

const generateSensorData = () => ({
  cpu: generateComplexHistory(50, 45, 25, 0.1, 0.2),
  memory: generateComplexHistory(50, 35, 20, 0.05, 0.15),
  battery: generateComplexHistory(50, 85, 15, -0.1, 0.1),
  power: generateComplexHistory(50, 12, 8, 0, 0.3),
  temp: generateComplexHistory(50, 42, 18, 0.05, 0.25),
});

const chartOptions = {
  responsive: true,
  maintainAspectRatio: false,
  interaction: {
    mode: 'index',
    intersect: false,
  },
  plugins: {
    legend: { 
      display: true, 
      labels: { 
        color: "#e5e7eb",
        usePointStyle: true,
        pointStyle: 'circle',
        padding: 15,
        font: { size: 11, family: 'monospace' }
      }
    },
    tooltip: { 
      backgroundColor: 'rgba(0, 0, 0, 0.9)',
      titleColor: '#ef4444',
      bodyColor: '#e5e7eb',
      borderColor: '#ef4444',
      borderWidth: 1,
    },
  },
  scales: {
    x: {
      ticks: { display: false },
      grid: { display: false },
      border: { display: false },
    },
    y: {
      grid: { 
        color: "rgba(239, 68, 68, 0.1)",
        drawBorder: false,
      },
      border: { display: false },
      ticks: { 
        color: "#9ca3af",
        font: { size: 10, family: 'monospace' }
      },
    },
  },
};

// Environmental radar options removed

const logMessages = [
  { level: "INFO", component: "NAV", message: "Waypoint navigation sequence initiated" },
  { level: "INFO", component: "GPS", message: "High-precision fix acquired: 1.2847°N, 103.8514°E" },
  { level: "WARNING", component: "THERMAL", message: "Motor assembly temperature: 68.3°C - monitoring threshold" },
  { level: "INFO", component: "CMD", message: "Autonomous navigation protocol engaged" },
  { level: "ERROR", component: "COMM", message: "Intermittent signal loss from sensor array #2" },
  { level: "INFO", component: "PATH", message: "Dynamic path recalculation completed successfully" },
  { level: "SUCCESS", component: "NAV", message: "Target waypoint achieved - proceeding to next objective" },
  { level: "WARNING", component: "PWR", message: "Battery discharge rate: 2.4A/h - optimize power consumption" },
  { level: "INFO", component: "ARM", message: "Robotic arm calibration sequence completed" },
  { level: "ERROR", component: "CAM", message: "Camera focus adjustment failed - manual intervention required" },
];

// Enhanced Rover Visualization Component
const RoverVisualization = ({ componentStatus, sensorData }) => {
  const currentTemp = sensorData.temp[sensorData.temp.length - 1];
  const currentPower = sensorData.power[sensorData.power.length - 1];
  
  return (
    <div className="flex flex-col items-center justify-center p-6">
      <div className="relative w-80 h-48 bg-gradient-to-br from-gray-900 to-black rounded-2xl border-2 border-red-500 shadow-2xl">
        {/* Main Body */}
        <div className="absolute inset-4 bg-gray-800 rounded-xl border border-red-400/50">
          {/* Solar Panels */}
          <div className="absolute -top-2 left-4 right-4 h-3 bg-gradient-to-r from-blue-900 to-blue-800 rounded-full border border-blue-400/50" />
          
          {/* Camera Array */}
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex space-x-1">
            <div className={cn("w-3 h-3 rounded-full border", 
              componentStatus.camera === "OK" ? "bg-green-400 border-green-300" : "bg-red-400 border-red-300")} />
            <div className="w-2 h-2 rounded-full bg-gray-600 border border-gray-400" />
          </div>
          
          {/* Robotic Arm */}
          <div className="absolute top-6 right-2 w-12 h-2 bg-gray-700 rounded-full border border-red-400/30" />
          <div className="absolute top-4 right-8 w-8 h-2 bg-gray-600 rounded-full border border-red-400/30" />
          
          {/* Communication Array */}
          <div className="absolute top-1 right-6 w-6 h-4 bg-gradient-to-t from-gray-700 to-gray-600 rounded-t-full border border-red-400/40" />
          
          {/* Sensor Pods */}
          <div className="absolute top-1/2 left-1 -translate-y-1/2 flex flex-col space-y-1">
            <div className={cn("w-4 h-4 rounded-full border-2", 
              componentStatus.sensors === "OK" ? "bg-green-500 border-green-300" : "bg-red-500 border-red-300")} />
            <div className="w-3 h-3 rounded-full bg-blue-500 border border-blue-300" />
          </div>
          <div className="absolute top-1/2 right-1 -translate-y-1/2 flex flex-col space-y-1">
            <div className="w-4 h-4 rounded-full bg-yellow-500 border border-yellow-300" />
            <div className="w-3 h-3 rounded-full bg-purple-500 border border-purple-300" />
          </div>
          
          {/* Central Processing Unit */}
          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-16 h-8 bg-gradient-to-br from-red-600 to-red-800 rounded-lg border border-red-400">
            <div className="absolute inset-1 bg-black rounded border border-red-500/50 flex items-center justify-center">
              <span className="text-red-400 font-bold text-xs tracking-wider">ARES9</span>
            </div>
          </div>
        </div>
        
        {/* Wheels/Tracks */}
        <div className="absolute bottom-1 left-6 w-10 h-6 rounded-full bg-gray-800 border-2 border-red-400/60" />
        <div className="absolute bottom-1 right-6 w-10 h-6 rounded-full bg-gray-800 border-2 border-red-400/60" />
        <div className="absolute bottom-1 left-1/2 -translate-x-1/2 w-8 h-4 rounded-full bg-gray-700 border border-red-400/40" />
      </div>
      
      {/* Component Status Grid */}
      <div className="grid grid-cols-4 gap-3 mt-6 text-xs font-mono">
        {[
          { key: 'camera', label: 'Camera', icon: Camera },
          { key: 'arm', label: 'Robotic Arm', icon: Settings },
          { key: 'sensors', label: 'Sensors', icon: Radio },
          { key: 'comm', label: 'Comms', icon: Satellite }
        ].map(({ key, label, icon: Icon }) => (
          <div key={key} className="flex flex-col items-center space-y-1 p-2 rounded bg-gray-900/50 border border-red-500/20">
            <Icon className={cn("w-4 h-4", 
              componentStatus[key] === "OK" ? "text-green-400" : 
              componentStatus[key] === "WARNING" ? "text-yellow-400" : "text-red-400")} />
            <div className={cn("w-2 h-2 rounded-full", 
              componentStatus[key] === "OK" ? "bg-green-500" : 
              componentStatus[key] === "WARNING" ? "bg-yellow-500" : "bg-red-500")} />
            <span className="text-gray-300 text-center">{label}</span>
          </div>
        ))}
      </div>
    </div>
  );
};

export default function RoverDashboard() {
  const [tab, setTab] = useState("overview");
  const [data, setData] = useState(generateSensorData);
  const [logs, setLogs] = useState(logMessages.slice(-8));
  const [missionTime, setMissionTime] = useState(5025); // seconds
  const [missionMode, setMissionMode] = useState("manual"); // manual | autonomous
  const [teleopTab, setTeleopTab] = useState("motion"); // motion | arm | system | connection
  const [rosStatus, setRosStatus] = useState({
    nodes: [
      { name: "/navigation", status: "running" },
      { name: "/perception", status: "running" },
      { name: "/teleop", status: "idle" },
      { name: "/motor_controller", status: "running" },
      { name: "/camera_driver", status: "running" },
    ],
    topics: [
      { name: "/cmd_vel", type: "geometry_msgs/Twist", hz: 10 },
      { name: "/odom", type: "nav_msgs/Odometry", hz: 20 },
      { name: "/tf", type: "tf2_msgs/TFMessage", hz: 30 },
      { name: "/camera/image_raw", type: "sensor_msgs/Image", hz: 15 },
    ],
  });

  // Real-time data updater
  useEffect(() => {
    const interval = setInterval(() => {
      setData(prev => ({
        cpu: [...prev.cpu.slice(1), Math.max(20, Math.min(95, prev.cpu[prev.cpu.length - 1] + (Math.random() - 0.5) * 8))],
        memory: [...prev.memory.slice(1), Math.max(15, Math.min(85, prev.memory[prev.memory.length - 1] + (Math.random() - 0.5) * 6))],
        battery: [...prev.battery.slice(1), Math.max(15, Math.min(100, prev.battery[prev.battery.length - 1] - 0.02 + (Math.random() - 0.5) * 0.5))],
        power: [...prev.power.slice(1), Math.max(5, Math.min(25, prev.power[prev.power.length - 1] + (Math.random() - 0.5) * 3))],
        temp: [...prev.temp.slice(1), Math.max(25, Math.min(85, prev.temp[prev.temp.length - 1] + (Math.random() - 0.5) * 4))],
      }));

      // Add new log entry
      const randomLog = logMessages[Math.floor(Math.random() * logMessages.length)];
      const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
      setLogs(prev => [...prev.slice(-9), {
        ...randomLog,
        message: `[${timestamp}] ${randomLog.message}`,
        timestamp
      }]);

      setMissionTime(prev => prev + 2);
    }, 2000);

    return () => clearInterval(interval);
  }, []);

  // Mock ROS2 status updates
  useEffect(() => {
    const interval = setInterval(() => {
      setRosStatus(prev => ({
        ...prev,
        nodes: prev.nodes.map(n => ({
          ...n,
          status: Math.random() < 0.95 ? n.status : (n.status === "running" ? "warning" : "running")
        })),
        topics: prev.topics.map(t => ({
          ...t,
          hz: Math.max(0, Math.round(t.hz + (Math.random() - 0.5) * 2))
        }))
      }));
    }, 3000);
    return () => clearInterval(interval);
  }, []);

  // Format mission time
  const formatMissionTime = (seconds) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  const getComponentStatus = () => {
    const currentTemp = data.temp[data.temp.length - 1];
    const currentBattery = data.battery[data.battery.length - 1];
    const currentCPU = data.cpu[data.cpu.length - 1];
    
    return {
      camera: currentCPU > 85 ? "WARNING" : "OK",
      arm: currentTemp > 70 ? "CRITICAL" : currentTemp > 60 ? "WARNING" : "OK",
      sensors: "OK",
      comm: currentBattery < 20 ? "WARNING" : "OK",
    };
  };

  // Chart data configurations
  const systemPerformanceData = {
    labels: Array(data.cpu.length).fill(""),
    datasets: [
      {
        label: "CPU Usage (%)",
        data: data.cpu,
        borderColor: "#ef4444",
        backgroundColor: "rgba(239, 68, 68, 0.1)",
        tension: 0.4,
        pointRadius: 0,
        borderWidth: 2,
      },
      {
        label: "Memory Usage (%)",
        data: data.memory,
        borderColor: "#3b82f6",
        backgroundColor: "rgba(59, 130, 246, 0.1)",
        tension: 0.4,
        pointRadius: 0,
        borderWidth: 2,
      },
      {
        label: "Temperature (°C)",
        data: data.temp,
        borderColor: "#f59e0b",
        backgroundColor: "rgba(245, 158, 11, 0.1)",
        tension: 0.4,
        pointRadius: 0,
        borderWidth: 2,
      },
    ],
  };

  // environmentalData removed

  const powerSystemData = {
    labels: ['Battery', 'Solar Input', 'System Load', 'Reserve'],
    datasets: [{
      data: [
        data.battery[data.battery.length - 1].toFixed(1),
        (data.power[data.power.length - 1] * 2).toFixed(1),
        (100 - data.battery[data.battery.length - 1]).toFixed(1),
        15
      ],
      backgroundColor: [
        '#22c55e',
        '#eab308',
        '#ef4444',
        '#6366f1'
      ],
      borderColor: '#ffffff',
      borderWidth: 2,
    }]
  };

  const currentBatteryLevel = data.battery[data.battery.length - 1];
  const currentCPU = data.cpu[data.cpu.length - 1];
  const currentTemp = data.temp[data.temp.length - 1];

  return (
    <div className="flex h-screen bg-black text-white font-mono overflow-hidden">
      {/* Sidebar */}
      <div className="w-72 bg-gradient-to-b from-gray-900 to-black shadow-2xl p-4 space-y-4 border-r border-red-500/30">
        <div className="text-center border-b border-red-500/30 pb-4">
          <h1 className="text-2xl font-bold text-red-500 tracking-wider">ARES9</h1>
          <p className="text-xs text-gray-400 mt-1">Mars Exploration Rover</p>
          <div className="flex items-center justify-center mt-2 text-sm text-green-400">
            <CheckCircle className="w-4 h-4 mr-2" />
            OPERATIONAL
          </div>
        </div>
        
        <nav className="space-y-2">
          {[
            { id: "overview", label: "Overview", icon: MapPin },
            { id: "navigation", label: "Navigation", icon: MapPin },
            { id: "ros2", label: "ROS2 Monitor", icon: Radio },
            { id: "teleop", label: "Teleop", icon: Joystick },
            { id: "logs", label: "Logs", icon: Terminal },
          ].map(({ id, label, icon: Icon }) => (
            <Button
              key={id}
              variant={tab === id ? "secondary" : "ghost"}
              className={cn(
                "w-full justify-start text-sm font-semibold transition-all",
                tab === id 
                  ? "bg-red-500 text-white shadow-lg" 
                  : "text-gray-300 hover:text-red-400 hover:bg-red-500/10"
              )}
              onClick={() => setTab(id)}
            >
              <Icon className="mr-3 h-4 w-4" /> {label}
            </Button>
          ))}
        </nav>

        <div className="border-t border-red-500/30 pt-4 space-y-3">
          <div className="flex items-center justify-between text-xs">
            <span className="text-gray-400">Mission Mode:</span>
            <span className={cn("font-bold", missionMode === "manual" ? "text-yellow-400" : "text-green-400")}>{missionMode.toUpperCase()}</span>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <Button className={cn("text-xs", missionMode === "manual" ? "bg-yellow-600" : "bg-gray-800")}
              onClick={() => setMissionMode("manual")}>
              Manual
            </Button>
            <Button className={cn("text-xs", missionMode === "autonomous" ? "bg-green-600" : "bg-gray-800")}
              onClick={() => setMissionMode("autonomous")}>
              Autonomous
            </Button>
          </div>
        </div>

        <div className="border-t border-red-500/30 pt-4 space-y-3">
          <div className="flex items-center justify-between text-xs">
            <span className="text-gray-400">Mission Time:</span>
            <span className="text-green-400 font-bold">{formatMissionTime(missionTime)}</span>
          </div>
          <div className="flex items-center justify-between text-xs">
            <span className="text-gray-400">Sol (Mars Day):</span>
            <span className="text-yellow-400 font-bold">2847</span>
          </div>
          <div className="flex items-center justify-between text-xs">
            <span className="text-gray-400">Distance:</span>
            <span className="text-blue-400 font-bold">24.7 km</span>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 p-6 overflow-y-auto bg-gradient-to-br from-gray-900 to-black">
        {tab === "overview" && (
          <div className="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-6">
            <Card className="xl:col-span-2 bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Settings className="mr-2" />Rover System Status</CardTitle></CardHeader>
              <CardContent>
                <RoverVisualization componentStatus={getComponentStatus()} sensorData={data} />
              </CardContent>
            </Card>

            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Camera className="mr-2" />Live Feed</CardTitle></CardHeader>
              <CardContent className="aspect-video bg-black flex items-center justify-center rounded-lg border-2 border-red-500/20 relative overflow-hidden">
                <div className="absolute inset-0 bg-gradient-to-br from-red-900/20 to-orange-900/20"></div>
                <div className="relative z-10 text-center">
                  <div className="w-12 h-12 border-2 border-red-500 rounded-full mb-4 mx-auto animate-pulse"></div>
                  <span className="text-red-400 font-bold tracking-wider text-sm">MARS SURFACE CAM</span>
                  <div className="text-xs text-gray-400 mt-2">Signal Strength: 87%</div>
                </div>
              </CardContent>
            </Card>

            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Gauge className="mr-2" />Power Systems</CardTitle></CardHeader>
              <CardContent className="h-64">
                <Doughnut data={powerSystemData} options={{
                  responsive: true,
                  maintainAspectRatio: false,
                  plugins: {
                    legend: {
                      position: 'bottom',
                      labels: {
                        color: '#e5e7eb',
                        font: { size: 10, family: 'monospace' },
                        usePointStyle: true,
                        padding: 15,
                      }
                    }
                  }
                }} />
              </CardContent>
            </Card>
            
            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Cpu className="mr-2" />System Performance</CardTitle></CardHeader>
              <CardContent className="h-64">
                <Line data={systemPerformanceData} options={chartOptions} />
              </CardContent>
            </Card>

            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><MapPin className="mr-2" />Navigation Map</CardTitle></CardHeader>
              <CardContent className="aspect-square bg-black flex items-center justify-center rounded-lg border-2 border-red-500/20 relative">
                <div className="absolute inset-0 bg-gradient-radial from-red-900/10 to-transparent"></div>
                <div className="relative z-10 text-center">
                  <div className="w-16 h-16 border-2 border-green-500 rounded-full mb-4 mx-auto relative">
                    <div className="absolute top-1/2 left-1/2 w-2 h-2 bg-green-500 rounded-full -translate-x-1/2 -translate-y-1/2 animate-pulse"></div>
                  </div>
                  <span className="text-green-400 font-bold tracking-wider text-sm">CURRENT POSITION</span>
                  <div className="text-xs text-gray-400 mt-2">Lat: -14.5684° Lon: 175.4728°</div>
                </div>
              </CardContent>
            </Card>

            <Card className="xl:col-span-3 bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Zap className="mr-2" />Critical System Metrics</CardTitle></CardHeader>
              <CardContent>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-6">
                  {[
                    { label: "Battery Level", value: `${currentBatteryLevel.toFixed(1)}%`, icon: Battery, color: currentBatteryLevel > 50 ? "text-green-400" : currentBatteryLevel > 20 ? "text-yellow-400" : "text-red-400" },
                    { label: "CPU Usage", value: `${currentCPU.toFixed(1)}%`, icon: Cpu, color: currentCPU < 70 ? "text-green-400" : currentCPU < 85 ? "text-yellow-400" : "text-red-400" },
                    { label: "Temperature", value: `${currentTemp.toFixed(1)}°C`, icon: Thermometer, color: currentTemp < 60 ? "text-green-400" : currentTemp < 75 ? "text-yellow-400" : "text-red-400" },
                    { label: "Mission Time", value: formatMissionTime(missionTime), icon: Clock, color: "text-blue-400" },
                  ].map(({ label, value, icon: Icon, color }) => (
                    <div key={label} className="text-center p-4 rounded-lg bg-black/30 border border-red-500/20">
                      <Icon className={cn("w-8 h-8 mx-auto mb-2", color)} />
                      <div className="text-xs text-gray-400 mb-1">{label}</div>
                      <div className={cn("text-xl font-bold", color)}>{value}</div>
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </div>
        )}

        {tab === "navigation" && (
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <Card className="lg:col-span-2 bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><MapPin className="mr-2" />Navigation</CardTitle></CardHeader>
              <CardContent className="aspect-video bg-black flex items-center justify-center rounded-lg border-2 border-red-500/20">
                <span className="text-gray-400 text-sm">Navigation map placeholder</span>
              </CardContent>
            </Card>
            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400">Waypoints</CardTitle></CardHeader>
              <CardContent className="space-y-3 text-sm text-gray-300">
                <div className="flex items-center justify-between"><span>WP1</span><span className="text-green-400">Reached</span></div>
                <div className="flex items-center justify-between"><span>WP2</span><span className="text-yellow-400">En route</span></div>
                <div className="flex items-center justify-between"><span>WP3</span><span className="text-gray-400">Pending</span></div>
              </CardContent>
            </Card>
          </div>
        )}

        {tab === "ros2" && (
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Radio className="mr-2" />Nodes</CardTitle></CardHeader>
              <CardContent className="space-y-2 text-sm">
                {rosStatus.nodes.map(n => (
                  <div key={n.name} className="flex items-center justify-between p-2 bg-black/40 rounded border border-red-500/20">
                    <span className="text-gray-300">{n.name}</span>
                    <span className={cn("text-xs px-2 py-1 rounded",
                      n.status === "running" ? "bg-green-900/50 text-green-400" :
                      n.status === "warning" ? "bg-yellow-900/50 text-yellow-400" : "bg-gray-700 text-gray-300"
                    )}>{n.status}</span>
                  </div>
                ))}
              </CardContent>
            </Card>
            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Satellite className="mr-2" />Topics</CardTitle></CardHeader>
              <CardContent className="space-y-2 text-sm">
                {rosStatus.topics.map(t => (
                  <div key={t.name} className="p-2 bg-black/40 rounded border border-red-500/20">
                    <div className="flex items-center justify-between">
                      <span className="text-gray-300">{t.name}</span>
                      <span className="text-gray-400">{t.type}</span>
                    </div>
                    <div className="text-xs text-gray-500 mt-1">Hz: <span className="text-blue-400 font-semibold">{t.hz}</span></div>
                  </div>
                ))}
              </CardContent>
            </Card>
          </div>
        )}

        {tab === "teleop" && (
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
            {/* Left sub-navigation */}
            <Card className="lg:col-span-1 bg-gradient-to-b from-gray-900 to-gray-950 border-2 border-red-500/30 shadow-2xl h-fit">
              <CardHeader>
                <CardTitle className="text-red-400 flex items-center"><Joystick className="mr-2" />Teleop Panels</CardTitle>
              </CardHeader>
              <CardContent className="space-y-2">
                {[
                  { id: "motion", label: "Motion" },
                  { id: "arm", label: "Arm" },
                  { id: "system", label: "System" },
                  { id: "connection", label: "Connection" },
                ].map(item => (
                  <Button key={item.id}
                    className={cn("w-full justify-start", teleopTab === item.id ? "bg-red-500" : "bg-gray-800 hover:bg-gray-700")}
                    onClick={() => setTeleopTab(item.id)}>
                    {item.label}
                  </Button>
                ))}
              </CardContent>
            </Card>

            {/* Right panel content */}
            <div className="lg:col-span-3 grid grid-cols-1 gap-6">
              {teleopTab === "motion" && (
                <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
                  <CardHeader><CardTitle className="text-red-400">Motion Commands</CardTitle></CardHeader>
                  <CardContent className="space-y-6">
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                      <div>
                        <div className="text-xs text-gray-400 mb-1">Linear X (m/s)</div>
                        <input type="range" min="-1" max="1" step="0.1" defaultValue="0" className="w-full accent-red-500" />
                      </div>
                      <div>
                        <div className="text-xs text-gray-400 mb-1">Linear Y (m/s)</div>
                        <input type="range" min="-1" max="1" step="0.1" defaultValue="0" className="w-full accent-red-500" />
                      </div>
                      <div>
                        <div className="text-xs text-gray-400 mb-1">Angular Z (rad/s)</div>
                        <input type="range" min="-2" max="2" step="0.1" defaultValue="0" className="w-full accent-red-500" />
                      </div>
                    </div>
                    <div className="grid grid-cols-3 gap-3 max-w-xs">
                      <div></div>
                      <Button className="bg-red-600 hover:bg-red-500">↑ Forward</Button>
                      <div></div>
                      <Button className="bg-red-600 hover:bg-red-500">← Left</Button>
                      <Button className="bg-black border-2 border-red-500 text-red-400 hover:bg-red-500 hover:text-white">STOP</Button>
                      <Button className="bg-red-600 hover:bg-red-500">Right →</Button>
                      <div></div>
                      <Button className="bg-red-600 hover:bg-red-500">↓ Backward</Button>
                      <div></div>
                    </div>
                    <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
                      <Button className="bg-gray-800 hover:bg-gray-700">Rotate ⟲</Button>
                      <Button className="bg-gray-800 hover:bg-gray-700">Rotate ⟳</Button>
                      <Button className="bg-gray-800 hover:bg-gray-700">Nudge +X</Button>
                      <Button className="bg-gray-800 hover:bg-gray-700">Nudge -X</Button>
                    </div>
                  </CardContent>
                </Card>
              )}

              {teleopTab === "arm" && (
                <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
                  <CardHeader><CardTitle className="text-red-400">Arm Controls</CardTitle></CardHeader>
                  <CardContent className="space-y-6">
                    <div className="grid grid-cols-3 gap-3 max-w-md">
                      <Button className="bg-gray-800 hover:bg-gray-700">Home</Button>
                      <Button className="bg-gray-800 hover:bg-gray-700">Stow</Button>
                      <Button className="bg-gray-800 hover:bg-gray-700">Sample</Button>
                    </div>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                      {['Shoulder','Elbow','Wrist','Gripper'].map(j => (
                        <div key={j}>
                          <div className="text-xs text-gray-400 mb-1">{j}</div>
                          <input type="range" min="-180" max="180" defaultValue="0" className="w-full accent-red-500" />
                        </div>
                      ))}
                    </div>
                    <div className="flex gap-3">
                      <Button className="bg-red-600 hover:bg-red-500">E-Stop Arm</Button>
                      <Button className="bg-green-600 hover:bg-green-500">Enable</Button>
                    </div>
                  </CardContent>
                </Card>
              )}

              {teleopTab === "system" && (
                <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
                  <CardHeader><CardTitle className="text-red-400">System Commands</CardTitle></CardHeader>
                  <CardContent className="space-y-6">
                    <div className="flex gap-3">
                      <Button className="bg-red-700 hover:bg-red-600">E-Stop</Button>
                      <Button className="bg-green-700 hover:bg-green-600">Resume</Button>
                      <Button className="bg-yellow-700 hover:bg-yellow-600">Clear Faults</Button>
                    </div>
                    <div className="grid grid-cols-2 gap-4 max-w-md">
                      <div>
                        <div className="text-xs text-gray-400 mb-1">Mode</div>
                        <div className="grid grid-cols-2 gap-2">
                          <Button className={cn("text-xs", missionMode === "manual" ? "bg-yellow-600" : "bg-gray-800")}
                            onClick={() => setMissionMode("manual")}>
                            Manual
                          </Button>
                          <Button className={cn("text-xs", missionMode === "autonomous" ? "bg-green-600" : "bg-gray-800")}
                            onClick={() => setMissionMode("autonomous")}>
                            Autonomous
                          </Button>
                        </div>
                      </div>
                      <div>
                        <div className="text-xs text-gray-400 mb-1">Nodes</div>
                        <div className="grid grid-cols-2 gap-2">
                          <Button className="bg-gray-800 hover:bg-gray-700">Restart NAV</Button>
                          <Button className="bg-gray-800 hover:bg-gray-700">Restart CAM</Button>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              )}

              {teleopTab === "connection" && (
                <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
                  <CardHeader><CardTitle className="text-red-400">ROS2 Bridge Settings</CardTitle></CardHeader>
                  <CardContent className="space-y-4">
                    <div>
                      <div className="text-xs text-gray-400 mb-1">WebSocket URL</div>
                      <input type="text" defaultValue="ws://localhost:9090" className="w-full bg-black/50 border border-red-500/20 rounded px-3 py-2 text-sm text-gray-200 placeholder-gray-500" placeholder="ws://host:port" />
                    </div>
                    <div className="flex gap-3">
                      <Button className="bg-green-700 hover:bg-green-600">Connect</Button>
                      <Button className="bg-red-700 hover:bg-red-600">Disconnect</Button>
                    </div>
                    <div className="text-xs text-gray-400">Status: <span className="text-yellow-400">DISCONNECTED</span></div>
                  </CardContent>
                </Card>
              )}
            </div>
          </div>
        )}

        {tab === "logs" && (
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <Card className="lg:col-span-2 bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><Terminal className="mr-2" />System Event Log</CardTitle></CardHeader>
              <CardContent className="h-96 overflow-y-auto p-4 bg-black/50 rounded-lg text-sm text-gray-300 font-mono border border-red-500/20">
                <div className="space-y-1">
                  {logs.map((log, index) => (
                    <div key={index} className={cn("flex items-start space-x-3 p-2 rounded transition-colors",
                      log.level === "ERROR" ? "bg-red-900/20 border-l-4 border-red-500" :
                      log.level === "WARNING" ? "bg-yellow-900/20 border-l-4 border-yellow-500" :
                      log.level === "SUCCESS" ? "bg-green-900/20 border-l-4 border-green-500" :
                      "bg-gray-900/20 border-l-4 border-gray-500"
                    )}>
                      <div className={cn("w-2 h-2 rounded-full mt-2 flex-shrink-0",
                        log.level === "ERROR" ? "bg-red-500" :
                        log.level === "WARNING" ? "bg-yellow-500" :
                        log.level === "SUCCESS" ? "bg-green-500" :
                        "bg-gray-500"
                      )} />
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center space-x-2 mb-1">
                          <span className={cn("text-xs px-2 py-1 rounded font-semibold",
                            log.level === "ERROR" ? "bg-red-500 text-white" :
                            log.level === "WARNING" ? "bg-yellow-500 text-black" :
                            log.level === "SUCCESS" ? "bg-green-500 text-white" :
                            "bg-gray-500 text-white"
                          )}>
                            {log.level}
                          </span>
                          <span className="text-blue-400 text-xs font-medium">{log.component}</span>
                          {log.timestamp && (
                            <span className="text-gray-500 text-xs">{log.timestamp}</span>
                          )}
                        </div>
                        <p className="text-gray-300 text-sm leading-relaxed">{log.message}</p>
                      </div>
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>

            <Card className="bg-gradient-to-br from-gray-900 to-gray-800 border-2 border-red-500/30 shadow-2xl">
              <CardHeader><CardTitle className="text-red-400 flex items-center"><AlertTriangle className="mr-2" />Alert Summary</CardTitle></CardHeader>
              <CardContent className="space-y-4">
                <div className="grid gap-3">
                  {[
                    { type: "CRITICAL", count: 0, color: "text-red-400", bgColor: "bg-red-900/30" },
                    { type: "WARNING", count: 2, color: "text-yellow-400", bgColor: "bg-yellow-900/30" },
                    { type: "INFO", count: 15, color: "text-blue-400", bgColor: "bg-blue-900/30" },
                    { type: "SUCCESS", count: 8, color: "text-green-400", bgColor: "bg-green-900/30" },
                  ].map(({ type, count, color, bgColor }) => (
                    <div key={type} className={cn("p-3 rounded-lg border", bgColor, 
                      type === "CRITICAL" ? "border-red-500/30" :
                      type === "WARNING" ? "border-yellow-500/30" :
                      type === "INFO" ? "border-blue-500/30" :
                      "border-green-500/30"
                    )}>
                      <div className="flex justify-between items-center">
                        <span className="text-sm text-gray-400">{type}</span>
                        <span className={cn("text-lg font-bold", color)}>{count}</span>
                      </div>
                    </div>
                  ))}
                </div>

                <div className="border-t border-red-500/20 pt-4">
                  <h3 className="text-red-400 font-semibold mb-3 text-sm">System Health</h3>
                  <div className="space-y-3">
                    {[
                      { system: "Navigation", health: 98, status: "OPTIMAL" },
                      { system: "Power Management", health: 92, status: "GOOD" },
                      { system: "Communications", health: 89, status: "GOOD" },
                      { system: "Life Support", health: 100, status: "OPTIMAL" },
                    ].map(({ system, health, status }) => (
                      <div key={system} className="space-y-1">
                        <div className="flex justify-between items-center text-xs">
                          <span className="text-gray-400">{system}</span>
                          <span className={cn("font-semibold",
                            health >= 95 ? "text-green-400" :
                            health >= 80 ? "text-yellow-400" : "text-red-400"
                          )}>{status}</span>
                        </div>
                        <div className="w-full bg-gray-800 rounded-full h-2 overflow-hidden">
                          <div 
                            className={cn("h-full transition-all duration-1000",
                              health >= 95 ? "bg-green-500" :
                              health >= 80 ? "bg-yellow-500" : "bg-red-500"
                            )}
                            style={{ width: `${health}%` }}
                          />
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        )}
      </div>
    </div>
  );
}