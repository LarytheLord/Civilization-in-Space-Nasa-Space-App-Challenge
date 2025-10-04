import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Area, AreaChart } from 'recharts';

interface SensorDashboardProps {
  habitatData: any;
}

const SensorDashboard: React.FC<SensorDashboardProps> = ({ habitatData }) => {
  // Sample data for demonstration
  const temperatureData = [
    { time: '10:00', temp: 22 },
    { time: '10:05', temp: 24 },
    { time: '10:10', temp: 23 },
    { time: '10:15', temp: 25 },
    { time: '10:20', temp: 21 },
    { time: '10:25', temp: 23 },
    { time: '10:30', temp: 24 },
  ];

  // Extract values from habitat data if available
  const temperature = habitatData?.environmental_parameters?.temperature || 22;
  const radiation = habitatData?.environmental_parameters?.radiation || 45;
  const co2Level = habitatData?.environmental_parameters?.co2 || 0.04;
  const o2Level = habitatData?.environmental_parameters?.o2 || 20.9;

  // Function to determine status
  const getLevelStatus = (value: number, type: string) => {
    switch (type) {
      case 'temperature':
        return value >= 18 && value <= 25 ? 'optimal' : value >= 10 && value <= 30 ? 'caution' : 'critical';
      case 'radiation':
        return value < 30 ? 'optimal' : value < 60 ? 'caution' : 'critical';
      case 'co2':
        return value < 0.05 ? 'optimal' : value < 0.1 ? 'caution' : 'critical';
      case 'o2':
        return value > 19 ? 'optimal' : value > 17 ? 'caution' : 'critical';
      default:
        return 'optimal';
    }
  };

  const getStatusBg = (status: string) => {
    switch (status) {
      case 'optimal': return 'bg-green-500/10 border-green-500/30';
      case 'caution': return 'bg-yellow-500/10 border-yellow-500/30';
      case 'critical': return 'bg-red-500/10 border-red-500/30';
      default: return 'bg-gray-500/10 border-gray-500/30';
    }
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      {/* Temperature Chart */}
      <div className="bg-[#0f172a]/70 border border-cyan-500/20 rounded-lg p-4 scanning-line">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-cyan-500/30">
          <h4 className="font-bold text-cyan-400">TEMPERATURE</h4>
          <div className={`w-3 h-3 rounded-full ${getLevelStatus(temperature, 'temperature') === 'optimal' ? 'bg-green-500' : getLevelStatus(temperature, 'temperature') === 'caution' ? 'bg-yellow-500' : 'bg-red-500'} animate-pulse`}></div>
        </div>
        
        <div className="flex items-center justify-between mb-4">
          <div className="text-2xl font-bold text-cyan-300">
            {temperature.toFixed(1)}°C
          </div>
          <div className={`text-xs px-2 py-1 rounded ${getLevelStatus(temperature, 'temperature') === 'optimal' ? 'bg-green-900/50 text-green-400' : getLevelStatus(temperature, 'temperature') === 'caution' ? 'bg-yellow-900/50 text-yellow-400' : 'bg-red-900/50 text-red-400'}`}>
            {getLevelStatus(temperature, 'temperature').toUpperCase()}
          </div>
        </div>
        
        <div className="h-24">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={temperatureData}>
              <defs>
                <linearGradient id="colorTemp" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#0ea5e9" stopOpacity={0.6}/>
                  <stop offset="95%" stopColor="#0284c7" stopOpacity={0.1}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="#1e3a8a" vertical={false} />
              <XAxis dataKey="time" stroke="#93c5fd" tick={{ fontSize: 10 }} />
              <YAxis stroke="#93c5fd" domain={['dataMin - 1', 'dataMax + 1']} tick={{ fontSize: 10 }} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: 'rgba(15, 23, 42, 0.9)',
                  borderColor: 'rgba(59, 130, 246, 0.3)',
                  borderRadius: '6px',
                  color: '#e2e8f0',
                  backdropFilter: 'blur(10px)'
                }} 
              />
              <Area 
                type="monotone" 
                dataKey="temp" 
                stroke="#0ea5e9" 
                strokeWidth={2}
                fillOpacity={0.4} 
                fill="url(#colorTemp)" 
              />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Radiation Level */}
      <div className={`bg-gradient-to-br from-gray-800 to-gray-900 rounded-xl p-5 border ${getStatusBg(getLevelStatus(radiation, 'radiation'))}`}>
        <div className="flex justify-between items-start mb-4">
          <h4 className="font-bold text-lg">RADIATION</h4>
          <div className={`w-3 h-3 rounded-full ${getLevelStatus(radiation, 'radiation') === 'optimal' ? 'bg-green-500' : getLevelStatus(radiation, 'radiation') === 'caution' ? 'bg-yellow-500' : 'bg-red-500'} animate-pulse`}></div>
        </div>
        
        <div className="flex items-center justify-between mb-6">
          <div className="text-4xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-purple-400 to-pink-400">
            {radiation.toFixed(1)}
          </div>
          <div className="text-gray-400 text-sm">mSv/h</div>
        </div>
        
        <div className="h-32">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={[...temperatureData.map((d, i) => ({ time: d.time, radiation: radiation + (i % 3 === 0 ? -2 : i % 3 === 1 ? 1 : 0) }))]}>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#9CA3AF" />
              <YAxis stroke="#9CA3AF" domain={[0, 100]} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: '#1F2937', 
                  borderColor: '#374151',
                  borderRadius: '0.5rem',
                  color: '#fff'
                }} 
              />
              <Line 
                type="monotone" 
                dataKey="radiation" 
                stroke={getLevelStatus(radiation, 'radiation') === 'optimal' ? "#10b981" : getLevelStatus(radiation, 'radiation') === 'caution' ? "#f59e0b" : "#ef4444"} 
                strokeWidth={2} 
                dot={false} 
                activeDot={{ r: 6 }} 
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* CO2 Level */}
      <div className={`bg-gradient-to-br from-gray-800 to-gray-900 rounded-xl p-5 border ${getStatusBg(getLevelStatus(co2Level, 'co2'))}`}>
        <div className="flex justify-between items-start mb-4">
          <h4 className="font-bold text-lg">CO2 LEVEL</h4>
          <div className={`w-3 h-3 rounded-full ${getLevelStatus(co2Level, 'co2') === 'optimal' ? 'bg-green-500' : getLevelStatus(co2Level, 'co2') === 'caution' ? 'bg-yellow-500' : 'bg-red-500'} animate-pulse`}></div>
        </div>
        
        <div className="flex items-center justify-between mb-6">
          <div className="text-4xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-red-400 to-orange-400">
            {co2Level.toFixed(2)}
          </div>
          <div className="text-gray-400 text-sm">%</div>
        </div>
        
        <div className="h-32">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={[...temperatureData.map((d, i) => ({ time: d.time, co2: co2Level + (i % 3 === 0 ? -0.01 : i % 3 === 1 ? 0.005 : 0.002) }))]}>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#9CA3AF" />
              <YAxis stroke="#9CA3AF" domain={[0, 1]} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: '#1F2937', 
                  borderColor: '#374151',
                  borderRadius: '0.5rem',
                  color: '#fff'
                }} 
              />
              <Line 
                type="monotone" 
                dataKey="co2" 
                stroke={getLevelStatus(co2Level, 'co2') === 'optimal' ? "#10b981" : getLevelStatus(co2Level, 'co2') === 'caution' ? "#f59e0b" : "#ef4444"} 
                strokeWidth={2} 
                dot={false} 
                activeDot={{ r: 6 }} 
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* O2 Level */}
      <div className={`bg-gradient-to-br from-gray-800 to-gray-900 rounded-xl p-5 border ${getStatusBg(getLevelStatus(o2Level, 'o2'))}`}>
        <div className="flex justify-between items-start mb-4">
          <h4 className="font-bold text-lg">O2 LEVEL</h4>
          <div className={`w-3 h-3 rounded-full ${getLevelStatus(o2Level, 'o2') === 'optimal' ? 'bg-green-500' : getLevelStatus(o2Level, 'o2') === 'caution' ? 'bg-yellow-500' : 'bg-red-500'} animate-pulse`}></div>
        </div>
        
        <div className="flex items-center justify-between mb-6">
          <div className="text-4xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-green-400 to-emerald-400">
            {o2Level.toFixed(1)}
          </div>
          <div className="text-gray-400 text-sm">%</div>
        </div>
        
        <div className="h-32">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={[...temperatureData.map((d, i) => ({ time: d.time, o2: o2Level + (i % 3 === 0 ? -0.3 : i % 3 === 1 ? 0.2 : 0.1) }))]}>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#9CA3AF" />
              <YAxis stroke="#9CA3AF" domain={[15, 25]} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: '#1F2937', 
                  borderColor: '#374151',
                  borderRadius: '0.5rem',
                  color: '#fff'
                }} 
              />
              <Line 
                type="monotone" 
                dataKey="o2" 
                stroke={getLevelStatus(o2Level, 'o2') === 'optimal' ? "#10b981" : getLevelStatus(o2Level, 'o2') === 'caution' ? "#f59e0b" : "#ef4444"} 
                strokeWidth={2} 
                dot={false} 
                activeDot={{ r: 6 }} 
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;