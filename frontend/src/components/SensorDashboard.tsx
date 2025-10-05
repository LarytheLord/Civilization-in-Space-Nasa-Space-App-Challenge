import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Area, AreaChart } from 'recharts';

interface SensorDashboardProps {
  habitatData: any;
}

const SensorDashboard: React.FC<SensorDashboardProps> = ({ habitatData }) => {
  // LUNAR-ACCURATE sensor data (no O2/CO2 - Moon has no atmosphere!)
  const temperatureData = [
    { time: '10:00', temp: -23.7 },
    { time: '10:05', temp: -21.2 },
    { time: '10:10', temp: -19.8 },
    { time: '10:15', temp: -18.4 },
    { time: '10:20', temp: -17.9 },
    { time: '10:25', temp: -16.5 },
    { time: '10:30', temp: -15.2 },
  ];

  // Realistic LUNAR parameters (based on actual lunar data)
  const temperature = habitatData?.environmental_parameters?.temperature || -23.7; // Lunar surface temp
  const radiation = habitatData?.environmental_parameters?.radiation || 46.5; // mSv/h (high on Moon)
  const regolithDensity = habitatData?.regolith_density || 1.52; // g/cm³
  const waterIce = habitatData?.water_ice_detection || 2.8; // percentage detected
  const electrostaticCharge = habitatData?.electrostatic_charge || 45; // kV/m (lunar dust hazard)
  const lidarObstacleDensity = habitatData?.lidar_obstacle_density || 0.8; // obstacles per m²

  // Function to determine status
  const getLevelStatus = (value: number, type: string) => {
    switch (type) {
      case 'temperature':
        return value >= -30 && value <= 10 ? 'nominal' : value >= -50 && value <= 30 ? 'caution' : 'critical';
      case 'radiation':
        return value < 50 ? 'nominal' : value < 80 ? 'caution' : 'critical';
      case 'regolith':
        return value >= 1.2 && value <= 1.8 ? 'nominal' : 'caution';
      case 'waterice':
        return value > 2 ? 'nominal' : value > 1 ? 'caution' : 'low';
      case 'electrostatic':
        return value < 50 ? 'nominal' : value < 80 ? 'caution' : 'critical';
      case 'lidar':
        return value < 1.0 ? 'clear' : value < 2.0 ? 'moderate' : 'high';
      default:
        return 'nominal';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'nominal': case 'clear': return 'text-emerald-400';
      case 'caution': case 'moderate': return 'text-amber-400';
      case 'critical': case 'high': case 'low': return 'text-red-400';
      default: return 'text-slate-400';
    }
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      {/* Temperature - Lunar Surface */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">TEMPERATURE</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(temperature, 'temperature') === 'nominal' ? 'bg-emerald-400' : getLevelStatus(temperature, 'temperature') === 'caution' ? 'bg-amber-400' : 'bg-red-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {temperature.toFixed(1)}°C
          </div>
          <div className={`text-xs px-2 py-0.5 rounded-full border ${getStatusColor(getLevelStatus(temperature, 'temperature'))} border-white/10 bg-white/5`}>
            {getLevelStatus(temperature, 'temperature').toUpperCase()}
          </div>
        </div>
        
        <div className="h-20">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={temperatureData}>
              <defs>
                <linearGradient id="colorTemp" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#3b82f6" stopOpacity={0.3}/>
                  <stop offset="95%" stopColor="#3b82f6" stopOpacity={0.05}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148, 163, 184, 0.1)" vertical={false} />
              <XAxis dataKey="time" stroke="#64748b" tick={{ fontSize: 10 }} />
              <YAxis stroke="#64748b" tick={{ fontSize: 10 }} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: 'rgba(15, 23, 42, 0.95)',
                  borderColor: 'rgba(148, 163, 184, 0.2)',
                  borderRadius: '6px',
                  color: '#e2e8f0',
                }} 
              />
              <Area 
                type="monotone" 
                dataKey="temp" 
                stroke="#3b82f6" 
                strokeWidth={1.5}
                fillOpacity={0.4} 
                fill="url(#colorTemp)" 
              />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Radiation Level (Lunar surface - high!) */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">RADIATION</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(radiation, 'radiation') === 'nominal' ? 'bg-emerald-400' : getLevelStatus(radiation, 'radiation') === 'caution' ? 'bg-amber-400' : 'bg-red-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {radiation.toFixed(1)}
          </div>
          <div className="text-xs text-slate-500">mSv/h</div>
        </div>
        
        <div className={`text-xs px-2 py-1 rounded-full border mb-3 ${getStatusColor(getLevelStatus(radiation, 'radiation'))} border-white/10 bg-white/5 inline-block`}>
          {getLevelStatus(radiation, 'radiation').toUpperCase()}
        </div>
        
        <div className="h-16">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={[...temperatureData.map((d, i) => ({ time: d.time, radiation: radiation + (i % 3 === 0 ? -2 : i % 3 === 1 ? 1 : 0) }))]}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148, 163, 184, 0.1)" vertical={false} />
              <XAxis dataKey="time" stroke="#64748b" tick={{ fontSize: 10 }} />
              <YAxis stroke="#64748b" tick={{ fontSize: 10 }} />
              <Tooltip 
                contentStyle={{ 
                  backgroundColor: 'rgba(15, 23, 42, 0.95)', 
                  borderColor: 'rgba(148, 163, 184, 0.2)',
                  borderRadius: '6px',
                  color: '#e2e8f0'
                }} 
              />
              <Line 
                type="monotone" 
                dataKey="radiation" 
                stroke="#64748b" 
                strokeWidth={1.5} 
                dot={false} 
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Regolith Density (for construction assessment) */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">REGOLITH DENSITY</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(regolithDensity, 'regolith') === 'nominal' ? 'bg-emerald-400' : 'bg-amber-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {regolithDensity.toFixed(2)}
          </div>
          <div className="text-xs text-slate-500">g/cm³</div>
        </div>
        
        <div className={`text-xs px-2 py-1 rounded-full border ${getStatusColor(getLevelStatus(regolithDensity, 'regolith'))} border-white/10 bg-white/5 inline-block`}>
          {getLevelStatus(regolithDensity, 'regolith').toUpperCase()}
        </div>
        
        <div className="text-xs text-slate-400 mt-3">
          Composition: Fe 4.5%, Ti 0.8%
        </div>
      </div>
      
      {/* Water Ice Detection (ISRU potential) */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">WATER ICE DETECTION</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(waterIce, 'waterice') === 'nominal' ? 'bg-emerald-400' : getLevelStatus(waterIce, 'waterice') === 'caution' ? 'bg-amber-400' : 'bg-red-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {waterIce.toFixed(1)}%
          </div>
          <div className="text-xs text-slate-500">detected</div>
        </div>
        
        <div className={`text-xs px-2 py-1 rounded-full border ${getStatusColor(getLevelStatus(waterIce, 'waterice'))} border-white/10 bg-white/5 inline-block`}>
          {getLevelStatus(waterIce, 'waterice').toUpperCase()}
        </div>
        
        <div className="text-xs text-slate-400 mt-3">
          ISRU Potential: {waterIce > 2 ? 'High' : waterIce > 1 ? 'Moderate' : 'Low'}
        </div>
      </div>
      
      {/* Electrostatic Charge (Lunar dust hazard) */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">ELECTROSTATIC CHARGE</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(electrostaticCharge, 'electrostatic') === 'nominal' ? 'bg-emerald-400' : getLevelStatus(electrostaticCharge, 'electrostatic') === 'caution' ? 'bg-amber-400' : 'bg-red-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {electrostaticCharge.toFixed(0)}
          </div>
          <div className="text-xs text-slate-500">kV/m</div>
        </div>
        
        <div className={`text-xs px-2 py-1 rounded-full border ${getStatusColor(getLevelStatus(electrostaticCharge, 'electrostatic'))} border-white/10 bg-white/5 inline-block`}>
          {getLevelStatus(electrostaticCharge, 'electrostatic').toUpperCase()}
        </div>
        
        <div className="text-xs text-slate-400 mt-3">
          Dust hazard level: {electrostaticCharge < 50 ? 'Manageable' : electrostaticCharge < 80 ? 'Moderate' : 'High'}
        </div>
      </div>
      
      {/* LiDAR Obstacle Density */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
          <h4 className="font-semibold text-slate-200 text-sm tracking-wide">LIDAR OBSTACLES</h4>
          <div className={`w-1.5 h-1.5 rounded-full ${getLevelStatus(lidarObstacleDensity, 'lidar') === 'clear' ? 'bg-emerald-400' : getLevelStatus(lidarObstacleDensity, 'lidar') === 'moderate' ? 'bg-amber-400' : 'bg-red-400'}`}></div>
        </div>
        
        <div className="flex items-baseline justify-between mb-3">
          <div className="text-3xl font-semibold text-slate-200">
            {lidarObstacleDensity.toFixed(1)}
          </div>
          <div className="text-xs text-slate-500">per m²</div>
        </div>
        
        <div className={`text-xs px-2 py-1 rounded-full border ${getStatusColor(getLevelStatus(lidarObstacleDensity, 'lidar'))} border-white/10 bg-white/5 inline-block`}>
          {getLevelStatus(lidarObstacleDensity, 'lidar').toUpperCase()}
        </div>
        
        <div className="text-xs text-slate-400 mt-3">
          Terrain: {lidarObstacleDensity < 1.0 ? 'Clear' : lidarObstacleDensity < 2.0 ? 'Moderate' : 'Rough'}
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;