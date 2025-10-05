import React from 'react';
import { LineChart, Line, AreaChart, Area, BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Cell, PieChart, Pie } from 'recharts';
import { SensorData } from '../utils/mockDataService';

interface AllSensorsPanelProps {
  sensorData: SensorData;
  historicalData: Array<{time: string; radiation: number; temperature: number; seismic: number;}>;
}

const AllSensorsPanel: React.FC<AllSensorsPanelProps> = ({ sensorData, historicalData }) => {
  // Regolith composition for pie chart
  const regolithData = Object.entries(sensorData.regolith.composition).map(([name, value]) => ({
    name: name.charAt(0).toUpperCase() + name.slice(1),
    value: value
  }));

  const COLORS = ['#ef4444', '#f59e0b', '#3b82f6', '#8b5cf6', '#22c55e', '#06b6d4'];

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-4">
      {/* Radiation Sensor */}
      <div className="bg-gradient-to-br from-slate-900/60 to-slate-800/40 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1.5 h-1.5 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">RADIATION (TEPC)</h4>
          </div>
          <span className={`text-xs px-2 py-0.5 rounded-full border ${sensorData.radiation.status === 'optimal' ? 'bg-emerald-500/10 text-emerald-400 border-emerald-500/20' : 'bg-amber-500/10 text-amber-400 border-amber-500/20'}`}>
            {sensorData.radiation.status.toUpperCase()}
          </span>
        </div>
        
        <div className="grid grid-cols-2 gap-2 mb-3">
          <div className="bg-black/20 border border-white/5 rounded p-2">
            <div className="text-[10px] text-slate-400 font-medium">COSMIC</div>
            <div className="text-lg font-semibold text-slate-200">{sensorData.radiation.cosmic.toFixed(2)}</div>
            <div className="text-[10px] text-slate-500">mSv/h</div>
          </div>
          <div className="bg-black/20 border border-white/5 rounded p-2">
            <div className="text-[10px] text-slate-400 font-medium">SOLAR</div>
            <div className="text-lg font-semibold text-slate-200">{sensorData.radiation.solar.toFixed(2)}</div>
            <div className="text-[10px] text-slate-500">mSv/h</div>
          </div>
        </div>
        
        <div className="bg-blue-500/5 border border-blue-500/20 rounded p-2">
          <div className="text-[10px] text-slate-400 mb-1">TOTAL EXPOSURE</div>
          <div className="text-2xl font-semibold text-blue-300">{sensorData.radiation.total.toFixed(2)} <span className="text-sm text-slate-400">mSv/h</span></div>
        </div>
        
        <div className="h-20 mt-3">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={historicalData}>
              <defs>
                <linearGradient id="colorRad" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#ef4444" stopOpacity={0.6}/>
                  <stop offset="95%" stopColor="#7f1d1d" stopOpacity={0.1}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#6b7280" tick={{ fontSize: 9 }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 9 }} domain={[0, 1]} />
              <Area type="monotone" dataKey="radiation" stroke="#ef4444" strokeWidth={2} fillOpacity={0.4} fill="url(#colorRad)" />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* Seismic Sensor */}
      <div className="bg-gradient-to-br from-slate-900/60 to-slate-800/40 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1.5 h-1.5 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">SEISMIC SENSOR</h4>
          </div>
          <span className={`text-xs px-2 py-0.5 rounded-full border ${sensorData.seismic.status === 'stable' ? 'bg-emerald-500/10 text-emerald-400 border-emerald-500/20' : 'bg-red-500/10 text-red-400 border-red-500/20'}`}>
            {sensorData.seismic.status.toUpperCase()}
          </span>
        </div>
        
        <div className="grid grid-cols-2 gap-3 mb-3">
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">FREQUENCY</div>
            <div className="text-lg font-bold text-yellow-300">{sensorData.seismic.frequency.toFixed(1)}</div>
            <div className="text-[10px] text-gray-500">Hz</div>
          </div>
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">MAGNITUDE</div>
            <div className="text-lg font-bold text-orange-300">{sensorData.seismic.magnitude.toFixed(1)}</div>
            <div className="text-[10px] text-gray-500">M<sub>L</sub></div>
          </div>
        </div>
        
        <div className="mb-2">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">STABILITY</span>
            <span className="font-bold text-green-400">{sensorData.seismic.stability.toFixed(0)}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-3 border border-yellow-500/30">
            <div 
              className="bg-gradient-to-r from-green-500 to-yellow-500 h-3 rounded-full transition-all duration-500" 
              style={{ width: `${sensorData.seismic.stability}%` }}
            ></div>
          </div>
        </div>
        
        <div className="bg-yellow-900/20 rounded p-2 border border-yellow-500/20 text-xs">
          <span className="text-gray-400">Last Quake:</span> <span className="text-yellow-300 font-mono">{sensorData.seismic.lastQuake}</span>
        </div>
        
        <div className="h-20 mt-3">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={historicalData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#6b7280" tick={{ fontSize: 9 }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 9 }} domain={[0, 5]} />
              <Line type="monotone" dataKey="seismic" stroke="#eab308" strokeWidth={2} dot={false} />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* Thermal Sensor */}
      <div className="bg-[#0f172a]/80 border border-blue-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-blue-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-blue-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-blue-400 text-sm">THERMAL SENSOR</h4>
          </div>
          <span className={`text-xs px-2 py-1 rounded ${sensorData.thermal.status === 'nominal' ? 'bg-green-900/50 text-green-400' : 'bg-yellow-900/50 text-yellow-400'}`}>
            {sensorData.thermal.status.toUpperCase()}
          </span>
        </div>
        
        <div className="grid grid-cols-2 gap-3 mb-3">
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">SURFACE</div>
            <div className="text-lg font-bold text-blue-300">{sensorData.thermal.surface.toFixed(1)}°C</div>
          </div>
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">SUBSURFACE</div>
            <div className="text-lg font-bold text-cyan-300">{sensorData.thermal.subsurface.toFixed(1)}°C</div>
          </div>
        </div>
        
        <div className="bg-blue-900/20 rounded p-2 border border-blue-500/20">
          <div className="text-[10px] text-gray-400 mb-1">THERMAL GRADIENT</div>
          <div className="text-xl font-bold text-blue-400">{sensorData.thermal.gradient.toFixed(1)}°C</div>
        </div>
        
        <div className="h-20 mt-3">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={historicalData}>
              <defs>
                <linearGradient id="colorTemp" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#3b82f6" stopOpacity={0.6}/>
                  <stop offset="95%" stopColor="#1e3a8a" stopOpacity={0.1}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
              <XAxis dataKey="time" stroke="#6b7280" tick={{ fontSize: 9 }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 9 }} domain={[-60, 20]} />
              <Area type="monotone" dataKey="temperature" stroke="#3b82f6" strokeWidth={2} fillOpacity={0.4} fill="url(#colorTemp)" />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </div>

      {/* Regolith Analyzer (XRF) */}
      <div className="bg-[#0f172a]/80 border border-purple-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-purple-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-purple-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-purple-400 text-sm">REGOLITH ANALYZER (XRF)</h4>
          </div>
          <span className="text-xs px-2 py-1 rounded bg-purple-900/50 text-purple-300">ACTIVE</span>
        </div>
        
        <div className="grid grid-cols-2 gap-2 mb-3 text-xs">
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Fe</span>
            <span className="font-bold text-red-300">{sensorData.regolith.composition.iron}%</span>
          </div>
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Ti</span>
            <span className="font-bold text-gray-300">{sensorData.regolith.composition.titanium}%</span>
          </div>
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Si</span>
            <span className="font-bold text-yellow-300">{sensorData.regolith.composition.silicon}%</span>
          </div>
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Al</span>
            <span className="font-bold text-blue-300">{sensorData.regolith.composition.aluminum}%</span>
          </div>
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Ca</span>
            <span className="font-bold text-orange-300">{sensorData.regolith.composition.calcium}%</span>
          </div>
          <div className="flex justify-between bg-[#020617]/60 rounded px-2 py-1">
            <span className="text-gray-400">Mg</span>
            <span className="font-bold text-green-300">{sensorData.regolith.composition.magnesium}%</span>
          </div>
        </div>
        
        <div className="grid grid-cols-2 gap-2 mb-2">
          <div className="bg-purple-900/20 rounded p-2 border border-purple-500/20">
            <div className="text-[10px] text-gray-400">DENSITY</div>
            <div className="text-sm font-bold text-purple-300">{sensorData.regolith.density} g/cm³</div>
          </div>
          <div className="bg-cyan-900/20 rounded p-2 border border-cyan-500/20">
            <div className="text-[10px] text-gray-400">H₂O ICE</div>
            <div className="text-sm font-bold text-cyan-300">{sensorData.regolith.waterIce.toFixed(2)}%</div>
          </div>
        </div>
        
        <div>
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">ISRU POTENTIAL</span>
            <span className="font-bold text-purple-400">{sensorData.regolith.isruPotential}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-2 border border-purple-500/30">
            <div 
              className="bg-gradient-to-r from-purple-600 to-purple-400 h-2 rounded-full" 
              style={{ width: `${sensorData.regolith.isruPotential}%` }}
            ></div>
          </div>
        </div>
      </div>

      {/* LiDAR Sensor */}
      <div className="bg-[#0f172a]/80 border border-green-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-green-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-green-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-green-400 text-sm">LIDAR 3D MAPPING</h4>
          </div>
          <span className="text-xs px-2 py-1 rounded bg-green-900/50 text-green-300">SCANNING</span>
        </div>
        
        <div className="grid grid-cols-2 gap-3 mb-3">
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">RANGE</div>
            <div className="text-lg font-bold text-green-300">{sensorData.lidar.range}</div>
            <div className="text-[10px] text-gray-500">meters</div>
          </div>
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">POINT RATE</div>
            <div className="text-lg font-bold text-green-300">{(sensorData.lidar.pointsPerSecond / 1000).toFixed(0)}K</div>
            <div className="text-[10px] text-gray-500">pts/sec</div>
          </div>
        </div>
        
        <div className="bg-green-900/20 rounded p-2 border border-green-500/20 mb-2">
          <div className="text-[10px] text-gray-400">OBSTACLES DETECTED</div>
          <div className="text-2xl font-bold text-green-400">{sensorData.lidar.obstaclesDetected}</div>
        </div>
        
        <div>
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">TERRAIN ROUGHNESS</span>
            <span className="font-bold text-yellow-400">{sensorData.lidar.terrainRoughness}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-2 border border-green-500/30">
            <div 
              className="bg-gradient-to-r from-green-500 to-yellow-500 h-2 rounded-full" 
              style={{ width: `${sensorData.lidar.terrainRoughness}%` }}
            ></div>
          </div>
        </div>
        
        {/* Simulated LiDAR visualization */}
        <div className="mt-3 h-16 bg-[#020617]/80 rounded border border-green-500/20 relative overflow-hidden">
          {[...Array(20)].map((_, i) => (
            <div 
              key={i}
              className="absolute bottom-0 bg-green-500/30 border-t border-green-500" 
              style={{
                left: `${i * 5}%`,
                width: '3px',
                height: `${20 + Math.random() * 60}%`,
                animation: `pulse ${1 + Math.random()}s infinite`
              }}
            ></div>
          ))}
        </div>
      </div>

      {/* Spectrometer */}
      <div className="bg-[#0f172a]/80 border border-cyan-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-cyan-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-cyan-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-cyan-400 text-sm">NIR SPECTROMETER</h4>
          </div>
          <span className="text-xs px-2 py-1 rounded bg-cyan-900/50 text-cyan-300">ANALYZING</span>
        </div>
        
        <div className="mb-3">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">WATER SIGNATURE</span>
            <span className="font-bold text-cyan-400">{sensorData.spectrometer.waterSignature.toFixed(0)}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-3 border border-cyan-500/30">
            <div 
              className="bg-gradient-to-r from-blue-500 to-cyan-400 h-3 rounded-full flex items-center justify-center" 
              style={{ width: `${sensorData.spectrometer.waterSignature}%` }}
            >
              {sensorData.spectrometer.waterSignature > 30 && (
                <span className="text-[8px] text-white font-bold">{sensorData.spectrometer.waterSignature.toFixed(0)}%</span>
              )}
            </div>
          </div>
        </div>
        
        <div className="mb-3">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">VOLATILE DETECTION</span>
            <span className="font-bold text-purple-400">{sensorData.spectrometer.volatileDetection.toFixed(0)}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-3 border border-purple-500/30">
            <div 
              className="bg-gradient-to-r from-purple-500 to-pink-400 h-3 rounded-full" 
              style={{ width: `${sensorData.spectrometer.volatileDetection}%` }}
            ></div>
          </div>
        </div>
        
        <div className="bg-cyan-900/20 rounded p-3 border border-cyan-500/20">
          <div className="text-[10px] text-gray-400 mb-1">MINERAL DETECTED</div>
          <div className="text-sm font-bold text-cyan-300">{sensorData.spectrometer.mineralSignature}</div>
        </div>
        
        {/* Spectral bars visualization */}
        <div className="mt-3 h-16 bg-[#020617]/80 rounded border border-cyan-500/20 flex items-end justify-around p-2">
          {[45, 68, 32, 89, 56, 72, 41, 63, 85, 38].map((height, i) => (
            <div 
              key={i}
              className="bg-gradient-to-t from-cyan-500 to-blue-400 rounded-t" 
              style={{
                width: '6%',
                height: `${height}%`,
                opacity: 0.7
              }}
            ></div>
          ))}
        </div>
      </div>

      {/* Cameras */}
      <div className="bg-[#0f172a]/80 border border-orange-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-orange-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-orange-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-orange-400 text-sm">STEREO CAMERAS</h4>
          </div>
          <span className="text-xs px-2 py-1 rounded bg-green-900/50 text-green-300">ONLINE</span>
        </div>
        
        <div className="grid grid-cols-2 gap-3 mb-3">
          <div className="bg-[#020617]/60 rounded p-2 border border-green-500/20">
            <div className="text-[10px] text-gray-400">FRONT CAM</div>
            <div className="text-sm font-bold text-green-300">{sensorData.cameras.frontCamera.status}</div>
            <div className="text-[10px] text-gray-500">{sensorData.cameras.frontCamera.resolution}</div>
          </div>
          <div className="bg-[#020617]/60 rounded p-2 border border-green-500/20">
            <div className="text-[10px] text-gray-400">REAR CAM</div>
            <div className="text-sm font-bold text-green-300">{sensorData.cameras.rearCamera.status}</div>
            <div className="text-[10px] text-gray-500">{sensorData.cameras.rearCamera.resolution}</div>
          </div>
        </div>
        
        <div className="mb-2">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">VISIBILITY</span>
            <span className="font-bold text-green-400">{sensorData.cameras.visibility}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-2 border border-orange-500/30">
            <div 
              className="bg-gradient-to-r from-green-500 to-green-400 h-2 rounded-full" 
              style={{ width: `${sensorData.cameras.visibility}%` }}
            ></div>
          </div>
        </div>
        
        <div className="mb-2">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">DUST LEVEL</span>
            <span className="font-bold text-yellow-400">{sensorData.cameras.dustLevel}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-2 border border-orange-500/30">
            <div 
              className="bg-gradient-to-r from-yellow-500 to-orange-500 h-2 rounded-full" 
              style={{ width: `${sensorData.cameras.dustLevel}%` }}
            ></div>
          </div>
        </div>
        
        {/* Simulated camera view */}
        <div className="mt-3 h-16 bg-[#020617]/80 rounded border border-orange-500/20 relative overflow-hidden flex items-center justify-center">
          <div className="absolute inset-0 bg-gradient-to-b from-orange-500/10 to-transparent"></div>
          <svg className="w-12 h-12 text-orange-500/40" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
          </svg>
        </div>
      </div>

      {/* Environmental Sensors */}
      <div className="bg-[#0f172a]/80 border border-indigo-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-indigo-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-indigo-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-indigo-400 text-sm">ENVIRONMENTAL</h4>
          </div>
          <span className="text-xs px-2 py-1 rounded bg-indigo-900/50 text-indigo-300">MONITORING</span>
        </div>
        
        <div className="space-y-2">
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="flex justify-between items-center mb-1">
              <span className="text-[10px] text-gray-400">SOLAR EXPOSURE</span>
              <span className="text-xs font-bold text-yellow-400">{sensorData.environmental.solarExposure}%</span>
            </div>
            <div className="w-full bg-[#020617] rounded-full h-1.5 border border-yellow-500/30">
              <div 
                className="bg-gradient-to-r from-yellow-500 to-orange-400 h-1.5 rounded-full" 
                style={{ width: `${sensorData.environmental.solarExposure}%` }}
              ></div>
            </div>
          </div>
          
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">UV RADIATION</span>
              <span className="font-bold text-purple-400">{sensorData.environmental.uvRadiation} W/m²</span>
            </div>
          </div>
          
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">ELECTROSTATIC</span>
              <span className="font-bold text-blue-400">{sensorData.environmental.electrostaticCharge.toFixed(1)} kV/m</span>
            </div>
          </div>
          
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="flex justify-between text-xs">
              <span className="text-gray-400">DUST PARTICLES</span>
              <span className="font-bold text-gray-400">{sensorData.environmental.dustParticles} /m³</span>
            </div>
          </div>
        </div>
      </div>

      {/* Power System */}
      <div className="bg-[#0f172a]/80 border border-emerald-500/30 rounded-lg p-4 scanning-line">
        <div className="flex items-center justify-between mb-3 pb-2 border-b border-emerald-500/30">
          <div className="flex items-center">
            <div className="w-2 h-2 bg-emerald-500 rounded-full mr-2 animate-pulse"></div>
            <h4 className="font-bold text-emerald-400 text-sm">POWER SYSTEM</h4>
          </div>
          <span className={`text-xs px-2 py-1 rounded ${sensorData.power.status === 'optimal' ? 'bg-green-900/50 text-green-400' : 'bg-yellow-900/50 text-yellow-400'}`}>
            {sensorData.power.status.toUpperCase()}
          </span>
        </div>
        
        <div className="grid grid-cols-2 gap-3 mb-3">
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">SOLAR</div>
            <div className="text-lg font-bold text-yellow-300">{sensorData.power.solar.toFixed(0)}W</div>
          </div>
          <div className="bg-[#020617]/60 rounded p-2">
            <div className="text-[10px] text-gray-400">RTG</div>
            <div className="text-lg font-bold text-orange-300">{sensorData.power.rtg}W</div>
          </div>
        </div>
        
        <div className="mb-2">
          <div className="flex justify-between text-xs mb-1">
            <span className="text-gray-400">BATTERY</span>
            <span className="font-bold text-green-400">{sensorData.power.battery.toFixed(1)}%</span>
          </div>
          <div className="w-full bg-[#020617] rounded-full h-3 border border-emerald-500/30">
            <div 
              className="bg-gradient-to-r from-emerald-500 to-green-400 h-3 rounded-full flex items-center justify-center" 
              style={{ width: `${sensorData.power.battery}%` }}
            >
              <span className="text-[8px] text-white font-bold">{sensorData.power.battery.toFixed(0)}%</span>
            </div>
          </div>
        </div>
        
        <div className="bg-emerald-900/20 rounded p-2 border border-emerald-500/20">
          <div className="text-[10px] text-gray-400">CONSUMPTION</div>
          <div className="text-xl font-bold text-emerald-300">{sensorData.power.consumption}W</div>
        </div>
      </div>
    </div>
  );
};

export default AllSensorsPanel;
