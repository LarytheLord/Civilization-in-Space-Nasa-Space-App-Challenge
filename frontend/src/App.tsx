import React, { useState, useEffect } from 'react';
import SiteAnalysisPanel from './components/SiteAnalysisPanel';
import SensorDashboard from './components/SensorDashboard';
import TerrainMap from './components/TerrainMap';
import SiteComparison from './components/SiteComparison';
import AllSensorsPanel from './components/AllSensorsPanel';
import MissionControlPanel from './components/MissionControlPanel';
import DecisionLayerPanel from './components/DecisionLayerPanel';
import mockDataService, { SensorData, Site, MissionLog } from './utils/mockDataService';

const App: React.FC = () => {
  const [sites, setSites] = useState<Site[]>([]);
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0 });
  const [sensorData, setSensorData] = useState<SensorData | null>(null);
  const [missionLogs, setMissionLogs] = useState<MissionLog[]>([]);
  const [historicalData, setHistoricalData] = useState<Array<{time: string; radiation: number; temperature: number; seismic: number}>>([]);
  const [activeTab, setActiveTab] = useState<'overview' | 'sensors' | 'sites' | 'decision'>('overview');
  const [currentTime, setCurrentTime] = useState(new Date());

  useEffect(() => {
    // Start mock data simulation
    mockDataService.startSimulation((data) => {
      setSites(data.sites);
      setSensorData(data.sensors);
      setMissionLogs(data.logs);
      setRobotPosition({
        x: data.sensors.navigation.position.x,
        y: data.sensors.navigation.position.y
      });
      
      // Update historical data for charts
      setHistoricalData(prev => {
        const newData = [...prev, {
          time: new Date().toLocaleTimeString(),
          radiation: data.sensors.radiation.total,
          temperature: data.sensors.thermal.surface,
          seismic: data.sensors.seismic.magnitude
        }];
        // Keep only last 20 data points
        return newData.slice(-20);
      });
    });
    
    // Update clock
    const clockInterval = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);
    
    return () => {
      mockDataService.stopSimulation();
      clearInterval(clockInterval);
    };
  }, []);

  if (!sensorData) {
    return (
      <div className="flex items-center justify-center h-screen bg-[#020617] text-gray-100">
        <div className="text-center">
          <div className="w-16 h-16 border-4 border-cyan-500 border-t-transparent rounded-full animate-spin mx-auto mb-4"></div>
          <div className="text-xl font-bold text-cyan-400">INITIALIZING LUNABOT SYSTEMS...</div>
        </div>
      </div>
    );
  }

  const connected = true; // Always show as connected with mock data
  const habitatData = {
    environmental_parameters: {
      temperature: sensorData.thermal.surface,
      radiation: sensorData.radiation.total * 100,
      co2: 0.04,
      o2: 20.9
    }
  };

  return (
    <div className="flex flex-col h-screen bg-gradient-to-br from-black via-[#0a0e1a] to-[#0f1419] text-gray-100 overflow-hidden font-mono">
      {/* Subtle background grid */}
      <div className="absolute inset-0 opacity-30 pointer-events-none" style={{
        backgroundImage: 'linear-gradient(rgba(59, 130, 246, 0.03) 1px, transparent 1px), linear-gradient(90deg, rgba(59, 130, 246, 0.03) 1px, transparent 1px)',
        backgroundSize: '50px 50px'
      }}></div>
      
      {/* Top Status Bar */}
      <div className="relative bg-gradient-to-r from-black/90 via-[#0a0e1a]/90 to-black/90 backdrop-blur-md border-b border-white/10 p-2 flex items-center justify-between text-xs z-10 shadow-lg">
        <div className="flex items-center space-x-6">
          <div className="flex items-center">
            <div className="w-2 h-2 rounded-full bg-emerald-400 mr-2 animate-pulse shadow-lg shadow-emerald-400/50"></div>
            <span className="text-emerald-400 font-semibold">LUNABOT STATUS: OPERATIONAL</span>
          </div>
          <div className="text-blue-300">POS: ({robotPosition.x.toFixed(2)}, {robotPosition.y.toFixed(2)})</div>
          <div className="text-slate-300">MISSION: LUNAR HABITAT SELECTION</div>
          <div className="text-amber-400">SITES ANALYZED: {sites.length}</div>
        </div>
        <div className="flex items-center space-x-4">
          <div className="flex items-center">
            <div className="w-2 h-2 rounded-full bg-emerald-400 mr-2 animate-pulse shadow-lg shadow-emerald-400/50"></div>
            <span className="text-emerald-400 font-semibold">ALL SYSTEMS NOMINAL</span>
          </div>
          <div className="text-slate-400 font-mono">
            {currentTime.toLocaleDateString()} | {currentTime.toLocaleTimeString()}
          </div>
        </div>
      </div>
      
      {/* Main Header */}
      <div className="relative px-6 py-4 border-b border-white/10 bg-gradient-to-b from-[#0a0e1a]/50 to-transparent backdrop-blur-sm">
        <div className="text-center mb-3">
          <h1 className="text-3xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-blue-400 via-blue-300 to-slate-300 tracking-wide drop-shadow-lg" style={{
            textShadow: '0 0 30px rgba(59, 130, 246, 0.3)'
          }}>
            LUNABOT MISSION CONTROL
          </h1>
          <div className="text-sm text-slate-400 font-light">Autonomous Lunar Habitat Site Selection System</div>
        </div>
        
        {/* Navigation Tabs */}
        <div className="flex justify-center space-x-2">
            {[
              { id: 'overview', label: 'MISSION OVERVIEW' },
              { id: 'sensors', label: 'ALL SENSORS' },
              { id: 'sites', label: 'SITE ANALYSIS' },
              { id: 'decision', label: 'AI DECISION LAYER' }
            ].map(tab => (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id as any)}
                className={`px-4 py-2 rounded text-xs font-semibold tracking-wide transition-all duration-200 ${
                  activeTab === tab.id
                    ? 'bg-blue-600/80 text-white border border-blue-400/30'
                    : 'bg-white/5 text-slate-400 border border-white/10 hover:border-white/20 hover:bg-white/10 hover:text-slate-200'
                }`}
              >
                {tab.label}
                </button>
            ))}
        </div>
      </div>
      
      {/* Main Content Area - Tabbed */}
      <div className="flex-1 overflow-hidden p-4">
        {activeTab === 'overview' && (
          <div className="h-full grid grid-rows-[auto_1fr] gap-4 overflow-y-auto">
            {/* Top Row - Mission Control & Terrain Map */}
            <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
            {/* Quick Stats */}
            <div className="grid grid-cols-2 gap-3">
                <div className="relative bg-gradient-to-br from-slate-900/60 to-slate-800/40 backdrop-blur-md border border-white/10 rounded-lg p-3 text-center overflow-hidden group hover:border-white/20 transition-all">
                  <div className="relative">
                    <div className="text-xs text-slate-400 font-medium mb-1">POWER</div>
                    <div className="text-2xl font-semibold text-slate-200">{sensorData.power.battery.toFixed(0)}%</div>
                  </div>
                </div>
                <div className="relative bg-gradient-to-br from-slate-900/60 to-slate-800/40 backdrop-blur-md border border-white/10 rounded-lg p-3 text-center overflow-hidden group hover:border-white/20 transition-all">
                  <div className="relative">
                    <div className="text-xs text-slate-400 font-medium mb-1">SPEED</div>
                    <div className="text-2xl font-semibold text-slate-200">{sensorData.navigation.speed.toFixed(1)}</div>
                    <div className="text-[10px] text-slate-500">m/s</div>
                  </div>
                </div>
                <div className="relative bg-gradient-to-br from-slate-900/60 to-slate-800/40 backdrop-blur-md border border-white/10 rounded-lg p-3 text-center overflow-hidden group hover:border-white/20 transition-all">
                  <div className="relative">
                    <div className="text-xs text-slate-400 font-medium mb-1">RADIATION</div>
                    <div className="text-2xl font-semibold text-slate-200">{sensorData.radiation.total.toFixed(2)}</div>
                    <div className="text-[10px] text-slate-500">mSv/h</div>
                  </div>
                </div>
                <div className="relative bg-gradient-to-br from-blue-950/30 to-slate-800/40 backdrop-blur-md border border-blue-500/20 rounded-lg p-3 text-center overflow-hidden group hover:border-blue-400/30 transition-all">
                  <div className="relative">
                    <div className="text-xs text-blue-400 font-medium mb-1">BEST SITE</div>
                    <div className="text-2xl font-semibold text-blue-300">{sites[0]?.total_score.toFixed(1)}</div>
                  </div>
                </div>
            </div>
              
              {/* Terrain Map */}
              <div className="lg:col-span-2">
                <div className="data-panel holographic-border h-full">
                  <div className="flex items-center mb-3 pb-2 border-b border-white/10">
                    <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
                    <h3 className="text-sm font-semibold text-slate-200 tracking-wide">TERRAIN MAP</h3>
                  </div>
                  <TerrainMap sites={sites} robotPosition={robotPosition} />
                </div>
              </div>
            </div>
            
            {/* Mission Control Panel */}
            <MissionControlPanel logs={missionLogs} navigationData={sensorData.navigation} />
          </div>
        )}
        
        {activeTab === 'sensors' && (
          <div className="h-full overflow-y-auto">
            <AllSensorsPanel sensorData={sensorData} historicalData={historicalData} />
          </div>
        )}
        
        {activeTab === 'sites' && (
          <div className="h-full overflow-y-auto space-y-4">
            {/* Site Analysis Panel */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3 pb-2 border-b border-white/10">
                <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
                <h3 className="text-base font-semibold text-slate-200 tracking-wide">HABITAT SITE ANALYSIS</h3>
              </div>
              <SiteAnalysisPanel sites={sites} />
            </div>
            
            {/* Site Comparison */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3 pb-2 border-b border-white/10">
                <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
                <h3 className="text-base font-semibold text-slate-200 tracking-wide">SITE COMPARISON MATRIX</h3>
              </div>
              <SiteComparison sites={sites} />
            </div>
            
            {/* Environmental Sensors */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3 pb-2 border-b border-white/10">
                <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
                <h3 className="text-base font-semibold text-slate-200 tracking-wide">ENVIRONMENTAL SENSORS</h3>
              </div>
              <SensorDashboard habitatData={habitatData} />
            </div>
          </div>
        )}
        
        {activeTab === 'decision' && (
          <div className="h-full overflow-y-auto">
            <DecisionLayerPanel currentSite={sites[0]} />
          </div>
        )}
      </div>
      
      {/* Bottom Status Bar */}
      <div className="relative bg-gradient-to-r from-black/90 via-[#0a0e1a]/90 to-black/90 backdrop-blur-md border-t border-white/10 p-2 text-xs flex justify-between items-center z-10 shadow-lg">
        <div className="flex items-center space-x-6">
          <div className="flex items-center">
            <span className="text-blue-400 font-bold">LUNABOT v2.1.0</span>
          </div>
          <div className="text-slate-400">NASA SPACE APPS CHALLENGE 2025</div>
          <div>MISSION: <span className="text-slate-300">LUNAR HABITAT SELECTION</span></div>
        </div>
        <div className="flex items-center space-x-4">
          <div>SITES: <span className="text-emerald-400 font-semibold">{sites.length} ANALYZED</span></div>
          <div className="flex items-center">
            <span className="text-slate-400 mr-2">STATUS:</span>
            <span className="text-emerald-400 font-semibold flex items-center">
              <span className="w-2 h-2 rounded-full bg-emerald-400 mr-2 animate-pulse shadow-lg shadow-emerald-400/50"></span>
              OPERATIONAL
            </span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default App;