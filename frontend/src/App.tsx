import React, { useState, useEffect } from 'react';
import SiteAnalysisPanel from './components/SiteAnalysisPanel';
import SensorDashboard from './components/SensorDashboard';
import TerrainMap from './components/TerrainMap';
import SiteComparison from './components/SiteComparison';
import { useWebSocket } from './hooks/useWebSocket';

const App: React.FC = () => {
  const { data, connected } = useWebSocket('ws://localhost:8765');
  
  const [sites, setSites] = useState<any[]>([]);
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0 });
  const [habitatData, setHabitatData] = useState<any>(null);

  useEffect(() => {
    if (data.sites) {
      setSites(data.sites.sites || []);
      if (data.sites.robot_position) {
        setRobotPosition(data.sites.robot_position);
      }
    }
    if (data.habitat) {
      setHabitatData(data.habitat);
    }
  }, [data]);

  return (
    <div className="flex flex-col h-screen bg-[#020617] text-gray-100 overflow-hidden font-mono">
      {/* Matrix-style background */}
      <div className="matrix-bg" id="matrix-bg"></div>
      
      {/* Top Status Bar */}
      <div className="bg-[#020617]/80 backdrop-blur-md border-b border-cyan-500/30 p-2 flex items-center justify-between text-xs">
        <div className="flex items-center space-x-6">
          <div className="flex items-center">
            <div className={`status-indicator ${connected ? 'status-active' : 'status-critical'}`}></div>
            <span className={`${connected ? 'text-green-400' : 'text-red-400'}`}>ROBOT CONNECTION: {connected ? 'ONLINE' : 'OFFLINE'}</span>
          </div>
          <div>POS: ({robotPosition.x.toFixed(2)}, {robotPosition.y.toFixed(2)})</div>
          <div>MISSION: LUNAR HABITAT SELECTION</div>
        </div>
        <div className="flex items-center space-x-4">
          <div className="flex items-center">
            <div className="status-indicator status-active"></div>
            <span className="text-green-400">SYSTEMS NOMINAL</span>
          </div>
          <div>TIME: {new Date().toLocaleTimeString()}</div>
        </div>
      </div>
      
      {/* Main Content Area */}
      <div className="flex flex-1 overflow-hidden relative">
        {/* Left Panel: Simulation View */}
        <div className="w-1/2 p-3 border-r border-cyan-500/20 flex flex-col">
          <div className="flex items-center justify-between mb-3 pb-2 border-b border-cyan-500/30">
            <h2 className="text-lg font-bold text-cyan-400 flex items-center">
              <span className="w-2 h-2 bg-cyan-500 rounded-full mr-2 animate-pulse"></span>
              LUNAR SIMULATION FEED
            </h2>
            <div className="text-xs text-gray-400">LIVE</div>
          </div>
          
          <div className="flex-1 bg-[#0f172a]/70 backdrop-blur border border-cyan-500/20 rounded-lg relative overflow-hidden scanning-line holographic-border">
            {/* Scanning grid lines */}
            <div className="absolute inset-0 opacity-10" style={{
              backgroundImage: `linear-gradient(rgba(6, 182, 212, 0.3) 1px, transparent 1px),
                                linear-gradient(90deg, rgba(6, 182, 212, 0.3) 1px, transparent 1px)`,
              backgroundSize: '40px 40px'
            }}></div>
            
            <div className="absolute inset-0 flex items-center justify-center z-10">
              <div className="text-center">
                <div className="w-20 h-20 mx-auto mb-4 rounded-full bg-gradient-to-r from-cyan-600 to-blue-600 flex items-center justify-center border-2 border-cyan-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-10 w-10 text-white" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                  </svg>
                </div>
                <div className="text-cyan-400 text-xl font-bold mb-2">LUNAR OPERATIONS</div>
                <div className="text-gray-400 text-sm">Real-time telemetry from LunaBot</div>
                
                <div className="mt-8 grid grid-cols-2 gap-4 max-w-xs mx-auto">
                  <div className="bg-[#0f172a]/80 p-3 rounded border border-cyan-500/30">
                    <div className="text-xs text-cyan-400">X COORD</div>
                    <div className="text-lg font-mono text-cyan-300">{robotPosition.x.toFixed(2)}</div>
                  </div>
                  <div className="bg-[#0f172a]/80 p-3 rounded border border-cyan-500/30">
                    <div className="text-xs text-cyan-400">Y COORD</div>
                    <div className="text-lg font-mono text-cyan-300">{robotPosition.y.toFixed(2)}</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
          
          {/* Status Panel */}
          <div className="mt-3 grid grid-cols-4 gap-2">
            <div className="bg-[#0f172a]/70 border border-cyan-500/20 rounded p-2 text-center">
              <div className="text-xs text-gray-400">POWER</div>
              <div className="text-sm font-mono text-green-400">98%</div>
            </div>
            <div className="bg-[#0f172a]/70 border border-cyan-500/20 rounded p-2 text-center">
              <div className="text-xs text-gray-400">SIGNAL</div>
              <div className="text-sm font-mono text-green-400">STRONG</div>
            </div>
            <div className="bg-[#0f172a]/70 border border-cyan-500/20 rounded p-2 text-center">
              <div className="text-xs text-gray-400">SPEED</div>
              <div className="text-sm font-mono text-yellow-400">1.2 m/s</div>
            </div>
            <div className="bg-[#0f172a]/70 border border-cyan-500/20 rounded p-2 text-center">
              <div className="text-xs text-gray-400">STATUS</div>
              <div className="text-sm font-mono text-green-400">ACTIVE</div>
            </div>
          </div>
        </div>
        
        {/* Right Panel: Dashboard */}
        <div className="w-1/2 flex flex-col p-3 overflow-y-auto">
          <div className="mb-4 text-center">
            <h1 className="text-2xl font-bold text-transparent bg-clip-text bg-gradient-to-r from-cyan-400 to-blue-500 tracking-wider">
              LUNABOT MISSION CONTROL
            </h1>
            <div className="text-sm text-gray-400">LUNAR HABITAT SELECTION DASHBOARD</div>
          </div>
          
          <div className="space-y-4 overflow-y-auto">
            {/* Site Analysis Panel */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3">
                <div className="w-3 h-3 bg-green-500 rounded-full mr-2 animate-pulse"></div>
                <h3 className="text-lg font-bold text-green-400">HABITAT SITE ANALYSIS</h3>
              </div>
              <SiteAnalysisPanel sites={sites} />
            </div>
            
            {/* Sensor Dashboard */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3">
                <div className="w-3 h-3 bg-yellow-500 rounded-full mr-2 animate-pulse"></div>
                <h3 className="text-lg font-bold text-yellow-400">ENVIRONMENTAL SENSORS</h3>
              </div>
              <SensorDashboard habitatData={habitatData} />
            </div>
            
            {/* Site Comparison */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3">
                <div className="w-3 h-3 bg-purple-500 rounded-full mr-2 animate-pulse"></div>
                <h3 className="text-lg font-bold text-purple-400">SITE COMPARISON MATRIX</h3>
              </div>
              <SiteComparison sites={sites} />
            </div>
            
            {/* 3D Terrain Visualization */}
            <div className="data-panel holographic-border">
              <div className="flex items-center mb-3">
                <div className="w-3 h-3 bg-red-500 rounded-full mr-2 animate-pulse"></div>
                <h3 className="text-lg font-bold text-red-400">TERRAIN ANALYSIS MAP</h3>
              </div>
              <TerrainMap sites={sites} robotPosition={robotPosition} />
            </div>
          </div>
        </div>
      </div>
      
      {/* Bottom Status Bar */}
      <div className="bg-[#020617]/80 backdrop-blur-md border-t border-cyan-500/30 p-2 text-xs flex justify-between">
        <div>LUNABOT v2.1.0 | LUNAR HABITAT MISSION</div>
        <div>STATUS: <span className="text-green-400">OPERATIONAL</span></div>
        <div>DATA STREAM: {sites.length} SITES ANALYZED</div>
      </div>
    </div>
  );
};

export default App;