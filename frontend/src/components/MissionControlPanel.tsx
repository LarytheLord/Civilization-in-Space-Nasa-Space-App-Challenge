import React from 'react';
import { MissionLog } from '../utils/mockDataService';

interface MissionControlPanelProps {
  logs: MissionLog[];
  navigationData: {
    position: { x: number; y: number; z: number };
    heading: number;
    speed: number;
    distance: number;
  };
}

const MissionControlPanel: React.FC<MissionControlPanelProps> = ({ logs, navigationData }) => {
  const formatTime = (isoString: string) => {
    const date = new Date(isoString);
    return date.toLocaleTimeString();
  };

  const getLogIcon = (type: MissionLog['type']) => {
    switch (type) {
      case 'success':
        return (
          <svg className="w-4 h-4 text-emerald-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
        );
      case 'warning':
        return (
          <svg className="w-4 h-4 text-amber-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
          </svg>
        );
      case 'error':
        return (
          <svg className="w-4 h-4 text-red-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 14l2-2m0 0l2-2m-2 2l-2-2m2 2l2 2m7-2a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
        );
      default:
        return (
          <svg className="w-4 h-4 text-slate-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
        );
    }
  };

  const getLogColor = (type: MissionLog['type']) => {
    switch (type) {
      case 'success': return 'border-l-emerald-500 bg-emerald-900/10';
      case 'warning': return 'border-l-amber-500 bg-amber-900/10';
      case 'error': return 'border-l-red-500 bg-red-900/10';
      default: return 'border-l-slate-500 bg-slate-900/10';
    }
  };

  return (
    <div className="grid grid-cols-1 lg:grid-cols-3 gap-4 h-full">
      {/* Navigation Status - PROFESSIONAL MONOCHROME */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex items-center justify-between mb-4 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">NAVIGATION STATUS</h4>
          </div>
        </div>
        
        <div className="space-y-3">
          <div className="bg-black/20 rounded-lg p-3 border border-white/10">
            <div className="text-[10px] text-slate-400 mb-1">POSITION (LAT, LON, ALT)</div>
            <div className="text-lg font-mono font-semibold text-slate-200">
              {navigationData.position.x.toFixed(1)}, {navigationData.position.y.toFixed(1)}, {navigationData.position.z.toFixed(1)}
            </div>
          </div>
          
          <div className="grid grid-cols-2 gap-3">
            <div className="bg-black/20 border border-white/5 rounded p-2">
              <div className="text-[10px] text-slate-400">HEADING</div>
              <div className="text-xl font-semibold text-slate-200">{navigationData.heading}°</div>
            </div>
            <div className="bg-black/20 border border-white/5 rounded p-2">
              <div className="text-[10px] text-slate-400">SPEED</div>
              <div className="text-xl font-semibold text-slate-200">{navigationData.speed.toFixed(1)}</div>
              <div className="text-[8px] text-slate-500">m/s</div>
            </div>
          </div>
          
          <div className="bg-blue-500/5 border border-blue-500/20 rounded p-3">
            <div className="text-[10px] text-slate-400 mb-1">TOTAL DISTANCE</div>
            <div className="text-2xl font-semibold text-blue-300">{navigationData.distance.toFixed(3)} km</div>
          </div>
          
          {/* Compass visualization - MONOCHROME */}
          <div className="bg-black/20 rounded-lg p-4 border border-white/10 flex items-center justify-center">
            <div className="relative w-32 h-32">
              {/* Compass circle */}
              <div className="absolute inset-0 border-2 border-white/20 rounded-full"></div>
              <div className="absolute inset-2 border border-white/10 rounded-full"></div>
              
              {/* Cardinal directions */}
              <div className="absolute top-0 left-1/2 -translate-x-1/2 -translate-y-2 text-xs font-bold text-slate-300">N</div>
              <div className="absolute bottom-0 left-1/2 -translate-x-1/2 translate-y-2 text-xs text-slate-400">S</div>
              <div className="absolute left-0 top-1/2 -translate-y-1/2 -translate-x-2 text-xs text-slate-400">W</div>
              <div className="absolute right-0 top-1/2 -translate-y-1/2 translate-x-2 text-xs text-slate-400">E</div>
              
              {/* Heading indicator */}
              <div 
                className="absolute inset-0 flex items-start justify-center"
                style={{ transform: `rotate(${navigationData.heading}deg)` }}
              >
                <div className="w-1 h-14 bg-gradient-to-b from-blue-400 to-transparent"></div>
              </div>
              
              {/* Center dot */}
              <div className="absolute inset-0 flex items-center justify-center">
                <div className="w-2 h-2 bg-blue-400 rounded-full"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      {/* Mission Telemetry Log - PROFESSIONAL MONOCHROME */}
      <div className="lg:col-span-2 bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all flex flex-col">
        <div className="flex items-center justify-between mb-4 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">MISSION TELEMETRY LOG</h4>
          </div>
          <div className="flex items-center space-x-2">
            <div className="w-1.5 h-1.5 bg-emerald-400 rounded-full"></div>
            <span className="text-xs text-emerald-400">LIVE STREAM</span>
          </div>
        </div>
        
        <div className="flex-1 overflow-y-auto space-y-2 pr-2 custom-scrollbar">
          {logs.slice(-15).reverse().map((log, index) => (
            <div 
              key={index} 
              className={`flex items-start space-x-3 p-3 rounded-lg border-l-2 ${getLogColor(log.type)} backdrop-blur-sm transition-all hover:bg-white/5`}
            >
              <div className="flex-shrink-0 mt-0.5">
                {getLogIcon(log.type)}
              </div>
              <div className="flex-1 min-w-0">
                <div className="flex items-center justify-between mb-1">
                  <span className="text-xs font-semibold text-slate-300 uppercase tracking-wide">
                    {log.source}
                  </span>
                  <span className="text-[10px] text-slate-500 font-mono">
                    {formatTime(log.timestamp)}
                  </span>
                </div>
                <p className="text-sm text-slate-400 leading-relaxed">
                  {log.message}
                </p>
              </div>
            </div>
          ))}
        </div>
        
        {logs.length === 0 && (
          <div className="flex-1 flex items-center justify-center text-slate-500">
            <div className="text-center">
              <svg className="w-12 h-12 mx-auto mb-3 text-slate-600" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
              </svg>
              <p className="text-sm">No telemetry data available</p>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default MissionControlPanel;