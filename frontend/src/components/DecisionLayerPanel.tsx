import React from 'react';
import { RadarChart, Radar, PolarGrid, PolarAngleAxis, PolarRadiusAxis, ResponsiveContainer } from 'recharts';

interface DecisionLayerPanelProps {
  currentSite?: {
    id: number;
    scores: {
      safety: number;
      resources: number;
      buildability: number;
      expandability: number;
    };
    total_score: number;
  };
}

const DecisionLayerPanel: React.FC<DecisionLayerPanelProps> = ({ currentSite }) => {
  // Multi-criteria scoring weights (MONOCHROME - professional)
  const scoringCriteria = [
    { name: 'Safety', weight: 40, description: 'Radiation, Seismic, Thermal' },
    { name: 'Resources', weight: 30, description: 'Water, Minerals, Solar' },
    { name: 'Construction', weight: 20, description: 'Terrain, Regolith' },
    { name: 'Science', weight: 10, description: 'Expandability, Research' }
  ];

  // Decision states (MONOCHROME)
  const decisionStates = [
    { 
      phase: 'SENSOR FUSION', 
      status: 'ACTIVE',
      icon: (
        <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
        </svg>
      )
    },
    { 
      phase: 'AI ANALYSIS', 
      status: 'PROCESSING',
      icon: (
        <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
        </svg>
      )
    },
    { 
      phase: 'SCORING', 
      status: 'ACTIVE',
      icon: (
        <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
        </svg>
      )
    },
    { 
      phase: 'DECISION', 
      status: 'READY',
      icon: (
        <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
        </svg>
      )
    }
  ];

  // Radar chart data
  const radarData = currentSite ? [
    { subject: 'Safety', score: currentSite.scores.safety, fullMark: 100 },
    { subject: 'Resources', score: currentSite.scores.resources, fullMark: 100 },
    { subject: 'Construction', score: currentSite.scores.buildability, fullMark: 100 },
    { subject: 'Science', score: currentSite.scores.expandability, fullMark: 100 }
  ] : [];

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
      {/* AI Decision Pipeline */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex items-center justify-between mb-4 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">AI DECISION PIPELINE</h4>
          </div>
          <span className="text-xs px-2 py-0.5 rounded-full border bg-emerald-500/10 text-emerald-400 border-emerald-500/20">AUTONOMOUS</span>
        </div>
        
        <div className="space-y-3">
          {decisionStates.map((state, index) => (
            <div key={index} className="relative">
              <div className="flex items-center space-x-3 bg-black/20 rounded-lg p-3 border border-white/10 hover:border-white/20 transition-all">
                <div className="flex-shrink-0 w-10 h-10 rounded-lg border border-white/10 bg-white/5 flex items-center justify-center">
                  <div className="text-slate-300">
                    {state.icon}
                  </div>
                </div>
                
                <div className="flex-1">
                  <div className="text-sm font-semibold text-slate-200">{state.phase}</div>
                  <div className="text-xs text-slate-400">{state.status}</div>
                </div>
                
                <div className="w-1.5 h-1.5 rounded-full bg-emerald-400"></div>
              </div>
              
              {/* Connector line */}
              {index < decisionStates.length - 1 && (
                <div className="absolute left-8 top-full w-0.5 h-3 bg-gradient-to-b from-white/10 to-transparent"></div>
              )}
            </div>
          ))}
        </div>
        
        {/* Real-time processing indicator */}
        <div className="mt-4 bg-blue-500/5 rounded-lg p-3 border border-blue-500/20">
          <div className="flex items-center justify-between">
            <div>
              <div className="text-xs text-slate-400">Processing Speed</div>
              <div className="text-lg font-semibold text-blue-300">Real-time</div>
            </div>
            <div>
              <div className="text-xs text-slate-400">Autonomy Level</div>
              <div className="text-lg font-semibold text-blue-300">95%</div>
            </div>
          </div>
        </div>
      </div>
      
      {/* Multi-Criteria Scoring */}
      <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
        <div className="flex items-center justify-between mb-4 pb-2 border-b border-white/10">
          <div className="flex items-center">
            <div className="w-1 h-4 bg-blue-400 rounded-full mr-2"></div>
            <h4 className="font-semibold text-slate-200 text-sm tracking-wide">SCORING ALGORITHM</h4>
          </div>
          <span className="text-xs px-2 py-0.5 rounded-full border bg-blue-500/10 text-blue-400 border-blue-500/20">WEIGHTED</span>
        </div>
        
        <div className="space-y-3 mb-4">
          {scoringCriteria.map((criterion, index) => (
            <div key={index} className="bg-black/20 rounded-lg p-3 border border-white/10">
              <div className="flex justify-between items-center mb-2">
                <span className="text-sm font-semibold text-slate-200">{criterion.name}</span>
                <span className="text-xs font-semibold text-blue-400">{criterion.weight}%</span>
              </div>
              <div className="text-xs text-slate-400 mb-2">{criterion.description}</div>
              <div className="w-full bg-black/30 rounded-full h-1.5 border border-white/5">
                <div 
                  className="bg-gradient-to-r from-slate-400 to-slate-500 h-1.5 rounded-full" 
                  style={{ width: `${criterion.weight}%` }}
                ></div>
              </div>
            </div>
          ))}
        </div>
        
        {/* Current Site Score (if available) */}
        {currentSite && (
          <div className="bg-blue-500/5 border border-blue-500/20 rounded-lg p-4 text-center">
            <div className="text-xs text-slate-400 mb-1">CURRENT SITE SCORE</div>
            <div className="text-4xl font-bold text-blue-300">{currentSite.total_score.toFixed(1)}</div>
            <div className="text-xs text-slate-500 mt-1">Site #{currentSite.id}</div>
          </div>
        )}
      </div>
      
      {/* Radar Chart for Multi-Criteria Analysis */}
      {currentSite && radarData.length > 0 && (
        <div className="lg:col-span-2 bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4">
          <h5 className="text-sm font-semibold mb-3 text-slate-200 tracking-wide">MULTI-CRITERIA ANALYSIS</h5>
          <div className="text-xs text-slate-400 mb-3">Site #{currentSite.id}</div>
          <div className="h-64">
            <ResponsiveContainer width="100%" height="100%">
              <RadarChart cx="50%" cy="50%" outerRadius="70%" data={radarData}>
                <defs>
                  <linearGradient id="radarGradient" x1="0" y1="0" x2="0" y2="1">
                    <stop offset="0%" stopColor="#3b82f6" stopOpacity={0.3} />
                    <stop offset="100%" stopColor="#3b82f6" stopOpacity={0.05} />
                  </linearGradient>
                </defs>
                <PolarGrid stroke="rgba(148, 163, 184, 0.2)" />
                <PolarAngleAxis 
                  dataKey="subject" 
                  tick={{ fill: '#94a3b8', fontSize: 12 }}
                  tickLine={false}
                  axisLine={false}
                />
                <PolarRadiusAxis 
                  angle={90} 
                  domain={[0, 100]} 
                  tick={{ fill: '#64748b', fontSize: 10 }}
                  tickCount={6}
                />
                <Radar
                  name="Score"
                  dataKey="score"
                  stroke="#3b82f6"
                  fill="url(#radarGradient)"
                  fillOpacity={0.5}
                  strokeWidth={2}
                />
              </RadarChart>
            </ResponsiveContainer>
          </div>
        </div>
      )}
    </div>
  );
};

export default DecisionLayerPanel;