import React from 'react';
import { Radar, RadarChart, PolarGrid, PolarAngleAxis, PolarRadiusAxis, ResponsiveContainer, Legend } from 'recharts';

interface Site {
  position: { x: number; y: number };
  scores: {
    safety: number;
    buildability: number;
    resources: number;
    expandability: number;
  };
  total_score: number;
}

interface SiteComparisonProps {
  sites: Site[];
}

const SiteComparison: React.FC<SiteComparisonProps> = ({ sites }) => {
  // Prepare data for radar chart (top 3 sites)
  const topSites = sites.slice(0, 3);
  
  // Format data for radar chart
  const radarData = [
    { subject: 'Safety', ...topSites.reduce((acc, site, idx) => ({...acc, [`site${idx+1}`]: site.scores.safety}), {}) },
    { subject: 'Buildability', ...topSites.reduce((acc, site, idx) => ({...acc, [`site${idx+1}`]: site.scores.buildability}), {}) },
    { subject: 'Resources', ...topSites.reduce((acc, site, idx) => ({...acc, [`site${idx+1}`]: site.scores.resources}), {}) },
    { subject: 'Expandability', ...topSites.reduce((acc, site, idx) => ({...acc, [`site${idx+1}`]: site.scores.expandability}), {}) },
    { subject: 'Total', ...topSites.reduce((acc, site, idx) => ({...acc, [`site${idx+1}`]: site.total_score}), {}) },
  ];

  // PROFESSIONAL MONOCHROME COLORS (gray shades only)
  const siteColors = [
    { stroke: '#64748b', fill: '#64748b', name: 'Site 1' },  // slate-500
    { stroke: '#94a3b8', fill: '#94a3b8', name: 'Site 2' },  // slate-400
    { stroke: '#cbd5e1', fill: '#cbd5e1', name: 'Site 3' }   // slate-300
  ];

  return (
    <div>
      {sites.length > 0 ? (
        <div>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-5 mb-6">
            {topSites.map((site, index) => (
              <div key={index} className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4 hover:border-white/20 transition-all">
                <div className="flex justify-between items-center mb-3 pb-2 border-b border-white/10">
                  <h4 className="font-semibold text-slate-200 tracking-wide">
                    SITE {index + 1}
                  </h4>
                  <div className={`w-1.5 h-1.5 rounded-full ${index === 0 ? 'bg-blue-400' : 'bg-slate-400'}`}></div>
                </div>
                
                <div className="space-y-2">
                  <div className="flex justify-between border-b border-white/10 pb-1">
                    <span className="text-slate-400 text-xs">COORDS:</span>
                    <span className="font-mono text-slate-300 text-xs">({site.position.x.toFixed(2)}, {site.position.y.toFixed(2)})</span>
                  </div>
                  
                  <div className="flex justify-between text-xs">
                    <span className="text-slate-400">Safety:</span>
                    <span className="font-semibold text-slate-300">{site.scores.safety.toFixed(1)}</span>
                  </div>
                  <div className="flex justify-between text-xs">
                    <span className="text-slate-400">Buildability:</span>
                    <span className="font-semibold text-slate-300">{site.scores.buildability.toFixed(1)}</span>
                  </div>
                  <div className="flex justify-between text-xs">
                    <span className="text-slate-400">Resources:</span>
                    <span className="font-semibold text-slate-300">{site.scores.resources.toFixed(1)}</span>
                  </div>
                  <div className="flex justify-between text-xs">
                    <span className="text-slate-400">Expandability:</span>
                    <span className="font-semibold text-slate-300">{site.scores.expandability.toFixed(1)}</span>
                  </div>
                  <div className="flex justify-between font-bold border-t border-white/10 pt-2 mt-2 text-sm">
                    <span className="text-blue-300">
                      Total:
                    </span>
                    <span className="text-blue-300">
                      {site.total_score.toFixed(1)}
                    </span>
                  </div>
                </div>
              </div>
            ))}
          </div>
          
          <div className="bg-gradient-to-br from-slate-900/70 to-slate-800/50 backdrop-blur-sm border border-white/10 rounded-lg p-4">
            <h5 className="text-sm font-semibold mb-3 text-slate-200 tracking-wide">SITE COMPARISON RADAR</h5>
            <div className="h-72">
              <ResponsiveContainer width="100%" height="100%">
                <RadarChart cx="50%" cy="50%" outerRadius="70%" data={radarData}>
                  <defs>
                    {siteColors.map((color, index) => (
                      <linearGradient key={`color-${index}`} id={`colorGradient-${index}`} x1="0" y1="0" x2="0" y2="1">
                        <stop offset="0%" stopColor={color.stroke} stopOpacity={0.3} />
                        <stop offset="100%" stopColor={color.stroke} stopOpacity={0.05} />
                      </linearGradient>
                    ))}
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
                  {topSites.map((_, index) => (
                    <Radar
                      key={index}
                      name={`Site ${index + 1}`}
                      dataKey={`site${index + 1}`}
                      stroke={siteColors[index].stroke}
                      fill={`url(#colorGradient-${index})`}
                      fillOpacity={0.4}
                      strokeWidth={1.5}
                    />
                  ))}
                  <Legend 
                    wrapperStyle={{ 
                      paddingTop: '15px',
                      color: '#94a3b8',
                      fontSize: '12px'
                    }} 
                  />
                </RadarChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>
      ) : (
        <div className="text-center py-12">
          <div className="inline-block p-4 rounded-full bg-slate-900/70 mb-4 border border-white/10">
            <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 text-slate-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9 17v-2m3 2v-4m3 4v-6m2 10H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
            </svg>
          </div>
          <h3 className="text-lg font-semibold text-slate-200 mb-2">NO DATA AVAILABLE</h3>
          <p className="text-slate-400 max-w-md mx-auto">
            LUNABOT IS STILL SCANNING THE LUNAR SURFACE... Please wait for habitat site analysis.
          </p>
        </div>
      )}
    </div>
  );
};

export default SiteComparison;