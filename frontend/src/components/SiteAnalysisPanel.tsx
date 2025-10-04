import React from 'react';
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, Cell } from 'recharts';

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

interface SiteAnalysisPanelProps {
  sites: Site[];
}

const SiteAnalysisPanel: React.FC<SiteAnalysisPanelProps> = ({ sites }) => {
  // Take top 3 sites for detailed analysis
  const topSites = sites.slice(0, 3);

  // Prepare data for the bar chart
  const chartData = topSites.map((site, index) => ({
    name: `Site ${index + 1}`,
    safety: site.scores.safety,
    buildability: site.scores.buildability,
    resources: site.scores.resources,
    expandability: site.scores.expandability,
    total: site.total_score
  }));

  // Define colors for each metric
  const metricColors = {
    safety: '#ef4444',
    buildability: '#eab308',
    resources: '#3b82f6',
    expandability: '#22c55e',
    total: '#8b5cf6'
  };

  return (
    <div>
      {sites.length > 0 ? (
        <div>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
            {topSites.map((site, index) => (
              <div key={index} className="bg-[#0f172a]/70 border border-cyan-500/20 rounded-lg p-4 scanning-line">
                <div className="flex justify-between items-center mb-3 pb-2 border-b border-cyan-500/30">
                  <h4 className="font-bold text-cyan-400">
                    <span className="bg-clip-text text-transparent bg-gradient-to-r from-cyan-400 to-blue-400">
                      SITE {index + 1}
                    </span>
                  </h4>
                  <div className="flex items-center">
                    <div className="text-xs bg-cyan-900/50 text-cyan-300 px-2 py-1 rounded mr-2">
                      RANK #{index + 1}
                    </div>
                    <div className={`w-3 h-3 rounded-full ${index === 0 ? 'bg-green-500' : index === 1 ? 'bg-yellow-500' : 'bg-orange-500'} animate-pulse`}></div>
                  </div>
                </div>
                
                <div className="text-xs text-gray-400 mb-3 pb-2 border-b border-cyan-500/20">
                  <div className="flex justify-between mb-1">
                    <span>COORDS:</span>
                    <span className="font-mono text-cyan-300">({site.position.x.toFixed(2)}, {site.position.y.toFixed(2)})</span>
                  </div>
                </div>
                
                <div className="space-y-3">
                  {/* Safety */}
                  <div>
                    <div className="flex justify-between mb-1">
                      <span className="text-xs text-red-400">SAFETY</span>
                      <span className="text-xs font-bold text-red-300">{site.scores.safety.toFixed(1)}</span>
                    </div>
                    <div className="w-full bg-[#0f172a] rounded-full h-2 border border-red-500/30">
                      <div 
                        className="bg-gradient-to-r from-red-600 to-red-500 h-2 rounded-full" 
                        style={{ width: `${site.scores.safety}%` }}
                      ></div>
                    </div>
                  </div>
                  
                  {/* Buildability */}
                  <div>
                    <div className="flex justify-between mb-1">
                      <span className="text-xs text-yellow-400">BUILDABILITY</span>
                      <span className="text-xs font-bold">{site.scores.buildability.toFixed(1)}/100</span>
                    </div>
                    <div className="w-full bg-gray-700 rounded-full h-2.5">
                      <div 
                        className="bg-gradient-to-r from-yellow-500 to-yellow-600 h-2.5 rounded-full" 
                        style={{ width: `${site.scores.buildability}%` }}
                      ></div>
                    </div>
                  </div>
                  
                  {/* Resources */}
                  <div>
                    <div className="flex justify-between mb-1">
                      <span className="text-xs text-blue-400">RESOURCES</span>
                      <span className="text-xs font-bold">{site.scores.resources.toFixed(1)}/100</span>
                    </div>
                    <div className="w-full bg-gray-700 rounded-full h-2.5">
                      <div 
                        className="bg-gradient-to-r from-blue-500 to-blue-600 h-2.5 rounded-full" 
                        style={{ width: `${site.scores.resources}%` }}
                      ></div>
                    </div>
                  </div>
                  
                  {/* Expandability */}
                  <div>
                    <div className="flex justify-between mb-1">
                      <span className="text-xs text-green-400">EXPANDABILITY</span>
                      <span className="text-xs font-bold">{site.scores.expandability.toFixed(1)}/100</span>
                    </div>
                    <div className="w-full bg-gray-700 rounded-full h-2.5">
                      <div 
                        className="bg-gradient-to-r from-green-500 to-green-600 h-2.5 rounded-full" 
                        style={{ width: `${site.scores.expandability}%` }}
                      ></div>
                    </div>
                  </div>
                  
                  {/* Total Score */}
                  <div className="pt-2 mt-2 border-t border-gray-700">
                    <div className="flex justify-between mb-1">
                      <span className="text-xs text-purple-400 font-bold">TOTAL SCORE</span>
                      <span className="text-xs font-bold">{site.total_score.toFixed(1)}/100</span>
                    </div>
                    <div className="w-full bg-gray-700 rounded-full h-3">
                      <div 
                        className="bg-gradient-to-r from-purple-500 to-purple-600 h-3 rounded-full flex items-center justify-center" 
                        style={{ width: `${site.total_score}%` }}
                      >
                        {site.total_score > 60 && (
                          <div className="text-[8px] text-white font-bold">
                            {site.total_score.toFixed(0)}%
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            ))}
          </div>
          
          <div className="recharts-wrapper scanning-line">
            <h5 className="text-sm font-bold mb-3 text-cyan-400">SITE SCORE COMPARISON MATRIX</h5>
            <div className="h-72">
              <ResponsiveContainer width="100%" height="100%">
                <BarChart
                  data={chartData}
                  margin={{ top: 20, right: 30, left: 20, bottom: 50 }}
                >
                  <CartesianGrid strokeDasharray="3 3" stroke="#374151" vertical={false} />
                  <XAxis 
                    dataKey="name" 
                    stroke="#9CA3AF" 
                    angle={-45} 
                    textAnchor="end" 
                    height={60}
                    tick={{ fontSize: 12 }}
                  />
                  <YAxis 
                    stroke="#9CA3AF" 
                    domain={[0, 100]} 
                    tick={{ fontSize: 12 }}
                  />
                  <Tooltip 
                    contentStyle={{ 
                      backgroundColor: '#1F2937', 
                      borderColor: '#374151',
                      borderRadius: '0.5rem',
                      color: '#fff'
                    }} 
                    itemStyle={{ color: '#fff' }} 
                    labelStyle={{ color: '#fff', fontWeight: 'bold' }}
                  />
                  <Legend 
                    wrapperStyle={{ 
                      paddingTop: '10px',
                      color: '#9CA3AF'
                    }} 
                  />
                  <Bar dataKey="safety" name="Safety">
                    {chartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={metricColors.safety} />
                    ))}
                  </Bar>
                  <Bar dataKey="buildability" name="Buildability">
                    {chartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={metricColors.buildability} />
                    ))}
                  </Bar>
                  <Bar dataKey="resources" name="Resources">
                    {chartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={metricColors.resources} />
                    ))}
                  </Bar>
                  <Bar dataKey="expandability" name="Expandability">
                    {chartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={metricColors.expandability} />
                    ))}
                  </Bar>
                  <Bar dataKey="total" name="Total Score">
                    {chartData.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={metricColors.total} />
                    ))}
                  </Bar>
                </BarChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>
      ) : (
        <div className="text-center py-12 scanning-line">
          <div className="inline-block p-4 rounded-full bg-[#0f172a]/70 mb-4 border border-cyan-500/30">
            <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 text-cyan-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9 12l2 2 4-4m5.618-4.016A11.955 11.955 0 0112 2.944a11.955 11.955 0 01-8.618 3.04A12.02 12.02 0 003 9c0 5.591 3.824 10.29 9 11.622 5.176-1.332 9-6.03 9-11.622 0-1.042-.133-2.052-.382-3.016z" />
            </svg>
          </div>
          <h3 className="text-lg font-bold text-cyan-400 mb-2">NO DATA AVAILABLE</h3>
          <p className="text-gray-400 max-w-md mx-auto">
            LUNABOT SYSTEM IS INITIALIZING... Awaiting telemetry feed from lunar surface.
          </p>
        </div>
      )}
    </div>
  );
};

export default SiteAnalysisPanel;