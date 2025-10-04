import React from 'react';

interface Site {
  position: { x: number; y: number };
  total_score: number;
}

interface TerrainMapProps {
  sites: Site[];
  robotPosition: { x: number; y: number };
}

const TerrainMap: React.FC<TerrainMapProps> = ({ sites, robotPosition }) => {
  // Determine the bounds of the map based on sites and robot position
  const allPoints = [
    robotPosition,
    ...sites.map(site => site.position)
  ];
  
  if (allPoints.length === 0) {
    return (
      <div className="h-72 flex flex-col items-center justify-center bg-gradient-to-br from-gray-800 to-gray-900 rounded-xl border border-gray-700 p-6">
        <div className="w-16 h-16 mx-auto mb-4 rounded-full bg-gray-700 flex items-center justify-center">
          <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-gray-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9.172 16.172a4 4 0 015.656 0M9 10h.01M15 10h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
        </div>
        <h3 className="text-lg font-semibold text-gray-400 mb-1">No Data Available</h3>
        <p className="text-gray-500 max-w-md mx-auto text-center">
          The LunaBot is currently scanning the lunar surface for potential habitat sites. Please wait for data.
        </p>
      </div>
    );
  }
  
  // Calculate map boundaries
  const xValues = allPoints.map(p => p.x);
  const yValues = allPoints.map(p => p.y);
  const minX = Math.min(...xValues, robotPosition.x) - 5;
  const maxX = Math.max(...xValues, robotPosition.x) + 5;
  const minY = Math.min(...yValues, robotPosition.y) - 5;
  const maxY = Math.max(...yValues, robotPosition.y) + 5;
  
  // Calculate scale factors for converting coordinates to screen space
  const containerWidth = 100;
  const containerHeight = 100;
  const scaleX = containerWidth / (maxX - minX);
  const scaleY = containerHeight / (maxY - minY);
  
  // Function to convert world coordinates to percentage positions
  const toXPercent = (x: number) => ((x - minX) * scaleX);
  const toYPercent = (y: number) => ((y - minY) * scaleY);
  
  return (
    <div className="h-72 bg-[#0f172a]/70 border border-cyan-500/20 rounded-lg relative overflow-hidden scanning-line holographic-border">
      <div className="absolute inset-0 bg-[#0f172a]">
        {/* Grid lines */}
        <div className="absolute inset-0 opacity-30">
          {[...Array(12)].map((_, i) => (
            <div key={`v-${i}`} className="absolute top-0 bottom-0 border-r border-cyan-500/30" style={{ left: `${i * 8.33}%` }}></div>
          ))}
          {[...Array(12)].map((_, i) => (
            <div key={`h-${i}`} className="absolute left-0 right-0 border-b border-cyan-500/30" style={{ top: `${i * 8.33}%` }}></div>
          ))}
        </div>
        
        {/* Terrain features (simulated) */}
        <div className="absolute inset-0">
          {/* Simulated craters */}
          <div className="absolute rounded-full opacity-10" style={{ width: '8%', height: '8%', top: '20%', left: '15%', background: 'radial-gradient(circle, rgba(100,200,255,0.3) 0%, rgba(30,58,138,0.4) 100%)' }}></div>
          <div className="absolute rounded-full opacity-10" style={{ width: '6%', height: '6%', top: '60%', left: '70%', background: 'radial-gradient(circle, rgba(100,200,255,0.3) 0%, rgba(30,58,138,0.4) 100%)' }}></div>
          <div className="absolute rounded-full opacity-10" style={{ width: '12%', height: '12%', top: '40%', left: '40%', background: 'radial-gradient(circle, rgba(100,200,255,0.3) 0%, rgba(30,58,138,0.4) 100%)' }}></div>
        </div>
        
        {/* Sites */}
        {sites.map((site, index) => {
          const xPercent = toXPercent(site.position.x);
          const yPercent = toYPercent(site.position.y);
          
          // Determine color based on score
          let color = 'bg-red-500';
          let glow = 'glow-red';
          let size = 'w-6 h-6';
          
          if (site.total_score >= 80) {
            color = 'bg-green-500';
            glow = 'glow-green';
          } else if (site.total_score >= 60) {
            color = 'bg-yellow-500';
            glow = 'glow-yellow';
          }
          
          // Adjust size based on score
          if (site.total_score >= 90) size = 'w-7 h-7';
          else if (site.total_score >= 70) size = 'w-6 h-6';
          else size = 'w-5 h-5';
          
          return (
            <div 
              key={`site-${index}`}
              className={`absolute transform -translate-x-1/2 -translate-y-1/2 ${color} rounded-full ${glow} animate-pulse`}
              style={{
                left: `${xPercent}%`,
                top: `${yPercent}%`,
                width: size.split(' ')[0].replace('w-', '24px') // fallback
              }}
              title={`Site ${index + 1}: ${site.total_score.toFixed(1)} score`}
            >
              <div className="absolute -top-6 left-1/2 transform -translate-x-1/2 bg-[#0f172a]/90 backdrop-blur-sm text-white text-[10px] px-2 py-1 rounded border border-cyan-500/50">
                {site.total_score.toFixed(1)}
              </div>
              
              {/* Score indicator */}
              <div className="absolute -bottom-6 left-1/2 transform -translate-x-1/2 text-[10px] font-bold" style={{color: site.total_score >= 80 ? '#10b981' : site.total_score >= 60 ? '#fbbf24' : '#ef4444'}}>
                #{index + 1}
              </div>
            </div>
          );
        })}
        
        {/* Robot position */}
        <div 
          className="absolute transform -translate-x-1/2 -translate-y-1/2 bg-gradient-to-r from-cyan-500 to-blue-500 rounded-full glow-cyan animate-pulse"
          style={{
            left: `${toXPercent(robotPosition.x)}%`,
            top: `${toYPercent(robotPosition.y)}%`,
            width: '28px',
            height: '28px',
          }}
          title={`Robot: (${robotPosition.x.toFixed(1)}, ${robotPosition.y.toFixed(1)})`}
        >
          <div className="absolute -top-6 left-1/2 transform -translate-x-1/2 bg-[#0f172a]/90 backdrop-blur-sm text-white text-[10px] px-2 py-1 rounded border border-cyan-500/50">
            ROBOT
          </div>
          
          {/* Robot icon */}
          <div className="absolute inset-0 flex items-center justify-center">
            <svg xmlns="http://www.w3.org/2000/svg" className="h-4 w-4 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 9l3 3-3 3m5 0h3M5 20h14a2 2 0 002-2V6a2 2 0 00-2-2H5a2 2 0 00-2 2v12a2 2 0 002 2z" />
            </svg>
          </div>
        </div>
      </div>
      
      {/* Legend */}
      <div className="absolute bottom-2 left-2 bg-[#0f172a]/80 backdrop-blur-sm rounded border border-cyan-500/30 p-2">
        <div className="text-[10px] font-bold text-cyan-400 mb-1">LEGEND</div>
        <div className="flex space-x-3 text-[10px]">
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-1 glow-green"></div>
            <span>≥80</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-yellow-500 rounded-full mr-1 glow-yellow"></div>
            <span>60-79</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-red-500 rounded-full mr-1 glow-red"></div>
            <span>&lt;60</span>
          </div>
          <div className="flex items-center ml-2">
            <div className="w-3 h-3 bg-cyan-500 rounded-full mr-1 glow-cyan"></div>
            <span>ROBOT</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default TerrainMap;