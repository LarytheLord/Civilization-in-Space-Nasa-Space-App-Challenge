// Mock Data Service - Simulates LunaBot telemetry without backend

export interface SensorData {
  // Radiation sensors (TEPC)
  radiation: {
    cosmic: number; // mSv/h
    solar: number;  // mSv/h
    total: number;
    status: 'optimal' | 'caution' | 'critical';
  };
  
  // Seismic sensors
  seismic: {
    frequency: number; // Hz
    magnitude: number;
    stability: number; // 0-100
    lastQuake: string;
    status: 'stable' | 'minor' | 'major';
  };
  
  // Regolith analyzer (XRF)
  regolith: {
    composition: {
      iron: number;
      titanium: number;
      silicon: number;
      aluminum: number;
      calcium: number;
      magnesium: number;
    };
    density: number; // g/cm³
    waterIce: number; // percentage
    isruPotential: number; // 0-100
  };
  
  // Thermal sensors
  thermal: {
    surface: number; // °C
    subsurface: number;
    gradient: number;
    status: 'nominal' | 'hot' | 'cold';
  };
  
  // LiDAR
  lidar: {
    range: number; // meters
    pointsPerSecond: number;
    obstaclesDetected: number;
    terrainRoughness: number; // 0-100
  };
  
  // Cameras
  cameras: {
    frontCamera: { status: string; resolution: string; };
    rearCamera: { status: string; resolution: string; };
    visibility: number; // 0-100
    dustLevel: number; // 0-100
  };
  
  // Spectrometer
  spectrometer: {
    waterSignature: number; // 0-100
    mineralSignature: string;
    volatileDetection: number; // 0-100
  };
  
  // Environmental
  environmental: {
    solarExposure: number; // 0-100
    uvRadiation: number;
    electrostaticCharge: number;
    dustParticles: number; // per m³
  };
  
  // Power system
  power: {
    solar: number; // watts
    rtg: number;   // watts
    battery: number; // percentage
    consumption: number; // watts
    status: 'optimal' | 'reduced' | 'critical';
  };
  
  // Navigation
  navigation: {
    position: { x: number; y: number; z: number };
    heading: number; // degrees
    speed: number;   // m/s
    distance: number; // total km traveled
  };
}

export interface Site {
  id: number;
  position: { x: number; y: number };
  scores: {
    safety: number;
    resources: number;
    buildability: number;
    expandability: number;
  };
  total_score: number;
  details: {
    radiation: number;
    terrain: string;
    waterIce: number;
    solarExposure: number;
    seismicStability: number;
  };
}

export interface MissionLog {
  timestamp: string;
  type: 'info' | 'warning' | 'success' | 'error';
  message: string;
  subsystem?: string;
}

class MockDataService {
  private sensorData: SensorData;
  private sites: Site[];
  private missionLogs: MissionLog[];
  private updateInterval: NodeJS.Timeout | null = null;
  
  constructor() {
    this.sensorData = this.generateInitialSensorData();
    this.sites = this.generateSites(8);
    this.missionLogs = this.generateInitialLogs();
  }
  
  private generateInitialSensorData(): SensorData {
    return {
      radiation: {
        cosmic: 0.42,
        solar: 0.18,
        total: 0.60,
        status: 'optimal'
      },
      seismic: {
        frequency: 1.2,
        magnitude: 0.3,
        stability: 94,
        lastQuake: '12h 34m ago',
        status: 'stable'
      },
      regolith: {
        composition: {
          iron: 14.2,
          titanium: 3.8,
          silicon: 42.5,
          aluminum: 12.1,
          calcium: 9.4,
          magnesium: 7.2
        },
        density: 1.52,
        waterIce: 0.8,
        isruPotential: 78
      },
      thermal: {
        surface: -23,
        subsurface: -18,
        gradient: 5,
        status: 'nominal'
      },
      lidar: {
        range: 150,
        pointsPerSecond: 300000,
        obstaclesDetected: 3,
        terrainRoughness: 42
      },
      cameras: {
        frontCamera: { status: 'ONLINE', resolution: '4K' },
        rearCamera: { status: 'ONLINE', resolution: '4K' },
        visibility: 98,
        dustLevel: 12
      },
      spectrometer: {
        waterSignature: 45,
        mineralSignature: 'Anorthite detected',
        volatileDetection: 32
      },
      environmental: {
        solarExposure: 87,
        uvRadiation: 245,
        electrostaticCharge: 2.3,
        dustParticles: 145
      },
      power: {
        solar: 380,
        rtg: 120,
        battery: 94,
        consumption: 48,
        status: 'optimal'
      },
      navigation: {
        position: { x: 124.5, y: 89.3, z: 2.1 },
        heading: 142,
        speed: 0.8,
        distance: 3.247
      }
    };
  }
  
  private generateSites(count: number): Site[] {
    const sites: Site[] = [];
    for (let i = 0; i < count; i++) {
      const safety = 60 + Math.random() * 35;
      const resources = 50 + Math.random() * 45;
      const buildability = 55 + Math.random() * 40;
      const expandability = 50 + Math.random() * 45;
      
      // Weighted scoring: Safety 40%, Resources 30%, Construction 20%, Science 10%
      const total = (safety * 0.4) + (resources * 0.3) + (buildability * 0.2) + (expandability * 0.1);
      
      sites.push({
        id: i + 1,
        position: {
          x: 100 + (Math.random() * 100) - 50,
          y: 80 + (Math.random() * 100) - 50
        },
        scores: {
          safety: Math.round(safety * 10) / 10,
          resources: Math.round(resources * 10) / 10,
          buildability: Math.round(buildability * 10) / 10,
          expandability: Math.round(expandability * 10) / 10
        },
        total_score: Math.round(total * 10) / 10,
        details: {
          radiation: 0.3 + Math.random() * 0.6,
          terrain: ['Flat Mare', 'Highland Crater', 'Gentle Slope', 'Ridge Area'][Math.floor(Math.random() * 4)],
          waterIce: Math.random() * 2.5,
          solarExposure: 70 + Math.random() * 30,
          seismicStability: 80 + Math.random() * 20
        }
      });
    }
    
    // Sort by total score descending
    return sites.sort((a, b) => b.total_score - a.total_score);
  }
  
  private generateInitialLogs(): MissionLog[] {
    const now = new Date();
    return [
      {
        timestamp: new Date(now.getTime() - 300000).toISOString(),
        type: 'success',
        message: 'LunaBot systems initialized successfully',
        subsystem: 'CORE'
      },
      {
        timestamp: new Date(now.getTime() - 240000).toISOString(),
        type: 'info',
        message: 'Multi-sensor array online - all sensors nominal',
        subsystem: 'SENSORS'
      },
      {
        timestamp: new Date(now.getTime() - 180000).toISOString(),
        type: 'info',
        message: 'Autonomous navigation system activated',
        subsystem: 'NAV'
      },
      {
        timestamp: new Date(now.getTime() - 120000).toISOString(),
        type: 'success',
        message: 'Initial site analysis complete - 8 sites identified',
        subsystem: 'ANALYSIS'
      },
      {
        timestamp: new Date(now.getTime() - 60000).toISOString(),
        type: 'info',
        message: 'Water ice signature detected at Site 2',
        subsystem: 'SPECTROMETER'
      }
    ];
  }
  
  // Simulate realistic sensor updates
  private updateSensorData() {
    // Radiation - slow drift
    this.sensorData.radiation.cosmic += (Math.random() - 0.5) * 0.02;
    this.sensorData.radiation.solar += (Math.random() - 0.5) * 0.05;
    this.sensorData.radiation.total = this.sensorData.radiation.cosmic + this.sensorData.radiation.solar;
    
    // Keep radiation in realistic range
    this.sensorData.radiation.cosmic = Math.max(0.3, Math.min(0.6, this.sensorData.radiation.cosmic));
    this.sensorData.radiation.solar = Math.max(0.1, Math.min(0.4, this.sensorData.radiation.solar));
    
    // Seismic - occasional activity
    if (Math.random() < 0.05) {
      this.sensorData.seismic.magnitude = Math.random() * 1.5;
      this.addMissionLog('warning', 'Minor seismic activity detected', 'SEISMIC');
    } else {
      this.sensorData.seismic.magnitude *= 0.95;
    }
    this.sensorData.seismic.stability = 100 - (this.sensorData.seismic.magnitude * 20);
    
    // Thermal - slow temperature changes
    this.sensorData.thermal.surface += (Math.random() - 0.5) * 2;
    this.sensorData.thermal.surface = Math.max(-50, Math.min(10, this.sensorData.thermal.surface));
    this.sensorData.thermal.subsurface = this.sensorData.thermal.surface + 5;
    
    // Navigation - simulate movement
    this.sensorData.navigation.position.x += Math.sin(this.sensorData.navigation.heading * Math.PI / 180) * this.sensorData.navigation.speed * 0.1;
    this.sensorData.navigation.position.y += Math.cos(this.sensorData.navigation.heading * Math.PI / 180) * this.sensorData.navigation.speed * 0.1;
    this.sensorData.navigation.distance += this.sensorData.navigation.speed * 0.0001;
    
    // Power - fluctuate slightly
    this.sensorData.power.solar = 350 + Math.random() * 50;
    this.sensorData.power.battery += (Math.random() - 0.48) * 0.5; // Slow charging
    this.sensorData.power.battery = Math.max(85, Math.min(100, this.sensorData.power.battery));
    
    // LiDAR
    this.sensorData.lidar.obstaclesDetected = Math.floor(Math.random() * 5);
    
    // Spectrometer
    this.sensorData.spectrometer.waterSignature += (Math.random() - 0.5) * 5;
    this.sensorData.spectrometer.waterSignature = Math.max(0, Math.min(100, this.sensorData.spectrometer.waterSignature));
    
    // Occasionally add mission logs
    if (Math.random() < 0.1) {
      const messages = [
        { type: 'info' as const, msg: 'Terrain mapping in progress', sys: 'LIDAR' },
        { type: 'info' as const, msg: 'Regolith sample analyzed', sys: 'XRF' },
        { type: 'success' as const, msg: 'Site scoring updated', sys: 'ANALYSIS' },
        { type: 'info' as const, msg: 'Navigation waypoint reached', sys: 'NAV' },
      ];
      const selected = messages[Math.floor(Math.random() * messages.length)];
      this.addMissionLog(selected.type, selected.msg, selected.sys);
    }
  }
  
  private addMissionLog(type: MissionLog['type'], message: string, subsystem?: string) {
    this.missionLogs.unshift({
      timestamp: new Date().toISOString(),
      type,
      message,
      subsystem
    });
    
    // Keep only last 50 logs
    if (this.missionLogs.length > 50) {
      this.missionLogs = this.missionLogs.slice(0, 50);
    }
  }
  
  // Public API
  startSimulation(callback: (data: { sensors: SensorData; sites: Site[]; logs: MissionLog[] }) => void) {
    this.updateInterval = setInterval(() => {
      this.updateSensorData();
      callback({
        sensors: this.sensorData,
        sites: this.sites,
        logs: this.missionLogs
      });
    }, 1000); // Update every second
    
    // Immediate first call
    callback({
      sensors: this.sensorData,
      sites: this.sites,
      logs: this.missionLogs
    });
  }
  
  stopSimulation() {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
    }
  }
  
  getSensorData() {
    return this.sensorData;
  }
  
  getSites() {
    return this.sites;
  }
  
  getMissionLogs() {
    return this.missionLogs;
  }
}

export default new MockDataService();
