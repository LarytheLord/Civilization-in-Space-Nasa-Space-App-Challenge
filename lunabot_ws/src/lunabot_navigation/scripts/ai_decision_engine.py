#!/usr/bin/env python3
"""
LunaBot AI Decision Engine
Integrates existing AI agent with ROS2 for intelligent decision making and mission planning
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import sys
import os
from datetime import datetime
from typing import Dict, List, Optional, Any
import asyncio
import threading

# Add the existing AI project to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../Project-Chimera-Athena/ai'))

# Import existing AI components
try:
    from agent import Agent
    from prometheus_core import PrometheusCognitiveCore
    from tool_user import ToolRegistry
    from memory import Memory
except ImportError as e:
    print(f"Warning: Could not import AI components: {e}")
    print("AI decision engine will run in simulation mode")

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray

class LunarMissionContext:
    """Context information for lunar mission decision making"""
    def __init__(self):
        self.current_mission_phase = "exploration"
        self.habitat_status = "operational"
        self.environmental_conditions = {}
        self.detected_anomalies = []
        self.maintenance_schedule = []
        self.emergency_protocols = []
        self.resource_levels = {}
        self.communication_window = True
        self.solar_conditions = "day"

class AIDecisionEngine(Node):
    """
    AI-powered decision engine for LunaBot
    Integrates cognitive AI with robotic systems for autonomous mission execution
    """
    
    def __init__(self):
        super().__init__('ai_decision_engine')
        
        # Initialize AI components
        self.initialize_ai_system()
        
        # Mission context
        self.mission_context = LunarMissionContext()
        self.decision_history = []
        self.learning_data = []
        
        # Decision making parameters
        self.decision_confidence_threshold = 0.7
        self.emergency_override_threshold = 0.9
        self.learning_rate = 0.1
        
        # Subscribers for sensor and system data
        self.habitat_status_sub = self.create_subscription(
            String, '/habitat_status', self.habitat_status_callback, 10)
        self.patrol_status_sub = self.create_subscription(
            String, '/patrol_status', self.patrol_status_callback, 10)
        self.navigation_status_sub = self.create_subscription(
            String, '/navigation_status', self.navigation_status_callback, 10)
        self.anomaly_alert_sub = self.create_subscription(
            String, '/anomaly_alert', self.anomaly_alert_callback, 10)
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.sensor_data_callback, 10)
        
        # Publishers for AI decisions and commands
        self.mission_command_pub = self.create_publisher(String, '/mission_command', 10)
        self.patrol_command_pub = self.create_publisher(String, '/patrol_command', 10)
        self.ai_status_pub = self.create_publisher(String, '/ai_status', 10)
        self.decision_log_pub = self.create_publisher(String, '/ai_decisions', 10)
        self.learning_insights_pub = self.create_publisher(String, '/ai_insights', 10)
        
        # Timers
        self.decision_timer = self.create_timer(5.0, self.decision_loop)
        self.learning_timer = self.create_timer(30.0, self.learning_loop)
        self.status_timer = self.create_timer(10.0, self.publish_ai_status)
        
        # Threading for AI processing
        self.ai_thread = None
        self.ai_queue = asyncio.Queue()
        
        self.get_logger().info("🤖 LunaBot AI Decision Engine initialized")
    
    def initialize_ai_system(self):
        """Initialize the AI cognitive system"""
        try:
            # Initialize cognitive core
            self.cognitive_core = PrometheusCognitiveCore(
                api_url="https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent"
            )
            
            # Initialize tool registry with lunar-specific tools
            self.tool_registry = ToolRegistry()
            self.register_lunar_tools()
            
            # Initialize agent with memory
            self.ai_agent = Agent(
                cognitive_core=self.cognitive_core,
                tool_registry=self.tool_registry,
                db_path="./lunabot_memory"
            )
            
            self.ai_available = True
            self.get_logger().info("✅ AI system initialized successfully")
            
        except Exception as e:
            self.get_logger().warn(f"AI system initialization failed: {e}")
            self.get_logger().info("Running in simulation mode")
            self.ai_available = False
            self.ai_agent = None
    
    def register_lunar_tools(self):
        """Register lunar mission-specific tools for the AI agent"""
        
        class NavigationTool:
            def __init__(self, node):
                self.node = node
            
            def execute(self, command: str, location: str = None):
                """Execute navigation commands"""
                if location:
                    cmd = f"goto {location}"
                else:
                    cmd = command
                
                msg = String()
                msg.data = cmd
                self.node.mission_command_pub.publish(msg)
                return f"Navigation command executed: {cmd}"
        
        class PatrolTool:
            def __init__(self, node):
                self.node = node
            
            def execute(self, action: str):
                """Execute patrol commands"""
                msg = String()
                msg.data = action
                self.node.patrol_command_pub.publish(msg)
                return f"Patrol command executed: {action}"
        
        class AnalysisTool:
            def __init__(self, node):
                self.node = node
            
            def execute(self, data_type: str):
                """Analyze mission data"""
                if data_type == "habitat":
                    return self.analyze_habitat_data()
                elif data_type == "environmental":
                    return self.analyze_environmental_data()
                else:
                    return "Analysis complete"
            
            def analyze_habitat_data(self):
                context = self.node.mission_context
                return f"Habitat status: {context.habitat_status}, Anomalies: {len(context.detected_anomalies)}"
            
            def analyze_environmental_data(self):
                context = self.node.mission_context
                return f"Environmental conditions: {context.environmental_conditions}"
        
        # Register tools
        if hasattr(self, 'tool_registry'):
            self.tool_registry.register_tool("navigation", NavigationTool(self))
            self.tool_registry.register_tool("patrol", PatrolTool(self))
            self.tool_registry.register_tool("analysis", AnalysisTool(self))
    
    def habitat_status_callback(self, msg):
        """Process habitat status updates"""
        try:
            status_data = json.loads(msg.data)
            self.mission_context.habitat_status = status_data.get('overall_status', 'unknown')
            self.mission_context.environmental_conditions = status_data.get('environmental_parameters', {})
            
            # Trigger AI analysis if critical status
            if self.mission_context.habitat_status in ['WARNING', 'EMERGENCY']:
                self.queue_ai_analysis("habitat_critical", status_data)
                
        except json.JSONDecodeError:
            pass
    
    def patrol_status_callback(self, msg):
        """Process patrol status updates"""
        try:
            patrol_data = json.loads(msg.data)
            
            # Update mission context
            if 'due_tasks' in patrol_data:
                due_tasks = patrol_data['due_tasks']
                if due_tasks > 5:  # Many overdue tasks
                    self.queue_ai_analysis("maintenance_backlog", patrol_data)
                    
        except json.JSONDecodeError:
            pass
    
    def navigation_status_callback(self, msg):
        """Process navigation status updates"""
        status = msg.data
        if "EMERGENCY_STOP" in status:
            self.queue_ai_analysis("navigation_emergency", {"status": status})
    
    def anomaly_alert_callback(self, msg):
        """Process anomaly alerts"""
        try:
            anomaly_data = json.loads(msg.data)
            self.mission_context.detected_anomalies.append(anomaly_data)
            
            # Keep only recent anomalies
            if len(self.mission_context.detected_anomalies) > 10:
                self.mission_context.detected_anomalies.pop(0)
            
            # Trigger immediate AI analysis for high severity anomalies
            if anomaly_data.get('severity') == 'high':
                self.queue_ai_analysis("high_severity_anomaly", anomaly_data)
                
        except json.JSONDecodeError:
            pass
    
    def diagnostics_callback(self, msg):
        """Process system diagnostics"""
        error_count = sum(1 for status in msg.status if status.level >= 2)
        
        if error_count > 3:  # Multiple system errors
            diagnostic_summary = {
                'error_count': error_count,
                'errors': [status.name for status in msg.status if status.level >= 2]
            }
            self.queue_ai_analysis("system_errors", diagnostic_summary)
    
    def sensor_data_callback(self, msg):
        """Process sensor data for situational awareness"""
        # Analyze laser scan for environmental changes
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
        
        if len(valid_ranges) > 0:
            # Detect significant environmental changes
            avg_distance = np.mean(valid_ranges)
            if avg_distance < 1.0:  # Very close obstacles
                self.queue_ai_analysis("obstacle_detection", {"avg_distance": avg_distance})
    
    def queue_ai_analysis(self, analysis_type: str, data: Dict):
        """Queue AI analysis task"""
        analysis_task = {
            'timestamp': datetime.now().isoformat(),
            'type': analysis_type,
            'data': data,
            'priority': self.get_analysis_priority(analysis_type)
        }
        
        # Add to processing queue (simplified)
        if not hasattr(self, 'analysis_queue'):
            self.analysis_queue = []
        
        self.analysis_queue.append(analysis_task)
        self.get_logger().info(f"Queued AI analysis: {analysis_type}")
    
    def get_analysis_priority(self, analysis_type: str) -> int:
        """Get priority level for analysis type"""
        priority_map = {
            'habitat_critical': 5,
            'navigation_emergency': 5,
            'high_severity_anomaly': 4,
            'system_errors': 3,
            'maintenance_backlog': 2,
            'obstacle_detection': 2,
            'routine_analysis': 1
        }
        return priority_map.get(analysis_type, 1)
    
    def decision_loop(self):
        """Main AI decision making loop"""
        # Process queued analyses
        if hasattr(self, 'analysis_queue') and self.analysis_queue:
            # Sort by priority
            self.analysis_queue.sort(key=lambda x: x['priority'], reverse=True)
            
            # Process highest priority analysis
            analysis_task = self.analysis_queue.pop(0)
            self.process_ai_analysis(analysis_task)
        
        # Routine decision making
        self.make_routine_decisions()
    
    def process_ai_analysis(self, analysis_task: Dict):
        """Process AI analysis task"""
        analysis_type = analysis_task['type']
        data = analysis_task['data']
        
        self.get_logger().info(f"Processing AI analysis: {analysis_type}")
        
        if self.ai_available and self.ai_agent:
            # Use actual AI agent
            decision = self.get_ai_decision(analysis_type, data)
        else:
            # Use rule-based fallback
            decision = self.get_fallback_decision(analysis_type, data)
        
        # Execute decision
        if decision:
            self.execute_decision(decision)
            self.log_decision(analysis_type, decision, data)
    
    def get_ai_decision(self, analysis_type: str, data: Dict) -> Optional[Dict]:
        """Get decision from AI agent"""
        try:
            # Construct prompt for AI
            prompt = self.construct_ai_prompt(analysis_type, data)
            
            # Get AI response
            action = self.ai_agent._think(prompt)
            outcome = self.ai_agent._act(action)
            
            # Parse AI decision
            decision = self.parse_ai_response(outcome, analysis_type)
            return decision
            
        except Exception as e:
            self.get_logger().error(f"AI decision failed: {e}")
            return self.get_fallback_decision(analysis_type, data)
    
    def construct_ai_prompt(self, analysis_type: str, data: Dict) -> str:
        """Construct prompt for AI analysis"""
        context = f"""
        Lunar Habitat Mission Context:
        - Mission Phase: {self.mission_context.current_mission_phase}
        - Habitat Status: {self.mission_context.habitat_status}
        - Environmental Conditions: {self.mission_context.environmental_conditions}
        - Recent Anomalies: {len(self.mission_context.detected_anomalies)}
        - Communication Window: {self.mission_context.communication_window}
        
        Analysis Type: {analysis_type}
        Data: {json.dumps(data, indent=2)}
        
        As the AI system for a lunar habitat robot, analyze this situation and recommend:
        1. Immediate actions required
        2. Priority level (1-5)
        3. Risk assessment
        4. Resource requirements
        5. Alternative options
        
        Provide response in JSON format with clear action recommendations.
        """
        
        return prompt
    
    def parse_ai_response(self, outcome: Dict, analysis_type: str) -> Dict:
        """Parse AI response into actionable decision"""
        try:
            # Extract text response
            response_text = outcome.get('data', {}).get('text_data', '')
            
            # Try to parse JSON response
            if '{' in response_text and '}' in response_text:
                json_start = response_text.find('{')
                json_end = response_text.rfind('}') + 1
                json_text = response_text[json_start:json_end]
                ai_decision = json.loads(json_text)
            else:
                # Fallback parsing
                ai_decision = {
                    'action': 'analyze_further',
                    'priority': 3,
                    'reasoning': response_text
                }
            
            return ai_decision
            
        except Exception as e:
            self.get_logger().warn(f"Failed to parse AI response: {e}")
            return self.get_fallback_decision(analysis_type, {})
    
    def get_fallback_decision(self, analysis_type: str, data: Dict) -> Dict:
        """Get rule-based fallback decision"""
        fallback_decisions = {
            'habitat_critical': {
                'action': 'emergency_return',
                'priority': 5,
                'reasoning': 'Critical habitat status detected, returning to base'
            },
            'navigation_emergency': {
                'action': 'emergency_stop',
                'priority': 5,
                'reasoning': 'Navigation emergency detected, stopping immediately'
            },
            'high_severity_anomaly': {
                'action': 'investigate_anomaly',
                'priority': 4,
                'reasoning': 'High severity anomaly requires immediate investigation'
            },
            'system_errors': {
                'action': 'diagnostic_check',
                'priority': 3,
                'reasoning': 'Multiple system errors require diagnostic check'
            },
            'maintenance_backlog': {
                'action': 'start_patrol',
                'priority': 2,
                'reasoning': 'Maintenance backlog requires patrol mission'
            },
            'obstacle_detection': {
                'action': 'cautious_navigation',
                'priority': 2,
                'reasoning': 'Obstacles detected, proceed with caution'
            }
        }
        
        return fallback_decisions.get(analysis_type, {
            'action': 'continue_mission',
            'priority': 1,
            'reasoning': 'No specific action required'
        })
    
    def execute_decision(self, decision: Dict):
        """Execute AI decision"""
        action = decision.get('action', 'continue_mission')
        priority = decision.get('priority', 1)
        
        self.get_logger().info(f"Executing AI decision: {action} (Priority: {priority})")
        
        # Map actions to ROS commands
        action_map = {
            'emergency_return': 'return_home',
            'emergency_stop': 'emergency_stop',
            'start_patrol': 'start_patrol',
            'investigate_anomaly': 'force_maintenance',
            'diagnostic_check': 'system_check',
            'cautious_navigation': 'slow_mode'
        }
        
        ros_command = action_map.get(action, action)
        
        # Publish appropriate command
        if action in ['emergency_return', 'emergency_stop', 'return_home']:
            msg = String()
            msg.data = ros_command
            self.mission_command_pub.publish(msg)
        elif action in ['start_patrol', 'force_maintenance']:
            msg = String()
            msg.data = ros_command
            self.patrol_command_pub.publish(msg)
        
        # Log decision execution
        decision['executed_at'] = datetime.now().isoformat()
        decision['ros_command'] = ros_command
    
    def make_routine_decisions(self):
        """Make routine autonomous decisions"""
        # Check if any routine decisions are needed
        current_time = datetime.now()
        
        # Example routine decisions
        if self.mission_context.habitat_status == 'OPERATIONAL':
            # Check if patrol should be started
            if not hasattr(self, 'last_patrol_check'):
                self.last_patrol_check = current_time
                
            time_since_patrol = (current_time - self.last_patrol_check).total_seconds()
            if time_since_patrol > 3600:  # 1 hour
                decision = {
                    'action': 'start_patrol',
                    'priority': 2,
                    'reasoning': 'Routine patrol due',
                    'type': 'routine'
                }
                self.execute_decision(decision)
                self.last_patrol_check = current_time
    
    def learning_loop(self):
        """AI learning and adaptation loop"""
        if not self.ai_available:
            return
        
        # Analyze recent decisions and outcomes
        self.analyze_decision_effectiveness()
        
        # Update AI knowledge base
        self.update_knowledge_base()
        
        # Publish learning insights
        self.publish_learning_insights()
    
    def analyze_decision_effectiveness(self):
        """Analyze effectiveness of recent AI decisions"""
        # Simplified effectiveness analysis
        recent_decisions = [d for d in self.decision_history 
                          if (datetime.now() - datetime.fromisoformat(d['timestamp'])).total_seconds() < 3600]
        
        if recent_decisions:
            effectiveness_score = self.calculate_effectiveness_score(recent_decisions)
            
            learning_data = {
                'timestamp': datetime.now().isoformat(),
                'decisions_analyzed': len(recent_decisions),
                'effectiveness_score': effectiveness_score,
                'insights': self.generate_learning_insights(recent_decisions)
            }
            
            self.learning_data.append(learning_data)
    
    def calculate_effectiveness_score(self, decisions: List[Dict]) -> float:
        """Calculate effectiveness score for decisions"""
        # Simplified scoring based on outcomes
        total_score = 0
        for decision in decisions:
            # Score based on priority and execution success
            priority_weight = decision.get('priority', 1) / 5.0
            execution_success = 1.0 if 'executed_at' in decision else 0.5
            total_score += priority_weight * execution_success
        
        return total_score / len(decisions) if decisions else 0.0
    
    def generate_learning_insights(self, decisions: List[Dict]) -> List[str]:
        """Generate learning insights from decision analysis"""
        insights = []
        
        # Analyze decision patterns
        action_counts = {}
        for decision in decisions:
            action = decision.get('action', 'unknown')
            action_counts[action] = action_counts.get(action, 0) + 1
        
        # Generate insights
        if action_counts:
            most_common_action = max(action_counts, key=action_counts.get)
            insights.append(f"Most common action: {most_common_action} ({action_counts[most_common_action]} times)")
        
        high_priority_decisions = [d for d in decisions if d.get('priority', 1) >= 4]
        if high_priority_decisions:
            insights.append(f"High priority decisions: {len(high_priority_decisions)}")
        
        return insights
    
    def update_knowledge_base(self):
        """Update AI knowledge base with new experiences"""
        if self.ai_available and self.ai_agent and hasattr(self.ai_agent, 'memory'):
            # Store recent experiences in AI memory
            for learning_item in self.learning_data[-5:]:  # Last 5 learning items
                experience = {
                    'type': 'mission_experience',
                    'data': learning_item,
                    'context': {
                        'habitat_status': self.mission_context.habitat_status,
                        'mission_phase': self.mission_context.current_mission_phase
                    }
                }
                
                try:
                    self.ai_agent.memory.store_experience(experience)
                except Exception as e:
                    self.get_logger().warn(f"Failed to store experience: {e}")
    
    def publish_learning_insights(self):
        """Publish AI learning insights"""
        if self.learning_data:
            latest_learning = self.learning_data[-1]
            
            insights_msg = String()
            insights_msg.data = json.dumps(latest_learning, indent=2)
            self.learning_insights_pub.publish(insights_msg)
    
    def log_decision(self, analysis_type: str, decision: Dict, data: Dict):
        """Log AI decision for analysis"""
        decision_log = {
            'timestamp': datetime.now().isoformat(),
            'analysis_type': analysis_type,
            'decision': decision,
            'input_data': data,
            'mission_context': {
                'habitat_status': self.mission_context.habitat_status,
                'mission_phase': self.mission_context.current_mission_phase
            }
        }
        
        self.decision_history.append(decision_log)
        
        # Keep only recent decisions
        if len(self.decision_history) > 100:
            self.decision_history.pop(0)
        
        # Publish decision log
        log_msg = String()
        log_msg.data = json.dumps(decision_log, indent=2)
        self.decision_log_pub.publish(log_msg)
    
    def publish_ai_status(self):
        """Publish AI system status"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'ai_available': self.ai_available,
            'mission_phase': self.mission_context.current_mission_phase,
            'habitat_status': self.mission_context.habitat_status,
            'recent_decisions': len([d for d in self.decision_history 
                                   if (datetime.now() - datetime.fromisoformat(d['timestamp'])).total_seconds() < 3600]),
            'learning_items': len(self.learning_data),
            'active_anomalies': len(self.mission_context.detected_anomalies),
            'analysis_queue_size': len(getattr(self, 'analysis_queue', []))
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.ai_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    ai_decision_engine = AIDecisionEngine()
    
    try:
        rclpy.spin(ai_decision_engine)
    except KeyboardInterrupt:
        pass
    finally:
        ai_decision_engine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
