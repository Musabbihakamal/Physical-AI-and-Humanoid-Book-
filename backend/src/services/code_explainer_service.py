"""
Code explainer service for the multi-agent book generation system.
"""
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional, List
from ..models.generated_content import GeneratedContent
from ..utils.content_formatter import sanitize_content
import logging
import re
import json

logger = logging.getLogger(__name__)


class CodeExplainerService:
    @staticmethod
    async def explain_code(
        db: Session,
        code: str,
        language: Optional[str] = None,
        request_id: str = None,
        user_profile: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Explain the provided code with focus on ROS 2 and Isaac Sim commands.

        Args:
            db: Database session
            code: The code to explain
            language: Programming language of the code
            request_id: The ID of the agent request
            user_profile: Optional user profile for personalization

        Returns:
            Dictionary containing the code explanation
        """
        try:
            # Sanitize the code
            sanitized_code = sanitize_content(code)

            # Analyze the code for ROS 2 and Isaac Sim patterns
            ros2_commands = CodeExplainerService._find_ros2_commands(sanitized_code)
            isaac_commands = CodeExplainerService._find_isaac_commands(sanitized_code)

            # Create a basic explanation structure
            explanation = {
                "overview": "Code explanation would be generated here based on the provided code.",
                "key_components": CodeExplainerService._analyze_code_structure(sanitized_code),
                "ros2_specifics": ros2_commands,
                "isaac_sim_specifics": isaac_commands,
                "technical_concepts": CodeExplainerService._identify_technical_concepts(sanitized_code),
                "usage_context": "How this code fits into the broader application would be explained here.",
                "learning_points": CodeExplainerService._extract_learning_points(sanitized_code),
                "code": sanitized_code,
                "language": language or "unknown"
            }

            # Create the explanation content
            explanation_content = {
                "title": "Code Explanation",
                "explanation": explanation,
                "original_code_length": len(code),
                "language": language
            }

            # Save the generated content to the database if request_id is provided
            if request_id:
                generated_content = GeneratedContent(
                    request_id=request_id,
                    content_type="CODE_EXPLANATION",
                    content=json.dumps(explanation_content),
                    metadata={
                        "original_code_length": len(code),
                        "language": language,
                        "ros2_commands_found": len(ros2_commands),
                        "isaac_commands_found": len(isaac_commands)
                    },
                    quality_score="75"  # Default quality score
                )

                db.add(generated_content)
                db.commit()
                db.refresh(generated_content)

                explanation_content["id"] = str(generated_content.id)

            logger.info(f"Code explanation generated for request: {request_id}")
            return {
                "id": request_id,
                "content_type": "CODE_EXPLANATION",
                "explanation": explanation_content,
                "quality_score": 75
            }

        except Exception as e:
            logger.error(f"Error explaining code: {str(e)}", exc_info=True)
            raise

    @staticmethod
    def _find_ros2_commands(code: str) -> List[str]:
        """
        Find ROS 2 specific commands, functions, and patterns in the code.

        Args:
            code: The code to analyze

        Returns:
            List of ROS 2 commands found
        """
        ros2_patterns = [
            r'rclcpp::init',  # ROS 2 C++ init
            r'rclpy\.init',  # ROS 2 Python init
            r'Node\(\s*["\'][^"\']*["\']\s*\)',  # Node creation
            r'rclcpp::Node',  # C++ Node class
            r'rclpy\.Node',  # Python Node class
            r'create_publisher',  # Publisher creation
            r'create_subscription',  # Subscription creation
            r'create_service',  # Service creation
            r'create_client',  # Client creation
            r'create_timer',  # Timer creation
            r'twist',  # Twist messages
            r'nav_msgs',  # Navigation messages
            r'std_msgs',  # Standard messages
            r'geometry_msgs',  # Geometry messages
            r'rclcpp::spin',  # ROS 2 spin
            r'rclpy\.spin',  # ROS 2 spin
            r'rclpy\.shutdown',  # ROS 2 shutdown
        ]

        found_commands = []
        for pattern in ros2_patterns:
            matches = re.findall(pattern, code, re.IGNORECASE)
            found_commands.extend(matches)

        # Remove duplicates while preserving order
        unique_commands = []
        for cmd in found_commands:
            if cmd not in unique_commands:
                unique_commands.append(cmd)

        return unique_commands

    @staticmethod
    def _find_isaac_commands(code: str) -> List[str]:
        """
        Find Isaac Sim specific commands, functions, and patterns in the code.

        Args:
            code: The code to analyze

        Returns:
            List of Isaac Sim commands found
        """
        isaac_patterns = [
            r'omni\.isaac\.core',  # Isaac Core
            r'omni\.isaac\.sensor',  # Isaac Sensors
            r'omni\.isaac\.robots',  # Isaac Robots
            r'omni\.isaac\.gym',  # Isaac Gym
            r'World\(\)',  # World class
            r'Vehicle\(\)',  # Vehicle class
            r'DifferentialController',  # Differential controller
            r'ArticulationView',  # Articulation view
            r'RigidContactView',  # Rigid contact view
            r'RayCaster',  # Ray caster
            r'IsaacSensor',  # Isaac sensor
            r'omni\.replicator',  # Replicator
            r'Annotator',  # Annotator
        ]

        found_commands = []
        for pattern in isaac_patterns:
            matches = re.findall(pattern, code, re.IGNORECASE)
            found_commands.extend(matches)

        # Remove duplicates while preserving order
        unique_commands = []
        for cmd in found_commands:
            if cmd not in unique_commands:
                unique_commands.append(cmd)

        return unique_commands

    @staticmethod
    def _analyze_code_structure(code: str) -> List[Dict[str, str]]:
        """
        Analyze the structure of the code to identify key components.

        Args:
            code: The code to analyze

        Returns:
            List of key components with descriptions
        """
        components = []

        # Look for class definitions
        class_pattern = r'(class\s+\w+\s*(?:\([^)]*\))?\s*:)'
        class_matches = re.findall(class_pattern, code)
        for match in class_matches:
            components.append({
                "type": "class",
                "name": re.search(r'class\s+(\w+)', match).group(1),
                "declaration": match.strip()
            })

        # Look for function definitions
        func_pattern = r'(def\s+\w+\s*\([^)]*\)\s*:|function\s+\w+\s*\([^)]*\)\s*{)'
        func_matches = re.findall(func_pattern, code)
        for match in func_matches:
            components.append({
                "type": "function",
                "name": re.search(r'(?:def|function)\s+(\w+)', match).group(1),
                "declaration": match.strip()
            })

        # Look for variable assignments that might be important
        var_pattern = r'(\w+\s*=\s*[^=\n]+)'
        var_matches = re.findall(var_pattern, code)
        for match in var_matches[:5]:  # Limit to first 5 to avoid noise
            if not re.match(r'(import|from|class|def|function)', match):
                components.append({
                    "type": "variable",
                    "name": match.split('=')[0].strip(),
                    "declaration": match.strip()
                })

        return components

    @staticmethod
    def _identify_technical_concepts(code: str) -> List[str]:
        """
        Identify key technical concepts in the code.

        Args:
            code: The code to analyze

        Returns:
            List of technical concepts
        """
        concepts = []

        # Look for common technical patterns
        if re.search(r'(publisher|subscriber|topic|message)', code, re.IGNORECASE):
            concepts.append("ROS 2 Communication Patterns")
        if re.search(r'(node|service|client)', code, re.IGNORECASE):
            concepts.append("ROS 2 Node Architecture")
        if re.search(r'(sensor|camera|lidar|imu)', code, re.IGNORECASE):
            concepts.append("Robot Perception")
        if re.search(r'(control|motor|actuator)', code, re.IGNORECASE):
            concepts.append("Robot Control")
        if re.search(r'(simulation|world|physics|render)', code, re.IGNORECASE):
            concepts.append("Simulation Concepts")
        if re.search(r'(gym|training|learning|agent)', code, re.IGNORECASE):
            concepts.append("Reinforcement Learning")

        return list(set(concepts))  # Remove duplicates

    @staticmethod
    def _extract_learning_points(code: str) -> List[str]:
        """
        Extract key learning points from the code.

        Args:
            code: The code to analyze

        Returns:
            List of learning points
        """
        learning_points = []

        # Add generic learning points based on code patterns
        if re.search(r'__init__', code):
            learning_points.append("Object initialization patterns")
        if re.search(r'async|await', code):
            learning_points.append("Asynchronous programming concepts")
        if re.search(r'try:|except|catch', code):
            learning_points.append("Error handling strategies")
        if re.search(r'for|while|loop', code, re.IGNORECASE):
            learning_points.append("Iteration and control flow")
        if re.search(r'if|elif|else', code):
            learning_points.append("Conditional logic")

        return learning_points

    @staticmethod
    async def parse_and_highlight_code(code: str, language: str = "python") -> Dict[str, Any]:
        """
        Parse code and identify important elements for highlighting.

        Args:
            code: The code to parse
            language: Programming language

        Returns:
            Dictionary with parsed code elements
        """
        try:
            # Find ROS 2 and Isaac Sim commands
            ros2_commands = CodeExplainerService._find_ros2_commands(code)
            isaac_commands = CodeExplainerService._find_isaac_commands(code)

            # Create a representation of the code with highlighted elements
            parsed_code = {
                "original": code,
                "language": language,
                "ros2_commands": ros2_commands,
                "isaac_commands": isaac_commands,
                "highlights": {
                    "ros2": ros2_commands,
                    "isaac": isaac_commands,
                    "functions": CodeExplainerService._analyze_code_structure(code)
                }
            }

            logger.info(f"Parsed and highlighted code with {len(ros2_commands)} ROS2 and {len(isaac_commands)} Isaac commands")
            return parsed_code

        except Exception as e:
            logger.error(f"Error parsing and highlighting code: {str(e)}", exc_info=True)
            raise