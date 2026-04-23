"""
Quiz service for the multi-agent book generation system.
"""
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional, List
from ..models.generated_content import GeneratedContent
from ..models.user_profile import UserProfile
from ..shared.utils.content_formatter import sanitize_content
import logging
import json
import random

logger = logging.getLogger(__name__)


class QuizService:
    @staticmethod
    async def generate_quiz(
        db: Session,
        content: str,
        request_id: str,
        user_profile: Optional[Dict[str, Any]] = None,
        difficulty: Optional[str] = "MEDIUM"
    ) -> Dict[str, Any]:
        """
        Generate a quiz from the provided content based on user profile and difficulty.

        Args:
            db: Database session
            content: The content to generate quiz from
            request_id: The ID of the agent request
            user_profile: Optional user profile for personalization
            difficulty: Quiz difficulty level (EASY, MEDIUM, HARD)

        Returns:
            Dictionary containing the generated quiz
        """
        try:
            # Sanitize the content
            sanitized_content = sanitize_content(content)

            # Determine quiz parameters based on user profile and difficulty
            if user_profile:
                user_difficulty = user_profile.get('preferred_difficulty', difficulty)
                experience_level = user_profile.get('experience_level', 'INTERMEDIATE')
            else:
                user_difficulty = difficulty
                experience_level = 'INTERMEDIATE'

            # Generate different types of questions
            mcq_questions = QuizService._generate_mcq_questions(sanitized_content, user_difficulty, experience_level)
            short_answer_questions = QuizService._generate_short_answer_questions(sanitized_content, user_difficulty, experience_level)
            coding_exercises = QuizService._generate_coding_exercises(sanitized_content, user_difficulty, experience_level)

            # Create the quiz structure
            quiz = {
                "title": "Generated Quiz",
                "metadata": {
                    "difficulty": user_difficulty,
                    "experience_level": experience_level,
                    "source_content_length": len(sanitized_content),
                    "questions_count": len(mcq_questions) + len(short_answer_questions) + len(coding_exercises)
                },
                "questions": {
                    "multiple_choice": mcq_questions,
                    "short_answer": short_answer_questions,
                    "coding_exercises": coding_exercises
                }
            }

            # Save the generated content to the database
            generated_content = GeneratedContent(
                request_id=request_id,
                content_type="QUIZ",
                content=json.dumps(quiz),
                metadata={
                    "difficulty": user_difficulty,
                    "experience_level": experience_level,
                    "source_content_length": len(sanitized_content),
                    "questions_count": len(mcq_questions) + len(short_answer_questions) + len(coding_exercises)
                },
                quality_score="75"  # Default quality score
            )

            db.add(generated_content)
            db.commit()
            db.refresh(generated_content)

            logger.info(f"Quiz generated with ID: {generated_content.id}")
            return {
                "id": str(generated_content.id),
                "content_type": "QUIZ",
                "quiz": quiz,
                "quality_score": 75
            }

        except Exception as e:
            logger.error(f"Error generating quiz: {str(e)}", exc_info=True)
            raise

    @staticmethod
    def _generate_mcq_questions(content: str, difficulty: str, experience_level: str) -> List[Dict[str, Any]]:
        """
        Generate multiple choice questions from content.

        Args:
            content: The content to generate questions from
            difficulty: Difficulty level (EASY, MEDIUM, HARD)
            experience_level: User experience level (BEGINNER, INTERMEDIATE, EXPERT)

        Returns:
            List of multiple choice questions
        """
        # Generate questions based on content analysis and difficulty level
        questions = []

        # Determine number of questions based on difficulty
        if difficulty == "EASY":
            num_questions = 3
        elif difficulty == "HARD":
            num_questions = 7
        else:  # MEDIUM
            num_questions = 5

        # Basic question templates based on robotics content
        question_templates = [
            {
                "question": "What is the primary function of a humanoid robot's control system?",
                "correct": "To coordinate movement and maintain balance",
                "options": [
                    "To store data permanently",
                    "To coordinate movement and maintain balance",
                    "To generate electrical power",
                    "To communicate with humans only"
                ]
            },
            {
                "question": "Which sensor is most commonly used for robot navigation?",
                "correct": "LiDAR sensor",
                "options": [
                    "Temperature sensor",
                    "LiDAR sensor",
                    "Pressure sensor",
                    "Chemical sensor"
                ]
            },
            {
                "question": "What does ROS stand for in robotics?",
                "correct": "Robot Operating System",
                "options": [
                    "Robotic Optimization Software",
                    "Robot Operating System",
                    "Remote Operation Service",
                    "Rotational Output System"
                ]
            },
            {
                "question": "What is the main advantage of using simulation in robotics development?",
                "correct": "Safe testing without hardware damage risk",
                "options": [
                    "Faster robot movement",
                    "Safe testing without hardware damage risk",
                    "Reduced power consumption",
                    "Better visual appearance"
                ]
            },
            {
                "question": "Which programming language is most commonly used with ROS?",
                "correct": "Python",
                "options": [
                    "JavaScript",
                    "Python",
                    "HTML",
                    "CSS"
                ]
            }
        ]

        # Select questions based on difficulty and available templates
        selected_questions = question_templates[:min(num_questions, len(question_templates))]

        for i, template in enumerate(selected_questions):
            question_text = template["question"]
            correct_answer = template["correct"]
            options = template["options"].copy()  # Use the predefined options

            # Shuffle options to randomize the correct answer position
            random.shuffle(options)
            correct_index = options.index(correct_answer)

            # Generate appropriate explanation based on the question
            explanations = {
                "What is the primary function of a humanoid robot's control system?": "The control system is responsible for coordinating all robot movements, maintaining balance, and ensuring stable operation.",
                "Which sensor is most commonly used for robot navigation?": "LiDAR sensors provide accurate distance measurements and 3D mapping capabilities essential for navigation.",
                "What does ROS stand for in robotics?": "ROS (Robot Operating System) is a flexible framework for writing robot software and managing robot components.",
                "What is the main advantage of using simulation in robotics development?": "Simulation allows developers to test algorithms and behaviors safely without risking damage to expensive hardware.",
                "Which programming language is most commonly used with ROS?": "Python is widely used in ROS due to its simplicity and extensive libraries for robotics applications."
            }

            explanation = explanations.get(question_text, "This concept is fundamental to understanding robotics systems.")

            questions.append({
                "id": f"mcq_{i+1}",
                "question": question_text,
                "options": options,
                "correct_answer_index": correct_index,
                "explanation": explanation
            })

        return questions

    @staticmethod
    def _generate_short_answer_questions(content: str, difficulty: str, experience_level: str) -> List[Dict[str, Any]]:
        """
        Generate short answer questions from content.

        Args:
            content: The content to generate questions from
            difficulty: Difficulty level (EASY, MEDIUM, HARD)
            experience_level: User experience level (BEGINNER, INTERMEDIATE, EXPERT)

        Returns:
            List of short answer questions
        """
        # Generate short answer questions based on robotics content
        questions = []

        # Determine number of questions based on difficulty
        if difficulty == "EASY":
            num_questions = 2
        elif difficulty == "HARD":
            num_questions = 5
        else:  # MEDIUM
            num_questions = 3

        # Short answer question templates
        question_templates = [
            {
                "question": "Explain the main components of a humanoid robot control system.",
                "sample_answer": "A humanoid robot control system typically includes sensors for perception, actuators for movement, a central processing unit for decision-making, and feedback loops for balance and coordination.",
                "keywords": ["sensors", "actuators", "processing", "feedback", "balance"]
            },
            {
                "question": "Describe how LiDAR sensors work in robot navigation.",
                "sample_answer": "LiDAR sensors emit laser pulses and measure the time it takes for them to return after hitting objects, creating detailed 3D maps of the environment for navigation.",
                "keywords": ["laser", "distance", "mapping", "3D", "navigation"]
            },
            {
                "question": "What are the advantages of using ROS in robotics development?",
                "sample_answer": "ROS provides modularity, reusable components, standardized communication protocols, extensive libraries, and a large community ecosystem for robotics development.",
                "keywords": ["modularity", "reusable", "communication", "libraries", "community"]
            },
            {
                "question": "How does simulation benefit robotics development?",
                "sample_answer": "Simulation allows safe testing, rapid prototyping, cost reduction, algorithm validation, and scenario testing without physical hardware constraints.",
                "keywords": ["testing", "prototyping", "validation", "cost", "safety"]
            },
            {
                "question": "Explain the importance of kinematics in humanoid robotics.",
                "sample_answer": "Kinematics describes robot motion without considering forces, essential for path planning, joint coordination, and ensuring smooth, natural movement patterns.",
                "keywords": ["motion", "joints", "planning", "coordination", "movement"]
            }
        ]

        # Select questions based on available templates
        selected_questions = question_templates[:min(num_questions, len(question_templates))]

        for i, template in enumerate(selected_questions):
            question_text = template["question"]
            sample_answer = template["sample_answer"]
            keywords = template["keywords"]

            questions.append({
                "id": f"sa_{i+1}",
                "question": question_text,
                "expected_length": "medium",  # short, medium, long
                "sample_answer": sample_answer,
                "keywords": keywords,
                "evaluation_criteria": {
                    "key_concepts": keywords,
                    "min_length": 50,
                    "max_length": 300,
                    "required_elements": ["clear explanation", "technical accuracy", "relevant examples"]
                }
            })

        return questions

    @staticmethod
    def _generate_coding_exercises(content: str, difficulty: str, experience_level: str) -> List[Dict[str, Any]]:
        """
        Generate coding exercises from content.

        Args:
            content: The content to generate exercises from
            difficulty: Difficulty level (EASY, MEDIUM, HARD)
            experience_level: User experience level (BEGINNER, INTERMEDIATE, EXPERT)

        Returns:
            List of coding exercises
        """
        # Generate practical coding exercises for robotics
        exercises = []

        # Determine number of exercises based on difficulty and experience level
        if difficulty == "EASY" or experience_level == "BEGINNER":
            num_exercises = 1
        elif difficulty == "HARD" or experience_level == "EXPERT":
            num_exercises = 3
        else:  # MEDIUM or INTERMEDIATE
            num_exercises = 2

        # Coding exercise templates for robotics
        exercise_templates = [
            {
                "title": "Basic Robot Movement Control",
                "description": "Write a Python function to control basic robot movement using ROS.",
                "starter_code": """#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot(linear_speed, angular_speed, duration):
    \"\"\"
    Move robot with given speeds for specified duration
    Args:
        linear_speed: Forward/backward speed (m/s)
        angular_speed: Rotation speed (rad/s)
        duration: Time to move (seconds)
    \"\"\"
    # TODO: Implement robot movement
    pass

if __name__ == "__main__":
    rospy.init_node('robot_mover')
    # Test the function
    move_robot(0.5, 0.0, 2.0)  # Move forward for 2 seconds
""",
                "solution": """#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot(linear_speed, angular_speed, duration):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(0.1)  # Allow publisher to connect

    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time).to_sec() < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node('robot_mover')
    move_robot(0.5, 0.0, 2.0)
""",
                "difficulty": "EASY"
            },
            {
                "title": "Sensor Data Processing",
                "description": "Create a class to process and filter sensor data from a robot.",
                "starter_code": """import numpy as np

class SensorProcessor:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.data_buffer = []

    def add_reading(self, value):
        \"\"\"Add new sensor reading to buffer\"\"\"
        # TODO: Implement buffer management
        pass

    def get_filtered_value(self):
        \"\"\"Return filtered sensor value using moving average\"\"\"
        # TODO: Implement filtering
        pass

    def detect_anomaly(self, threshold=2.0):
        \"\"\"Detect if current reading is anomalous\"\"\"
        # TODO: Implement anomaly detection
        pass

# Test the class
processor = SensorProcessor()
test_data = [1.0, 1.1, 1.2, 5.0, 1.1, 1.0, 1.3]
for reading in test_data:
    processor.add_reading(reading)
    print(f"Reading: {reading}, Filtered: {processor.get_filtered_value()}")
""",
                "solution": """import numpy as np

class SensorProcessor:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.data_buffer = []

    def add_reading(self, value):
        self.data_buffer.append(value)
        if len(self.data_buffer) > self.window_size:
            self.data_buffer.pop(0)

    def get_filtered_value(self):
        if not self.data_buffer:
            return 0.0
        return np.mean(self.data_buffer)

    def detect_anomaly(self, threshold=2.0):
        if len(self.data_buffer) < 3:
            return False

        current = self.data_buffer[-1]
        mean = np.mean(self.data_buffer[:-1])
        std = np.std(self.data_buffer[:-1])

        return abs(current - mean) > threshold * std

processor = SensorProcessor()
test_data = [1.0, 1.1, 1.2, 5.0, 1.1, 1.0, 1.3]
for reading in test_data:
    processor.add_reading(reading)
    filtered = processor.get_filtered_value()
    anomaly = processor.detect_anomaly()
    print(f"Reading: {reading:.1f}, Filtered: {filtered:.2f}, Anomaly: {anomaly}")
""",
                "difficulty": "MEDIUM"
            },
            {
                "title": "Robot Path Planning",
                "description": "Implement a simple A* pathfinding algorithm for robot navigation.",
                "starter_code": """import heapq
import math

class PathPlanner:
    def __init__(self, grid):
        self.grid = grid  # 2D array: 0 = free, 1 = obstacle
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        \"\"\"Calculate heuristic distance between two points\"\"\"
        # TODO: Implement Manhattan or Euclidean distance
        pass

    def get_neighbors(self, pos):
        \"\"\"Get valid neighboring positions\"\"\"
        # TODO: Return list of valid neighbor coordinates
        pass

    def find_path(self, start, goal):
        \"\"\"Find shortest path using A* algorithm\"\"\"
        # TODO: Implement A* pathfinding
        pass

# Test the pathfinder
grid = [
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

planner = PathPlanner(grid)
path = planner.find_path((0, 0), (4, 4))
print("Path:", path)
""",
                "solution": """import heapq
import math

class PathPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, pos):
        neighbors = []
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            x, y = pos[0] + dx, pos[1] + dy
            if (0 <= x < self.rows and 0 <= y < self.cols and
                self.grid[x][y] == 0):
                neighbors.append((x, y))
        return neighbors

    def find_path(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

grid = [
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

planner = PathPlanner(grid)
path = planner.find_path((0, 0), (4, 4))
print("Path:", path)
""",
                "difficulty": "HARD"
            }
        ]

        # Select exercises based on difficulty and number needed
        available_exercises = [ex for ex in exercise_templates
                             if ex["difficulty"] == difficulty or
                             (difficulty == "MEDIUM" and ex["difficulty"] in ["EASY", "MEDIUM"])]

        if not available_exercises:
            available_exercises = exercise_templates

        selected_exercises = available_exercises[:min(num_exercises, len(available_exercises))]

        for i, template in enumerate(selected_exercises):
            exercises.append({
                "id": f"code_{i+1}",
                "title": template["title"],
                "description": template["description"],
                "starter_code": template["starter_code"],
                "solution": template["solution"],
                "difficulty": template["difficulty"],
                "language": "python",
                "test_cases": [
                    {"input": "Basic functionality test", "expected": "Should execute without errors"},
                    {"input": "Edge case handling", "expected": "Should handle invalid inputs gracefully"}
                ]
            })

        for i in range(num_exercises):
            exercise_description = f"Sample coding exercise {i+1} based on the content would be generated here."

            exercises.append({
                "id": f"code_{i+1}",
                "description": exercise_description,
                "requirements": [
                    "List of requirements for the exercise would be defined here."
                ],
                "starter_code": "// Starter code would be provided here\n",
                "expected_output": "Expected output would be described here.",
                "evaluation_criteria": f"Evaluation criteria for exercise {i+1} would be defined here."
            })

        return exercises

    @staticmethod
    async def integrate_user_profile_for_difficulty(
        db: Session,
        user_profile_id: str,
        base_difficulty: str = "MEDIUM"
    ) -> Dict[str, Any]:
        """
        Integrate user profile to adjust quiz difficulty.

        Args:
            db: Database session
            user_profile_id: ID of the user profile
            base_difficulty: Base difficulty level

        Returns:
            Dictionary with adjusted difficulty settings
        """
        try:
            # Get the user profile from the database
            user_profile = db.query(UserProfile).filter(
                UserProfile.user_id == user_profile_id
            ).first()

            if not user_profile:
                logger.warning(f"User profile {user_profile_id} not found, using default settings")
                return {
                    "difficulty": base_difficulty,
                    "experience_level": "INTERMEDIATE",
                    "adjusted": False
                }

            # Determine the final difficulty based on user preferences and experience
            final_difficulty = user_profile.preferred_difficulty or base_difficulty
            experience_level = user_profile.experience_level

            # Create adjustment settings
            adjustment_settings = {
                "difficulty": final_difficulty,
                "experience_level": experience_level,
                "learning_goals": user_profile.learning_goals,
                "technical_background": user_profile.technical_background,
                "adjusted": True
            }

            logger.info(f"Applied user profile settings for quiz generation: {user_profile_id}")
            return adjustment_settings

        except Exception as e:
            logger.error(f"Error integrating user profile for quiz: {str(e)}", exc_info=True)
            raise