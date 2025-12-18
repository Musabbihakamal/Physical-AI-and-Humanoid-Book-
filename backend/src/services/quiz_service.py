"""
Quiz service for the multi-agent book generation system.
"""
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional, List
from ..models.generated_content import GeneratedContent
from ..models.user_profile import UserProfile
from ..utils.content_formatter import sanitize_content
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
        # In a real implementation, this would use an LLM to generate questions
        # For now, we'll create placeholder questions

        questions = []

        # Determine number of questions based on difficulty
        if difficulty == "EASY":
            num_questions = 3
        elif difficulty == "HARD":
            num_questions = 7
        else:  # MEDIUM
            num_questions = 5

        for i in range(num_questions):
            question_text = f"Sample MCQ {i+1} based on the content would be generated here."
            correct_answer = f"Correct answer {i+1}"
            options = [
                correct_answer,
                f"Distractor 1 for question {i+1}",
                f"Distractor 2 for question {i+1}",
                f"Distractor 3 for question {i+1}"
            ]

            # Shuffle options to randomize the correct answer position
            random.shuffle(options)
            correct_index = options.index(correct_answer)

            questions.append({
                "id": f"mcq_{i+1}",
                "question": question_text,
                "options": options,
                "correct_answer_index": correct_index,
                "explanation": f"Explanation for question {i+1} would be provided here."
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
        # In a real implementation, this would use an LLM to generate questions
        # For now, we'll create placeholder questions

        questions = []

        # Determine number of questions based on difficulty
        if difficulty == "EASY":
            num_questions = 2
        elif difficulty == "HARD":
            num_questions = 5
        else:  # MEDIUM
            num_questions = 3

        for i in range(num_questions):
            question_text = f"Sample short answer question {i+1} based on the content would be generated here."

            questions.append({
                "id": f"sa_{i+1}",
                "question": question_text,
                "expected_length": "medium",  # short, medium, long
                "evaluation_criteria": f"Evaluation criteria for question {i+1} would be defined here."
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
        # In a real implementation, this would use an LLM to generate exercises
        # For now, we'll create placeholder exercises

        exercises = []

        # Determine number of exercises based on difficulty and experience level
        if difficulty == "EASY" or experience_level == "BEGINNER":
            num_exercises = 1
        elif difficulty == "HARD" or experience_level == "EXPERT":
            num_exercises = 3
        else:  # MEDIUM or INTERMEDIATE
            num_exercises = 2

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