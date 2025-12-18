"""
Quiz Creator agent implementation for the multi-agent book generation system.
"""
from typing import Dict, Any, Optional
from ...services.quiz_service import QuizService
from sqlalchemy.orm import Session
import logging

logger = logging.getLogger(__name__)


class QuizCreator:
    """
    Agent responsible for generating quizzes from content.
    """

    @staticmethod
    async def execute(
        db: Session,
        content: str,
        user_profile: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Execute the Quiz Creator agent.

        Args:
            db: Database session
            content: Content to generate quiz from
            user_profile: Optional user profile for personalization
            parameters: Additional parameters for the agent

        Returns:
            Dictionary containing the generated quiz
        """
        try:
            # Determine difficulty from parameters or user profile
            difficulty = (parameters or {}).get('difficulty')
            if not difficulty and user_profile:
                difficulty = user_profile.get('preferred_difficulty')

            # Generate the quiz using the service
            result = await QuizService.generate_quiz(
                db=db,
                content=content,
                request_id=None,  # Will be set by the caller
                user_profile=user_profile,
                difficulty=difficulty
            )

            logger.info("Quiz creator agent executed successfully")
            return result

        except Exception as e:
            logger.error(f"Error in Quiz Creator agent: {str(e)}", exc_info=True)
            raise