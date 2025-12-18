"""
Code Explainer agent implementation for the multi-agent book generation system.
"""
from typing import Dict, Any, Optional
from ...services.code_explainer_service import CodeExplainerService
from sqlalchemy.orm import Session
import logging

logger = logging.getLogger(__name__)


class CodeExplainer:
    """
    Agent responsible for explaining code with focus on ROS 2 and Isaac Sim commands.
    """

    @staticmethod
    async def execute(
        db: Session,
        code: str,
        language: Optional[str] = None,
        user_profile: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Execute the Code Explainer agent.

        Args:
            db: Database session
            code: Code to explain
            language: Programming language of the code
            user_profile: Optional user profile for personalization
            parameters: Additional parameters for the agent

        Returns:
            Dictionary containing the code explanation
        """
        try:
            # Explain the code using the service
            result = await CodeExplainerService.explain_code(
                db=db,
                code=code,
                language=language,
                request_id=None,  # Will be set by the caller
                user_profile=user_profile
            )

            logger.info("Code explainer agent executed successfully")
            return result

        except Exception as e:
            logger.error(f"Error in Code Explainer agent: {str(e)}", exc_info=True)
            raise