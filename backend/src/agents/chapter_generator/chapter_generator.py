"""
Chapter Generator agent implementation for the multi-agent book generation system.
"""
from typing import Dict, Any, Optional
from ...services.chapter_service import ChapterService
from sqlalchemy.orm import Session
import logging

logger = logging.getLogger(__name__)


class ChapterGenerator:
    """
    Agent responsible for generating chapters based on module focus.
    """

    @staticmethod
    async def execute(
        db: Session,
        module_focus: str,
        outline: Optional[list] = None,
        user_profile: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Execute the Chapter Generator agent.

        Args:
            db: Database session
            module_focus: Main topic or focus of the chapter
            outline: Optional outline with section headings
            user_profile: Optional user profile for personalization
            parameters: Additional parameters for the agent

        Returns:
            Dictionary containing the generated chapter
        """
        try:
            # Generate the chapter using the service
            result = await ChapterService.generate_chapter(
                db=db,
                module_focus=module_focus,
                outline=outline,
                user_profile=user_profile,
                request_id=None  # Will be set by the caller
            )

            logger.info("Chapter generator agent executed successfully")
            return result

        except Exception as e:
            logger.error(f"Error in Chapter Generator agent: {str(e)}", exc_info=True)
            raise