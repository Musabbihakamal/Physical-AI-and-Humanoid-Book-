"""
Glossary Maker agent implementation for the multi-agent book generation system.
"""
from typing import Dict, Any, Optional
from ...services.glossary_service import GlossaryService
from ...models.generated_content import GeneratedContent
from sqlalchemy.orm import Session
import logging

logger = logging.getLogger(__name__)


class GlossaryMaker:
    """
    Agent responsible for generating glossaries from content.
    """

    @staticmethod
    async def execute(
        db: Session,
        content: str,
        user_profile: Optional[Dict[str, Any]] = None,
        parameters: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Execute the Glossary Maker agent.

        Args:
            db: Database session
            content: Content to generate glossary from
            user_profile: Optional user profile for personalization
            parameters: Additional parameters for the agent

        Returns:
            Dictionary containing the generated glossary
        """
        try:
            # Generate the glossary using the service
            result = await GlossaryService.generate_glossary(
                db=db,
                content=content,
                request_id=None,  # Will be set by the caller
                user_profile=user_profile
            )

            logger.info("Glossary maker agent executed successfully")
            return result

        except Exception as e:
            logger.error(f"Error in Glossary Maker agent: {str(e)}", exc_info=True)
            raise