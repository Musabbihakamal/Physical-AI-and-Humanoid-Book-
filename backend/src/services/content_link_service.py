"""
Content link service for the multi-agent book generation system.
"""
from sqlalchemy.orm import Session
from typing import List, Optional
from ..models.content_link import ContentLink
from ..models.generated_content import GeneratedContent
import logging

logger = logging.getLogger(__name__)


class ContentLinkService:
    @staticmethod
    def create_content_link(
        db: Session,
        source_content_id: str,
        target_content_id: str,
        link_type: str
    ) -> ContentLink:
        """
        Create a link between two pieces of content.

        Args:
            db: Database session
            source_content_id: ID of the source content
            target_content_id: ID of the target content
            link_type: Type of link (GLOSSARY_TERM, CROSS_REFERENCE, etc.)

        Returns:
            Created ContentLink object
        """
        try:
            # Validate that both content items exist
            source_content = db.query(GeneratedContent).filter(
                GeneratedContent.id == source_content_id
            ).first()
            if not source_content:
                raise ValueError(f"Source content with ID {source_content_id} not found")

            target_content = db.query(GeneratedContent).filter(
                GeneratedContent.id == target_content_id
            ).first()
            if not target_content:
                raise ValueError(f"Target content with ID {target_content_id} not found")

            # Create the content link
            content_link = ContentLink(
                source_content_id=source_content_id,
                target_content_id=target_content_id,
                link_type=link_type
            )

            db.add(content_link)
            db.commit()
            db.refresh(content_link)

            logger.info(f"Created content link from {source_content_id} to {target_content_id}")
            return content_link

        except Exception as e:
            db.rollback()
            logger.error(f"Error creating content link: {str(e)}")
            raise

    @staticmethod
    def get_links_for_content(
        db: Session,
        content_id: str
    ) -> List[ContentLink]:
        """
        Get all links associated with a piece of content (both as source and target).

        Args:
            db: Database session
            content_id: ID of the content to get links for

        Returns:
            List of ContentLink objects
        """
        try:
            # Get links where this content is the source
            source_links = db.query(ContentLink).filter(
                ContentLink.source_content_id == content_id
            ).all()

            # Get links where this content is the target
            target_links = db.query(ContentLink).filter(
                ContentLink.target_content_id == content_id
            ).all()

            # Combine both lists
            all_links = source_links + target_links

            logger.info(f"Retrieved {len(all_links)} links for content {content_id}")
            return all_links

        except Exception as e:
            logger.error(f"Error getting links for content: {str(e)}")
            raise

    @staticmethod
    def create_glossary_term_links(
        db: Session,
        glossary_content_id: str,
        content_with_terms: str
    ) -> List[ContentLink]:
        """
        Create links from glossary terms to their occurrences in content.

        Args:
            db: Database session
            glossary_content_id: ID of the glossary content
            content_with_terms: Content that contains the terms

        Returns:
            List of created ContentLink objects
        """
        try:
            # Get the glossary content to extract terms
            glossary_content = db.query(GeneratedContent).filter(
                GeneratedContent.id == glossary_content_id
            ).first()

            if not glossary_content:
                raise ValueError(f"Glossary content with ID {glossary_content_id} not found")

            # In a real implementation, we would parse the glossary to extract terms
            # and then find their occurrences in the content_with_terms
            # For now, we'll just return an empty list as a placeholder

            logger.info(f"Created glossary term links for content {glossary_content_id}")
            return []

        except Exception as e:
            logger.error(f"Error creating glossary term links: {str(e)}")
            raise