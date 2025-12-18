"""
Content link model for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
import uuid


class ContentLink(Base):
    __tablename__ = "content_links"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    source_content_id = Column(UUID_TYPE, ForeignKey("generated_contents.id"), nullable=False)
    target_content_id = Column(UUID_TYPE, ForeignKey("generated_contents.id"), nullable=False)
    link_type = Column(String, nullable=False)  # GLOSSARY_TERM, CROSS_REFERENCE, etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __init__(self,
                 source_content_id: str,
                 target_content_id: str,
                 link_type: str):
        # Validation
        valid_link_types = [
            "GLOSSARY_TERM", "CROSS_REFERENCE", "RELATED_CONTENT", "SEE_ALSO"
        ]
        if link_type not in valid_link_types:
            raise ValueError(f"link_type must be one of {valid_link_types}")

        self.source_content_id = source_content_id
        self.target_content_id = target_content_id
        self.link_type = link_type