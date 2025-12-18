"""
Generated content model for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Text, JSON, ForeignKey
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from typing import Dict, Any, Optional
import uuid


class GeneratedContent(Base):
    __tablename__ = "generated_contents"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    request_id = Column(UUID_TYPE, ForeignKey("agent_requests.id"), nullable=False)
    content_type = Column(String, nullable=False)  # CHAPTER, GLOSSARY, CODE_EXPLANATION, etc.
    content = Column(Text, nullable=False)  # the actual generated content
    metadata_obj = Column("metadata", JSON, default={})  # generation details, sources, quality metrics
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    quality_score = Column(String, default="50")  # 0-100 quality assessment

    def __init__(self,
                 request_id: str,
                 content_type: str,
                 content: str,
                 metadata: Optional[Dict[str, Any]] = None,
                 quality_score: str = "50"):
        import uuid
        # Validation
        valid_content_types = [
            "CHAPTER", "GLOSSARY", "CODE_EXPLANATION", "QUIZ", "DOCUMENTATION",
            "RESEARCH_SUMMARY", "EDITED_CONTENT", "RAG_COMPONENT", "DEPLOYMENT_SCRIPT"
        ]
        if content_type not in valid_content_types:
            raise ValueError(f"content_type must be one of {valid_content_types}")

        # Validate quality score is between 0 and 100
        try:
            score = int(quality_score)
            if score < 0 or score > 100:
                raise ValueError("quality_score must be between 0 and 100")
        except ValueError:
            raise ValueError("quality_score must be a valid integer between 0 and 100")

        # Convert string request_id to UUID object for database storage
        if isinstance(request_id, str):
            self.request_id = uuid.UUID(request_id)
        else:
            self.request_id = request_id

        self.content_type = content_type
        self.content = content
        self.metadata_obj = metadata or {}
        self.quality_score = quality_score