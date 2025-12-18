"""
RAG session models for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Text, JSON, ForeignKey
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from typing import Dict, Any, List, Optional
import uuid


class RAGSession(Base):
    __tablename__ = "rag_sessions"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID_TYPE, ForeignKey("user_profiles.user_id"), nullable=True)
    session_token = Column(String, nullable=True)  # for anonymous users
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    query_history = Column(JSON, default=[])  # array of query/response objects
    active_context = Column(Text, nullable=True)

    def __init__(self,
                 user_id: Optional[str] = None,
                 session_token: Optional[str] = None,
                 active_context: Optional[str] = None):
        # Either user_id or session_token must be provided
        if not user_id and not session_token:
            raise ValueError("Either user_id or session_token must be provided")

        self.user_id = user_id
        self.session_token = session_token
        self.active_context = active_context
        self.query_history = []


class RAGQuery(Base):
    __tablename__ = "rag_queries"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID_TYPE, ForeignKey("rag_sessions.id"), nullable=False)
    query = Column(Text, nullable=False)
    response = Column(Text, nullable=False)
    sources = Column(JSON, default=[])  # array of document IDs
    confidence_score = Column(String, default="50")  # 0-100 confidence in response accuracy
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __init__(self,
                 session_id: str,
                 query: str,
                 response: str,
                 sources: Optional[List[str]] = None,
                 confidence_score: str = "50"):
        # Validate confidence score is between 0 and 100
        try:
            score = int(confidence_score)
            if score < 0 or score > 100:
                raise ValueError("confidence_score must be between 0 and 100")
        except ValueError:
            raise ValueError("confidence_score must be a valid integer between 0 and 100")

        if not query or not response:
            raise ValueError("query and response must not be empty")

        self.session_id = session_id
        self.query = query
        self.response = response
        self.sources = sources or []
        self.confidence_score = confidence_score