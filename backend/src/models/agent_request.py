"""
Agent request model for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Text, JSON, ForeignKey
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from typing import Dict, Any, Optional
import uuid


class AgentRequest(Base):
    __tablename__ = "agent_requests"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    agent_type = Column(String, nullable=False)  # RESEARCH_AGENT, WRITER_AGENT, etc.
    parameters = Column(JSON, nullable=False)  # agent-specific parameters
    context = Column(JSON, nullable=True)  # chapter content, user profile, book outline, etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    status = Column(String, default="PENDING")  # PENDING, PROCESSING, COMPLETED, FAILED
    user_id = Column(UUID_TYPE, ForeignKey("user_profiles.user_id"), nullable=True)

    def __init__(self,
                 agent_type: str,
                 parameters: Dict[str, Any],
                 context: Optional[Dict[str, Any]] = None,
                 user_id: Optional[str] = None):
        # Validation
        valid_agent_types = [
            "RESEARCH_AGENT", "WRITER_AGENT", "EDITOR_AGENT", "RAG_ENGINEER_AGENT",
            "DEVELOPER_AGENT", "DOCUMENTATION_AGENT", "PROJECT_PLANNER_AGENT",
            "GLOSSARY_MAKER", "CHAPTER_GENERATOR", "CODE_EXPLAINER", "QUIZ_CREATOR"
        ]
        if agent_type not in valid_agent_types:
            raise ValueError(f"agent_type must be one of {valid_agent_types}")

        self.agent_type = agent_type
        self.parameters = parameters
        self.context = context or {}
        self.user_id = user_id
        # Only validate status if it's already set (after creation)
        # The default value "PENDING" is set by the Column definition