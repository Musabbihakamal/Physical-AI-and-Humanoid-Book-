"""
Agent request model for the multi-agent book generation system.
This model tracks requests made to various AI agents in the system.
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
    agent_type = Column(String, nullable=False)  # e.g., "book_generator", "content_editor", "validator"
    request_data = Column(JSON, nullable=False)  # the input data for the agent
    response_data = Column(JSON, nullable=True)  # the output data from the agent
    status = Column(String, default="PENDING")  # PENDING, PROCESSING, COMPLETED, FAILED
    error_message = Column(Text, nullable=True)  # if status is FAILED, store the error
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    completed_at = Column(DateTime(timezone=True), nullable=True)
    duration_ms = Column(String, nullable=True)  # how long the request took
    user_id = Column(UUID_TYPE, ForeignKey("users.id"), nullable=True)  # which user made the request

    def __init__(self,
                 agent_type: str,
                 request_data: Dict[str, Any],
                 response_data: Optional[Dict[str, Any]] = None,
                 status: str = "PENDING",
                 error_message: Optional[str] = None,
                 user_id: Optional[str] = None):
        # Validation
        valid_agent_types = [
            "book_generator", "content_editor", "validator", "translator",
            "summarizer", "formatter", "researcher", "editor", "reviewer"
        ]
        if agent_type not in valid_agent_types:
            raise ValueError(f"agent_type must be one of {valid_agent_types}")

        valid_statuses = ["PENDING", "PROCESSING", "COMPLETED", "FAILED"]
        if status not in valid_statuses:
            raise ValueError(f"status must be one of {valid_statuses}")

        self.agent_type = agent_type
        self.request_data = request_data
        self.response_data = response_data or {}
        self.status = status
        self.error_message = error_message
        self.user_id = user_id