"""
Token model for the multi-agent book generation system.
Used for storing refresh tokens and managing token blacklisting.
"""
from sqlalchemy import Column, String, DateTime, Text, ForeignKey, Boolean
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
import uuid
from datetime import datetime


class Token(Base):
    __tablename__ = "tokens"

    id = Column(UUID_TYPE, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(UUID_TYPE, ForeignKey("users.id"), nullable=False, index=True)
    token = Column(String, unique=True, nullable=False, index=True)
    token_type = Column(String, nullable=False)  # "refresh", "access", "verification", etc.
    expires_at = Column(DateTime(timezone=True), nullable=False)
    is_blacklisted = Column(Boolean, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __init__(self, user_id: str, token: str, token_type: str, expires_at: datetime):
        self.user_id = user_id
        self.token = token
        self.token_type = token_type
        self.expires_at = expires_at

    def is_expired(self) -> bool:
        """Check if the token is expired."""
        return datetime.now() > self.expires_at

    def is_valid(self) -> bool:
        """Check if the token is valid (not expired and not blacklisted)."""
        return not self.is_expired() and not self.is_blacklisted