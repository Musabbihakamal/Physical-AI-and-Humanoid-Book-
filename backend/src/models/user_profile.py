"""
User profile model for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Text, JSON
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from typing import List, Optional
import uuid


class UserProfile(Base):
    __tablename__ = "user_profiles"

    # Fields
    user_id = Column(UUID_TYPE, primary_key=True, default=lambda: str(uuid.uuid4()))
    experience_level = Column(String, nullable=False)  # BEGINNER, INTERMEDIATE, EXPERT
    technical_background = Column(Text, nullable=True)
    preferred_difficulty = Column(String)  # EASY, MEDIUM, HARD
    learning_goals = Column(JSON, default=[])
    hardware_access = Column(JSON, default=[])
    language_preference = Column(String, default="en")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __init__(self,
                 experience_level: str,
                 technical_background: Optional[str] = None,
                 preferred_difficulty: Optional[str] = None,
                 learning_goals: Optional[List[str]] = None,
                 hardware_access: Optional[List[str]] = None,
                 language_preference: str = "en"):
        # Validation
        valid_experience_levels = ["BEGINNER", "INTERMEDIATE", "EXPERT"]
        if experience_level not in valid_experience_levels:
            raise ValueError(f"experience_level must be one of {valid_experience_levels}")

        valid_difficulty_levels = ["EASY", "MEDIUM", "HARD"]
        if preferred_difficulty and preferred_difficulty not in valid_difficulty_levels:
            raise ValueError(f"preferred_difficulty must be one of {valid_difficulty_levels}")

        self.user_id = str(uuid.uuid4())
        self.experience_level = experience_level
        self.technical_background = technical_background
        self.preferred_difficulty = preferred_difficulty
        self.learning_goals = learning_goals or []
        self.hardware_access = hardware_access or []
        self.language_preference = language_preference