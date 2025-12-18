"""
User service for the multi-agent book generation system.
"""
from sqlalchemy.orm import Session
from ..models.user_profile import UserProfile
from typing import Optional, List
import logging

logger = logging.getLogger(__name__)


class UserService:
    @staticmethod
    def create_user_profile(
        db: Session,
        experience_level: str,
        technical_background: Optional[str] = None,
        preferred_difficulty: Optional[str] = None,
        learning_goals: Optional[List[str]] = None,
        hardware_access: Optional[List[str]] = None,
        language_preference: str = "en"
    ) -> UserProfile:
        """
        Create a new user profile.

        Args:
            db: Database session
            experience_level: User's experience level (BEGINNER, INTERMEDIATE, EXPERT)
            technical_background: User's technical background
            preferred_difficulty: Preferred difficulty level (EASY, MEDIUM, HARD)
            learning_goals: List of user's learning goals
            hardware_access: List of hardware user has access to
            language_preference: Preferred language

        Returns:
            Created UserProfile object
        """
        try:
            user_profile = UserProfile(
                experience_level=experience_level,
                technical_background=technical_background,
                preferred_difficulty=preferred_difficulty,
                learning_goals=learning_goals,
                hardware_access=hardware_access,
                language_preference=language_preference
            )
            db.add(user_profile)
            db.commit()
            db.refresh(user_profile)
            logger.info(f"Created user profile with ID: {user_profile.user_id}")
            return user_profile
        except Exception as e:
            db.rollback()
            logger.error(f"Error creating user profile: {str(e)}")
            raise

    @staticmethod
    def get_user_profile(db: Session, user_id: str) -> Optional[UserProfile]:
        """
        Get a user profile by ID.

        Args:
            db: Database session
            user_id: User ID

        Returns:
            UserProfile object or None if not found
        """
        try:
            user_profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            return user_profile
        except Exception as e:
            logger.error(f"Error getting user profile: {str(e)}")
            raise

    @staticmethod
    def update_user_profile(
        db: Session,
        user_id: str,
        experience_level: Optional[str] = None,
        technical_background: Optional[str] = None,
        preferred_difficulty: Optional[str] = None,
        learning_goals: Optional[List[str]] = None,
        hardware_access: Optional[List[str]] = None,
        language_preference: Optional[str] = None
    ) -> Optional[UserProfile]:
        """
        Update a user profile.

        Args:
            db: Database session
            user_id: User ID
            experience_level: User's experience level (BEGINNER, INTERMEDIATE, EXPERT)
            technical_background: User's technical background
            preferred_difficulty: Preferred difficulty level (EASY, MEDIUM, HARD)
            learning_goals: List of user's learning goals
            hardware_access: List of hardware user has access to
            language_preference: Preferred language

        Returns:
            Updated UserProfile object or None if not found
        """
        try:
            user_profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not user_profile:
                return None

            if experience_level is not None:
                user_profile.experience_level = experience_level
            if technical_background is not None:
                user_profile.technical_background = technical_background
            if preferred_difficulty is not None:
                user_profile.preferred_difficulty = preferred_difficulty
            if learning_goals is not None:
                user_profile.learning_goals = learning_goals
            if hardware_access is not None:
                user_profile.hardware_access = hardware_access
            if language_preference is not None:
                user_profile.language_preference = language_preference

            db.commit()
            db.refresh(user_profile)
            logger.info(f"Updated user profile with ID: {user_profile.user_id}")
            return user_profile
        except Exception as e:
            db.rollback()
            logger.error(f"Error updating user profile: {str(e)}")
            raise

    @staticmethod
    def delete_user_profile(db: Session, user_id: str) -> bool:
        """
        Delete a user profile.

        Args:
            db: Database session
            user_id: User ID

        Returns:
            True if successful, False otherwise
        """
        try:
            user_profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not user_profile:
                return False

            db.delete(user_profile)
            db.commit()
            logger.info(f"Deleted user profile with ID: {user_profile.user_id}")
            return True
        except Exception as e:
            db.rollback()
            logger.error(f"Error deleting user profile: {str(e)}")
            raise