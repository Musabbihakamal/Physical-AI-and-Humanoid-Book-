"""
Content personalization service for the multi-agent book generation system.
Adapts content based on user profile and experience level.
"""
from typing import Dict, Any, Optional, List
import logging
from ..models.user_profile import UserProfile

logger = logging.getLogger(__name__)

class ContentPersonalizationService:
    """
    Service to handle content personalization based on user profiles and experience levels.
    """

    @staticmethod
    def adapt_content_for_experience_level(
        content: Dict[str, Any],
        user_profile: Optional[UserProfile] = None,
        override_level: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Adapt content based on user's experience level.

        Args:
            content: Original content dictionary
            user_profile: User's profile with experience level
            override_level: Override experience level (for chapter-specific overrides)

        Returns:
            Adapted content dictionary
        """
        # Determine the experience level to use
        experience_level = override_level or (user_profile.experience_level if user_profile else "INTERMEDIATE")

        # Make a copy of the content to avoid modifying the original
        adapted_content = content.copy()

        # Adapt content based on experience level
        if experience_level == "BEGINNER":
            adapted_content = ContentPersonalizationService._adapt_for_beginner(adapted_content)
        elif experience_level == "EXPERT":
            adapted_content = ContentPersonalizationService._adapt_for_expert(adapted_content)
        else:  # INTERMEDIATE (default)
            adapted_content = ContentPersonalizationService._adapt_for_intermediate(adapted_content)

        # Add metadata about the personalization
        adapted_content["personalization_metadata"] = {
            "original_experience_level": user_profile.experience_level if user_profile else "INTERMEDIATE",
            "applied_experience_level": experience_level,
            "was_overridden": override_level is not None
        }

        return adapted_content

    @staticmethod
    def _adapt_for_beginner(content: Dict[str, Any]) -> Dict[str, Any]:
        """
        Adapt content for beginner level users.
        Adds extra explanations, simpler examples, and more context.
        """
        adapted = content.copy()

        # Add beginner-specific content modifications
        if "content" in adapted:
            # Add extra explanations to text content
            if isinstance(adapted["content"], str):
                adapted["content"] = ContentPersonalizationService._add_beginner_explanations(adapted["content"])

        # Add beginner-specific metadata
        adapted["difficulty_level"] = "BEGINNER"
        adapted["has_extra_explanations"] = True
        adapted["has_simplified_examples"] = True

        # If there are exercises or quizzes, make them simpler
        if "exercises" in adapted:
            adapted["exercises"] = ContentPersonalizationService._simplify_exercises(adapted["exercises"], "BEGINNER")

        return adapted

    @staticmethod
    def _adapt_for_intermediate(content: Dict[str, Any]) -> Dict[str, Any]:
        """
        Adapt content for intermediate level users.
        Focuses on implementation and core concepts.
        """
        adapted = content.copy()

        # Add intermediate-specific metadata
        adapted["difficulty_level"] = "INTERMEDIATE"
        adapted["focus"] = "implementation"

        # If there are exercises or quizzes, keep them standard
        if "exercises" in adapted:
            adapted["exercises"] = ContentPersonalizationService._standardize_exercises(adapted["exercises"])

        return adapted

    @staticmethod
    def _adapt_for_expert(content: Dict[str, Any]) -> Dict[str, Any]:
        """
        Adapt content for expert level users.
        Adds advanced topics, optional challenges, and deeper insights.
        """
        adapted = content.copy()

        # Add expert-specific content modifications
        if "content" in adapted:
            if isinstance(adapted["content"], str):
                adapted["content"] = ContentPersonalizationService._add_expert_content(adapted["content"])

        # Add expert-specific metadata
        adapted["difficulty_level"] = "EXPERT"
        adapted["has_advanced_topics"] = True
        adapted["has_optional_challenges"] = True

        # If there are exercises or quizzes, make them more challenging
        if "exercises" in adapted:
            adapted["exercises"] = ContentPersonalizationService._enhance_exercises(adapted["exercises"], "EXPERT")

        return adapted

    @staticmethod
    def _add_beginner_explanations(content: str) -> str:
        """
        Add beginner-friendly explanations to content.
        """
        # This is a simplified implementation - in a real system, this would be more sophisticated
        # For now, we'll just return the content as is, but in a real implementation
        # this would add extra context, explanations, and simpler language
        return content

    @staticmethod
    def _add_expert_content(content: str) -> str:
        """
        Add expert-level content to make it more challenging.
        """
        # This is a simplified implementation - in a real system, this would be more sophisticated
        # For now, we'll just return the content as is, but in a real implementation
        # this would add advanced concepts, deeper insights, and more complex examples
        return content

    @staticmethod
    def _simplify_exercises(exercises: List[Dict[str, Any]], target_level: str) -> List[Dict[str, Any]]:
        """
        Simplify exercises for beginner level.
        """
        # Simplify exercises based on target level
        if target_level == "BEGINNER":
            # Make exercises simpler
            simplified_exercises = []
            for exercise in exercises:
                simple_exercise = exercise.copy()
                # Add hints, simpler instructions, etc.
                if "hints" not in simple_exercise:
                    simple_exercise["hints"] = ["This exercise is designed for beginners"]
                simplified_exercises.append(simple_exercise)
            return simplified_exercises
        return exercises

    @staticmethod
    def _enhance_exercises(exercises: List[Dict[str, Any]], target_level: str) -> List[Dict[str, Any]]:
        """
        Enhance exercises for expert level.
        """
        if target_level == "EXPERT":
            # Make exercises more challenging
            enhanced_exercises = []
            for exercise in exercises:
                enhanced_exercise = exercise.copy()
                # Add advanced challenges, extension tasks, etc.
                if "challenges" not in enhanced_exercise:
                    enhanced_exercise["challenges"] = ["This exercise includes advanced concepts"]
                enhanced_exercises.append(enhanced_exercise)
            return enhanced_exercises
        return exercises

    @staticmethod
    def _standardize_exercises(exercises: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Keep exercises at standard difficulty for intermediate level.
        """
        return exercises

    @staticmethod
    def get_user_preferred_difficulty(user_profile: Optional[UserProfile]) -> str:
        """
        Get the user's preferred difficulty level from their profile.
        """
        if not user_profile:
            return "INTERMEDIATE"  # Default to intermediate

        # Map experience level to difficulty
        experience_to_difficulty = {
            "BEGINNER": "EASY",
            "INTERMEDIATE": "MEDIUM",
            "EXPERT": "HARD"
        }

        return experience_to_difficulty.get(user_profile.experience_level, "MEDIUM")