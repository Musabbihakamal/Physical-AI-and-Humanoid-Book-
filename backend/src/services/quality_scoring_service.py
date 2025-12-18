"""
Quality Scoring Service for Generated Content

This service provides functionality for scoring and validating generated content
according to the Physical AI Book Constitution requirements.
"""
import re
from typing import Dict, Any, List, Tuple
from dataclasses import dataclass
from enum import Enum

from backend.src.models.generated_content import GeneratedContent
from shared.utils.content_validation import (
    validate_technical_accuracy,
    validate_docusaurus_formatting,
    validate_educational_structure,
    validate_content_safety
)


class QualityDimension(Enum):
    TECHNICAL_ACCURACY = "technical_accuracy"
    FORMATTING_COMPLIANCE = "formatting_compliance"
    EDUCATIONAL_VALUE = "educational_value"
    SAFETY_COMPLIANCE = "safety_compliance"
    CONTENT_STRUCTURE = "content_structure"


@dataclass
class QualityScore:
    """Represents a quality score with breakdown by dimension"""
    overall_score: float
    dimension_scores: Dict[QualityDimension, float]
    validation_issues: List[str]
    is_valid: bool


class QualityScoringService:
    """Service for scoring and validating generated content quality"""

    def __init__(self):
        self.weights = {
            QualityDimension.TECHNICAL_ACCURACY: 0.30,
            QualityDimension.FORMATTING_COMPLIANCE: 0.20,
            QualityDimension.EDUCATIONAL_VALUE: 0.25,
            QualityDimension.SAFETY_COMPLIANCE: 0.15,
            QualityDimension.CONTENT_STRUCTURE: 0.10
        }

    async def score_content(self, content: GeneratedContent) -> QualityScore:
        """
        Score the quality of generated content based on constitution requirements.

        Args:
            content: The generated content to score

        Returns:
            QualityScore object with overall score and breakdown
        """
        # Perform validation checks
        tech_validation = validate_technical_accuracy(
            content.content,
            content.metadata_obj.get("technical_domain", "ROS 2")
        )
        format_validation = validate_docusaurus_formatting(content.content)
        edu_validation = validate_educational_structure(content.content)
        safety_validation = validate_content_safety(content.content)

        # Calculate dimension scores
        dimension_scores = {
            QualityDimension.TECHNICAL_ACCURACY: tech_validation["quality_score"],
            QualityDimension.FORMATTING_COMPLIANCE: format_validation["format_compliance"],
            QualityDimension.EDUCATIONAL_VALUE: edu_validation["structure_score"],
            QualityDimension.SAFETY_COMPLIANCE: float(safety_validation["safety_score"]),
            QualityDimension.CONTENT_STRUCTURE: self._calculate_content_structure_score(content)
        }

        # Combine scores using weighted average
        overall_score = self._calculate_weighted_score(dimension_scores)

        # Collect all validation issues
        all_issues = []
        all_issues.extend(tech_validation["issues"])
        all_issues.extend(format_validation["issues"])
        all_issues.extend(edu_validation["issues"])
        if not safety_validation["safe"]:
            all_issues.extend(safety_validation["issues"])

        # Determine if content is valid (overall score > 70% and no safety issues)
        is_valid = overall_score >= 0.7 and safety_validation["safe"]

        return QualityScore(
            overall_score=overall_score,
            dimension_scores=dimension_scores,
            validation_issues=all_issues,
            is_valid=is_valid
        )

    async def validate_content_for_constitution_compliance(self, content: str, content_type: str) -> Dict[str, Any]:
        """
        Validate content specifically for constitution compliance.

        Args:
            content: The content to validate
            content_type: The type of content being validated

        Returns:
            Dictionary with validation results
        """
        # Check for required educational structure based on content type
        edu_validation = validate_educational_structure(content)

        # Check formatting compliance
        format_validation = validate_docusaurus_formatting(content)

        # Check technical accuracy
        tech_validation = validate_technical_accuracy(content)

        # Check safety compliance
        safety_validation = validate_content_safety(content)

        # Additional constitution-specific checks
        constitution_issues = []

        # Check for required sections based on content type
        if content_type in ["CHAPTER", "DOCUMENTATION"]:
            required_sections = ["## Learning Objectives", "## Detailed Theory", "## Summary"]
            missing_sections = [section for section in required_sections if section not in content]
            if missing_sections:
                constitution_issues.extend([f"Missing required section: {section}" for section in missing_sections])

        # Check for proper code examples in technical content
        if content_type in ["CHAPTER", "CODE_EXPLANATION"] and "```" not in content:
            constitution_issues.append("Technical content should include code examples in fenced blocks")

        # Check for glossary in chapter content
        if content_type == "CHAPTER" and "## Glossary" not in content:
            constitution_issues.append("Chapter content should include a glossary section")

        return {
            "technical_accuracy": tech_validation,
            "formatting_compliance": format_validation,
            "educational_structure": edu_validation,
            "safety_compliance": safety_validation,
            "constitution_issues": constitution_issues,
            "overall_compliant": (
                tech_validation["valid"] and
                format_validation["valid"] and
                edu_validation["valid"] and
                safety_validation["safe"] and
                len(constitution_issues) == 0
            )
        }

    def _calculate_content_structure_score(self, content: GeneratedContent) -> float:
        """
        Calculate a score based on content structure and completeness.

        Args:
            content: The generated content

        Returns:
            Score between 0 and 1
        """
        score = 0.0
        total_checks = 6  # Number of structure checks

        # Check for main heading
        if content.content.startswith("# "):
            score += 1

        # Check for learning objectives (if applicable)
        if content.content_type in ["CHAPTER", "DOCUMENTATION"]:
            if "## Learning Objectives" in content.content:
                score += 1

        # Check for detailed content
        word_count = len(content.content.split())
        if word_count > 100:  # Reasonable minimum for educational content
            score += 1

        # Check for code blocks in technical content
        if content.content_type in ["CODE_EXPLANATION", "CHAPTER"]:
            if "```" in content.content:
                score += 1

        # Check for exercises in chapter content
        if content.content_type == "CHAPTER":
            if "Exercises" in content.content or "## Exercises" in content.content:
                score += 1

        # Check for glossary in chapter content
        if content.content_type == "CHAPTER":
            if "## Glossary" in content.content:
                score += 1

        return score / total_checks

    def _calculate_weighted_score(self, dimension_scores: Dict[QualityDimension, float]) -> float:
        """
        Calculate the overall weighted score from dimension scores.

        Args:
            dimension_scores: Dictionary mapping dimensions to their scores

        Returns:
            Overall weighted score between 0 and 1
        """
        weighted_sum = 0.0
        total_weight = sum(self.weights.values())

        for dimension, score in dimension_scores.items():
            weight = self.weights.get(dimension, 0.0)
            weighted_sum += score * weight

        return weighted_sum / total_weight if total_weight > 0 else 0.0

    async def get_improvement_suggestions(self, content: GeneratedContent) -> List[str]:
        """
        Provide improvement suggestions for content quality.

        Args:
            content: The generated content to analyze

        Returns:
            List of improvement suggestions
        """
        suggestions = []

        # Check for content type specific improvements
        if content.content_type == "CHAPTER":
            if "## Learning Objectives" not in content.content:
                suggestions.append("Add a 'Learning Objectives' section to help students understand what they'll learn")

            if "## Summary" not in content.content:
                suggestions.append("Add a 'Summary' section to reinforce key concepts")

            if "## Exercises" not in content.content and "Exercises" not in content.content:
                suggestions.append("Add exercises to help students practice what they've learned")

        elif content.content_type == "CODE_EXPLANATION":
            # Check for ROS 2 or Isaac Sim specific commands
            has_ros2_specific = bool(re.search(r'rclpy|Node|create_publisher|create_subscription|ros2', content.content, re.IGNORECASE))
            has_isaac_specific = bool(re.search(r'isaac|omni|nvblox|jetson', content.content, re.IGNORECASE))

            if not has_ros2_specific and not has_isaac_specific:
                suggestions.append("Consider adding specific examples for ROS 2 or Isaac Sim commands to improve relevance")

        elif content.content_type == "GLOSSARY":
            if len(content.content) < 100:
                suggestions.append("Glossary appears to be too short - consider adding more terms and definitions")

        elif content.content_type == "QUIZ":
            if "MCQ" not in content.content.upper() and "TRUE/FALSE" not in content.content.upper():
                suggestions.append("Consider adding multiple-choice questions for better assessment variety")

        # General suggestions
        word_count = len(content.content.split())
        if word_count < 200 and content.content_type in ["CHAPTER", "DOCUMENTATION"]:
            suggestions.append(f"Content appears short ({word_count} words) - consider expanding for better educational value")

        if "TODO" in content.content or "TKTK" in content.content:
            suggestions.append("Remove placeholder text like TODO or TKTK before publishing")

        return suggestions

    async def apply_quality_threshold(self, content: GeneratedContent, min_score: float = 0.7) -> Tuple[bool, str]:
        """
        Check if content meets minimum quality threshold.

        Args:
            content: The generated content to check
            min_score: Minimum quality score threshold (0-1)

        Returns:
            Tuple of (is_acceptable, reason)
        """
        quality_score = await self.score_content(content)

        if not quality_score.is_valid:
            return False, f"Content failed validation with overall score {quality_score.overall_score:.2f} and issues: {quality_score.validation_issues}"

        if quality_score.overall_score < min_score:
            return False, f"Content score {quality_score.overall_score:.2f} below minimum threshold {min_score}"

        return True, f"Content meets quality standards with score {quality_score.overall_score:.2f}"


# Global instance for use in other services
quality_scoring_service = QualityScoringService()