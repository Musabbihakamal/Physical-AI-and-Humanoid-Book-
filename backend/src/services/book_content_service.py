"""
Book Content Service

This service provides functionality for generating book content using the Book Content Writer Agent,
with proper validation, storage, and retrieval capabilities.
"""
from typing import Dict, Any, Optional, List
from datetime import datetime

from ..agents.book_content_writer.book_content_writer_agent import (
    BookContentWriterAgent,
    BookContentRequest,
    ContentComplexity
)
from ..models.generated_content import GeneratedContent
from ..models.book_chapter import BookChapter
from .content_service import ContentService
try:
    # Try importing from the installed package structure
    from shared.utils.content_validation import (
        validate_technical_accuracy,
        validate_docusaurus_formatting,
        validate_educational_structure,
        validate_content_safety
    )
except ImportError:
    # Fallback to relative import for development
    import sys
    import os
    from pathlib import Path
    project_root = Path(__file__).parent.parent.parent.parent
    sys.path.insert(0, str(project_root))
    from shared.utils.content_validation import (
        validate_technical_accuracy,
        validate_docusaurus_formatting,
        validate_educational_structure,
        validate_content_safety
    )


class BookContentService:
    """Service for managing book content generation and validation"""

    def __init__(self, content_service: ContentService):
        self.content_service = content_service
        self.agent = BookContentWriterAgent(content_service)

    async def generate_book_chapter(
        self,
        module_focus: str,
        chapter_title: str,
        content_type: str,
        target_audience: str,  # "beginner", "intermediate", "expert"
        technical_domain: str,
        user_profile: Optional[Dict[str, Any]] = None,
        include_glossary: bool = True,
        include_exercises: bool = True,
        include_code_examples: bool = True,
        include_diagrams: bool = True
    ) -> BookChapter:
        """
        Generate a complete book chapter following constitution requirements.

        Args:
            module_focus: The focus area of the module
            chapter_title: Title of the chapter to generate
            content_type: Type of content ("theory", "code", "exercises", etc.)
            target_audience: Audience level ("beginner", "intermediate", "expert")
            technical_domain: Technical domain ("ROS 2", "Isaac Sim", etc.)
            user_profile: Optional user profile for personalization
            include_glossary: Whether to include a glossary
            include_exercises: Whether to include exercises
            include_code_examples: Whether to include code examples
            include_diagrams: Whether to include diagrams

        Returns:
            BookChapter object with generated content
        """
        # Convert target audience string to enum
        try:
            complexity_enum = ContentComplexity(target_audience.lower())
        except ValueError:
            raise ValueError(f"Invalid target audience: {target_audience}. Must be 'beginner', 'intermediate', or 'expert'")

        # Create the agent request
        request = BookContentRequest(
            module_focus=module_focus,
            chapter_title=chapter_title,
            content_type=content_type,
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            user_profile=user_profile,
            include_glossary=include_glossary,
            include_exercises=include_exercises,
            include_code_examples=include_code_examples,
            include_diagrams=include_diagrams
        )

        # Generate content using the agent
        generated_result = await self.agent.generate_content(request)

        # Perform additional validation using the validation utilities
        validation_results = await self._perform_comprehensive_validation(
            generated_result["content"],
            technical_domain
        )

        # Check if content passes all validations
        if not validation_results["overall_valid"]:
            raise ValueError(f"Content failed validation: {validation_results['issues']}")

        # Create and return the book chapter
        chapter = BookChapter(
            title=chapter_title,
            content=generated_result["content"],
            module_focus=module_focus,
            difficulty_level=target_audience.upper(),  # Map target_audience to difficulty_level
            technical_domain=technical_domain,
            validation_results=validation_results,
            quality_score=int(generated_result["quality_score"] * 100)  # Convert from 0-1 to 0-100 scale
        )

        return chapter

    async def _perform_comprehensive_validation(
        self,
        content: str,
        technical_domain: str
    ) -> Dict[str, Any]:
        """Perform comprehensive validation of generated content"""
        # Validate technical accuracy
        tech_validation = validate_technical_accuracy(content, technical_domain)

        # Validate Docusaurus formatting
        format_validation = validate_docusaurus_formatting(content)

        # Validate educational structure
        edu_validation = validate_educational_structure(content)

        # Validate content safety
        safety_validation = validate_content_safety(content)

        # Combine all validation results
        all_issues = []
        all_issues.extend(tech_validation["issues"])
        all_issues.extend(format_validation["issues"])
        all_issues.extend(edu_validation["issues"])
        all_issues.extend(safety_validation["issues"])

        # Calculate overall validity
        overall_valid = (
            tech_validation["valid"] and
            format_validation["valid"] and
            edu_validation["valid"] and
            safety_validation["safe"]
        )

        return {
            "overall_valid": overall_valid,
            "issues": all_issues,
            "technical_accuracy": tech_validation,
            "format_compliance": format_validation,
            "educational_structure": edu_validation,
            "safety_compliance": safety_validation,
            "comprehensive_score": (
                tech_validation["quality_score"] * 0.3 +
                format_validation["format_compliance"] * 0.2 +
                edu_validation["structure_score"] * 0.3 +
                safety_validation["safety_score"] * 0.2
            )
        }

    async def validate_existing_content(
        self,
        content: str,
        technical_domain: str = "ROS 2"
    ) -> Dict[str, Any]:
        """
        Validate existing content against constitution requirements.

        Args:
            content: The content to validate
            technical_domain: The technical domain to validate against

        Returns:
            Dictionary with validation results
        """
        return await self._perform_comprehensive_validation(content, technical_domain)

    async def update_chapter_content(
        self,
        chapter_id: str,
        updated_content: str,
        technical_domain: str = "ROS 2"
    ) -> BookChapter:
        """
        Update existing chapter content with validation.

        Args:
            chapter_id: ID of the chapter to update
            updated_content: New content to validate and update
            technical_domain: Technical domain for validation

        Returns:
            Updated BookChapter object
        """
        # Validate the updated content
        validation_results = await self._perform_comprehensive_validation(
            updated_content,
            technical_domain
        )

        if not validation_results["overall_valid"]:
            raise ValueError(f"Updated content failed validation: {validation_results['issues']}")

        # In a real implementation, this would update the stored chapter
        # For now, we'll return a new object with the updated content
        chapter = BookChapter(
            title=f"Updated Chapter {chapter_id}",
            module_focus="Updated Module",
            content=updated_content,
            content_type="theory",
            target_audience="intermediate",
            technical_domain=technical_domain,
            validation_results=validation_results,
            created_at=datetime.utcnow(),
            quality_score=validation_results["comprehensive_score"],
            metadata={"updated": True}
        )

        return chapter

    async def get_recommended_exercises(
        self,
        chapter_content: str,
        target_audience: str = "intermediate"
    ) -> List[Dict[str, str]]:
        """
        Generate recommended exercises based on chapter content and target audience.

        Args:
            chapter_content: The chapter content to generate exercises for
            target_audience: The target audience level

        Returns:
            List of exercise dictionaries with title and description
        """
        exercises = []

        # This would typically use an LLM to generate exercises based on content
        # For now, we'll return a template structure
        base_exercises = [
            {
                "title": f"Basic {target_audience.title()} Exercise",
                "description": "A foundational exercise to reinforce basic concepts",
                "difficulty": target_audience,
                "type": "conceptual"
            },
            {
                "title": f"Applied {target_audience.title()} Exercise",
                "description": "An applied exercise that uses the concepts in practice",
                "difficulty": target_audience,
                "type": "practical"
            }
        ]

        return base_exercises

    async def extend_chapter_content(
        self,
        chapter_id: str,
        extension_request: str,
        target_audience: str,
        technical_domain: str
    ) -> str:
        """
        Extend existing chapter content with additional information.

        Args:
            chapter_id: ID of the chapter to extend
            extension_request: Description of what content should be added
            target_audience: Target audience level for the extension
            technical_domain: Technical domain for the extension

        Returns:
            Extended content string
        """
        # In a real implementation, this would fetch the existing chapter content
        # For this implementation, we'll create a method to extend content based on the request
        from ..agents.book_content_writer.book_content_writer_agent import ContentComplexity

        complexity_enum = ContentComplexity(target_audience.lower())

        # Create a request to generate content that extends the existing chapter
        request = BookContentRequest(
            module_focus=extension_request,
            chapter_title=f"Extended Content: {extension_request}",
            content_type="extension",
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            include_glossary=True,
            include_exercises=True,
            include_code_examples=True,
            include_diagrams=True
        )

        # Generate the extension content
        generated_result = await self.agent.generate_content(request)

        return generated_result["content"]

    async def expand_chapter_topic(
        self,
        existing_content: str,
        topic_to_expand: str,
        expansion_details: str,
        target_audience: str,
        technical_domain: str
    ) -> str:
        """
        Expand a specific topic within existing chapter content.

        Args:
            existing_content: The current chapter content
            topic_to_expand: The specific topic to expand
            expansion_details: Details about what aspects to expand
            target_audience: Target audience level
            technical_domain: Technical domain

        Returns:
            Content with expanded topic section
        """
        from ..agents.book_content_writer.book_content_writer_agent import ContentComplexity

        complexity_enum = ContentComplexity(target_audience.lower())

        # Create a focused request for expanding the specific topic
        request = BookContentRequest(
            module_focus=f"Expanding: {topic_to_expand}",
            chapter_title=f"Detailed Coverage of {topic_to_expand}",
            content_type="detailed_explanation",
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            include_glossary=False,  # Will be added to existing glossary
            include_exercises=False,  # Will be added to existing exercises
            include_code_examples=True,
            include_diagrams=True
        )

        # Generate detailed content for the specific topic
        generated_result = await self.agent.generate_content(request)

        # Combine with existing content
        expanded_content = f"""
{existing_content}

## Extended Coverage: {topic_to_expand}

{expansion_details}

{generated_result["content"]}
"""

        return expanded_content

    async def add_advanced_content_section(
        self,
        existing_content: str,
        section_title: str,
        advanced_topics: list,
        target_audience: str,
        technical_domain: str
    ) -> str:
        """
        Add an advanced content section to existing chapter.

        Args:
            existing_content: The current chapter content
            section_title: Title for the new advanced section
            advanced_topics: List of advanced topics to cover
            target_audience: Target audience level
            technical_domain: Technical domain

        Returns:
            Content with new advanced section
        """
        from ..agents.book_content_writer.book_content_writer_agent import ContentComplexity

        complexity_enum = ContentComplexity(target_audience.lower())

        # Create a request for advanced content
        request = BookContentRequest(
            module_focus=f"Advanced: {', '.join(advanced_topics)}",
            chapter_title=section_title,
            content_type="advanced_concepts",
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            include_glossary=False,
            include_exercises=True,
            include_code_examples=True,
            include_diagrams=True
        )

        # Generate advanced content
        generated_result = await self.agent.generate_content(request)

        # Add the advanced section to existing content
        advanced_content = f"""
## {section_title}

{generated_result["content"]}
"""

        return existing_content + advanced_content

    async def generate_module_content(
        self,
        module_focus: str,
        content_focus: str,
        target_audience: str,
        technical_domain: str,
        include_practical_examples: bool = True,
        include_assessment: bool = True
    ) -> str:
        """
        Generate comprehensive content for a specific module focus area.

        Args:
            module_focus: The main module topic
            content_focus: Specific focus within the module
            target_audience: Target audience level
            technical_domain: Technical domain to focus on
            include_practical_examples: Whether to include practical examples
            include_assessment: Whether to include assessment questions

        Returns:
            Generated module content string
        """
        from ..agents.book_content_writer.book_content_writer_agent import ContentComplexity

        complexity_enum = ContentComplexity(target_audience.lower())

        # Create a request for comprehensive module content
        request = BookContentRequest(
            module_focus=module_focus,
            chapter_title=f"{content_focus} in {module_focus}",
            content_type="comprehensive",
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            include_glossary=True,
            include_exercises=include_assessment,  # Include exercises as part of assessment
            include_code_examples=include_practical_examples,
            include_diagrams=True
        )

        # Generate the module content
        generated_result = await self.agent.generate_content(request)

        # Validate the generated content
        validation_results = await self._perform_comprehensive_validation(
            generated_result["content"],
            technical_domain
        )

        if not validation_results["overall_valid"]:
            raise ValueError(f"Module content failed validation: {validation_results['issues']}")

        return generated_result["content"]

    async def enhance_existing_content(
        self,
        existing_content: str,
        enhancement_request: str,
        target_audience: str,
        technical_domain: str
    ) -> str:
        """
        Enhance existing content with additional information or improvements.

        Args:
            existing_content: The existing content to enhance
            enhancement_request: Description of what enhancements to make
            target_audience: Target audience level for the enhancement
            technical_domain: Technical domain for the enhancement

        Returns:
            Enhanced content string
        """
        from ..agents.book_content_writer.book_content_writer_agent import ContentComplexity

        complexity_enum = ContentComplexity(target_audience.lower())

        # Create a request to generate enhancement content
        request = BookContentRequest(
            module_focus="Content Enhancement",
            chapter_title="Enhanced Content",
            content_type="enhancement",
            target_audience=complexity_enum,
            technical_domain=technical_domain,
            include_glossary=True,
            include_exercises=False,  # Exercises would be separate
            include_code_examples=True,
            include_diagrams=True
        )

        # In a real implementation, we would analyze the existing content
        # and generate specific enhancements based on the request
        # For now, we'll generate content that addresses the enhancement_request

        generated_result = await self.agent.generate_content(request)

        # In a real implementation, we would intelligently merge the new content
        # with the existing content to create an enhanced version
        # For now, we'll return a combination of both
        enhanced_content = f"""
{existing_content}

## Enhancement: {enhancement_request}

{generated_result["content"]}
"""

        return enhanced_content