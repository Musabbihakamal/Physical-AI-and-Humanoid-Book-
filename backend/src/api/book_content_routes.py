"""
Book Content API Routes

This module defines the API endpoints for the Book Content Writer Agent,
allowing external systems to generate book content following constitution requirements.
"""
from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import Dict, Any, Optional, List

from ..services.book_content_service import BookContentService
from ..services.content_service import ContentService
from ..models.book_chapter import BookChapter
from ..database.database import get_db
try:
    # Try importing from the installed package structure
    from shared.utils.content_validation import validate_content_safety
except ImportError:
    # Fallback to relative import for development
    import sys
    import os
    from pathlib import Path
    project_root = Path(__file__).parent.parent.parent.parent
    sys.path.insert(0, str(project_root))
    from shared.utils.content_validation import validate_content_safety


router = APIRouter(prefix="/book-content", tags=["book-content"])


def get_book_content_service() -> BookContentService:
    """Dependency to get the book content service"""
    content_service = ContentService(get_db())
    return BookContentService(content_service)


@router.post("/generate-chapter", response_model=Dict[str, Any])
async def generate_book_chapter(
    module_focus: str,
    chapter_title: str,
    content_type: str,
    target_audience: str,  # "beginner", "intermediate", "expert"
    technical_domain: str,
    user_profile: Optional[Dict[str, Any]] = None,
    include_glossary: bool = True,
    include_exercises: bool = True,
    include_code_examples: bool = True,
    include_diagrams: bool = True,
    service: BookContentService = Depends(get_book_content_service)
):
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
        Dictionary containing the generated chapter and metadata
    """
    try:
        # Validate inputs
        valid_audiences = ["beginner", "intermediate", "expert"]
        if target_audience.lower() not in valid_audiences:
            raise HTTPException(
                status_code=400,
                detail=f"target_audience must be one of {valid_audiences}"
            )

        valid_content_types = ["theory", "code", "exercises", "diagrams", "overview", "practical", "assessment"]
        if content_type.lower() not in valid_content_types:
            raise HTTPException(
                status_code=400,
                detail=f"content_type must be one of {valid_content_types}"
            )

        valid_domains = ["ROS 2", "Isaac Sim", "Gazebo", "Unity", "NVIDIA Isaac", "RL", "VLA", "Physical AI"]
        if technical_domain not in valid_domains:
            raise HTTPException(
                status_code=400,
                detail=f"technical_domain must be one of {valid_domains}"
            )

        # Generate the chapter
        chapter = await service.generate_book_chapter(
            module_focus=module_focus,
            chapter_title=chapter_title,
            content_type=content_type,
            target_audience=target_audience,
            technical_domain=technical_domain,
            user_profile=user_profile,
            include_glossary=include_glossary,
            include_exercises=include_exercises,
            include_code_examples=include_code_examples,
            include_diagrams=include_diagrams
        )

        # Additional safety validation
        safety_check = validate_content_safety(chapter.content)
        if not safety_check["safe"]:
            raise HTTPException(
                status_code=422,
                detail=f"Content failed safety validation: {safety_check['issues']}"
            )

        return {
            "chapter_id": str(chapter.id) if hasattr(chapter, 'id') else "generated",
            "title": chapter.title,
            "content": chapter.get_docusaurus_compatible_content(),
            "module_focus": chapter.module_focus,
            "content_type": chapter.content_type if hasattr(chapter, 'content_type') else content_type,
            "target_audience": chapter.target_audience if hasattr(chapter, 'target_audience') else target_audience,
            "technical_domain": chapter.technical_domain if hasattr(chapter, 'technical_domain') else technical_domain,
            "quality_score": chapter.quality_score,
            "validation_results": chapter.validation_results,
            "learning_objectives": chapter.get_learning_objectives(),
            "glossary_terms": chapter.get_glossary_terms(),
            "generated_at": chapter.created_at if hasattr(chapter, 'created_at') else "now"
        }

    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating chapter: {str(e)}")


@router.post("/validate-content", response_model=Dict[str, Any])
async def validate_book_content(
    content: str,
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Validate existing book content against constitution requirements.

    Args:
        content: The content to validate
        technical_domain: The technical domain to validate against

    Returns:
        Dictionary containing validation results
    """
    try:
        validation_results = await service.validate_existing_content(
            content=content,
            technical_domain=technical_domain
        )

        return {
            "valid": validation_results["overall_valid"],
            "validation_results": validation_results,
            "issues": validation_results["issues"],
            "comprehensive_score": validation_results["comprehensive_score"]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating content: {str(e)}")


@router.post("/update-chapter/{chapter_id}", response_model=Dict[str, Any])
async def update_chapter_content(
    chapter_id: str,
    content: str,
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Update existing chapter content with validation.

    Args:
        chapter_id: ID of the chapter to update
        content: New content to validate and update
        technical_domain: Technical domain for validation

    Returns:
        Dictionary containing updated chapter information
    """
    try:
        updated_chapter = await service.update_chapter_content(
            chapter_id=chapter_id,
            updated_content=content,
            technical_domain=technical_domain
        )

        return {
            "chapter_id": str(updated_chapter.id) if hasattr(updated_chapter, 'id') else chapter_id,
            "title": updated_chapter.title,
            "content": updated_chapter.get_docusaurus_compatible_content(),
            "quality_score": updated_chapter.quality_score,
            "validation_results": updated_chapter.validation_results
        }
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating chapter: {str(e)}")


@router.post("/generate-exercises", response_model=Dict[str, Any])
async def generate_exercises(
    chapter_content: str,
    target_audience: str = "intermediate",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Generate recommended exercises based on chapter content.

    Args:
        chapter_content: The chapter content to generate exercises for
        target_audience: The target audience level

    Returns:
        Dictionary containing generated exercises
    """
    try:
        exercises = await service.get_recommended_exercises(
            chapter_content=chapter_content,
            target_audience=target_audience
        )

        return {
            "exercises": exercises,
            "target_audience": target_audience,
            "count": len(exercises)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating exercises: {str(e)}")


@router.post("/extend-chapter", response_model=Dict[str, Any])
async def extend_chapter_content(
    chapter_id: str,
    extension_request: str,
    target_audience: str = "intermediate",
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Extend existing chapter content with additional information.

    Args:
        chapter_id: ID of the chapter to extend
        extension_request: Description of what content should be added
        target_audience: Target audience level for the extension
        technical_domain: Technical domain for the extension

    Returns:
        Dictionary containing extended chapter information
    """
    try:
        valid_audiences = ["beginner", "intermediate", "expert"]
        if target_audience.lower() not in valid_audiences:
            raise HTTPException(
                status_code=400,
                detail=f"target_audience must be one of {valid_audiences}"
            )

        extended_content = await service.extend_chapter_content(
            chapter_id=chapter_id,
            extension_request=extension_request,
            target_audience=target_audience,
            technical_domain=technical_domain
        )

        return {
            "chapter_id": chapter_id,
            "extended_content": extended_content,
            "target_audience": target_audience,
            "technical_domain": technical_domain
        }
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error extending chapter: {str(e)}")


@router.post("/expand-topic", response_model=Dict[str, Any])
async def expand_chapter_topic(
    existing_content: str,
    topic_to_expand: str,
    expansion_details: str,
    target_audience: str = "intermediate",
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Expand a specific topic within existing chapter content.

    Args:
        existing_content: The current chapter content to expand
        topic_to_expand: The specific topic to expand
        expansion_details: Details about what aspects to expand
        target_audience: Target audience level
        technical_domain: Technical domain

    Returns:
        Dictionary containing content with expanded topic
    """
    try:
        valid_audiences = ["beginner", "intermediate", "expert"]
        if target_audience.lower() not in valid_audiences:
            raise HTTPException(
                status_code=400,
                detail=f"target_audience must be one of {valid_audiences}"
            )

        expanded_content = await service.expand_chapter_topic(
            existing_content=existing_content,
            topic_to_expand=topic_to_expand,
            expansion_details=expansion_details,
            target_audience=target_audience,
            technical_domain=technical_domain
        )

        return {
            "expanded_content": expanded_content,
            "topic_expanded": topic_to_expand,
            "target_audience": target_audience,
            "technical_domain": technical_domain
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error expanding topic: {str(e)}")


@router.post("/add-advanced-section", response_model=Dict[str, Any])
async def add_advanced_content_section(
    existing_content: str,
    section_title: str,
    advanced_topics: List[str],
    target_audience: str = "intermediate",
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Add an advanced content section to existing chapter.

    Args:
        existing_content: The current chapter content
        section_title: Title for the new advanced section
        advanced_topics: List of advanced topics to cover
        target_audience: Target audience level
        technical_domain: Technical domain

    Returns:
        Dictionary containing content with new advanced section
    """
    try:
        valid_audiences = ["beginner", "intermediate", "expert"]
        if target_audience.lower() not in valid_audiences:
            raise HTTPException(
                status_code=400,
                detail=f"target_audience must be one of {valid_audiences}"
            )

        advanced_content = await service.add_advanced_content_section(
            existing_content=existing_content,
            section_title=section_title,
            advanced_topics=advanced_topics,
            target_audience=target_audience,
            technical_domain=technical_domain
        )

        return {
            "enhanced_content": advanced_content,
            "section_added": section_title,
            "advanced_topics": advanced_topics,
            "target_audience": target_audience,
            "technical_domain": technical_domain
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error adding advanced section: {str(e)}")


@router.post("/generate-module-content", response_model=Dict[str, Any])
async def generate_module_content(
    module_focus: str,
    content_focus: str,
    target_audience: str = "intermediate",
    technical_domain: str = "ROS 2",
    include_practical_examples: bool = True,
    include_assessment: bool = True,
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Generate comprehensive content for a specific module focus area.

    Args:
        module_focus: The main module topic (e.g., "ROS 2 Basics", "Isaac Sim")
        content_focus: Specific focus within the module (e.g., "Nodes and Topics")
        target_audience: Target audience level
        technical_domain: Technical domain to focus on
        include_practical_examples: Whether to include practical examples
        include_assessment: Whether to include assessment questions

    Returns:
        Dictionary containing generated module content
    """
    try:
        valid_audiences = ["beginner", "intermediate", "expert"]
        if target_audience.lower() not in valid_audiences:
            raise HTTPException(
                status_code=400,
                detail=f"target_audience must be one of {valid_audiences}"
            )

        module_content = await service.generate_module_content(
            module_focus=module_focus,
            content_focus=content_focus,
            target_audience=target_audience,
            technical_domain=technical_domain,
            include_practical_examples=include_practical_examples,
            include_assessment=include_assessment
        )

        return {
            "module_focus": module_focus,
            "content_focus": content_focus,
            "target_audience": target_audience,
            "technical_domain": technical_domain,
            "content": module_content,
            "include_practical_examples": include_practical_examples,
            "include_assessment": include_assessment
        }
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating module content: {str(e)}")


@router.post("/enhance-existing-content", response_model=Dict[str, Any])
async def enhance_existing_content(
    existing_content: str,
    enhancement_request: str,
    target_audience: str = "intermediate",
    technical_domain: str = "ROS 2",
    service: BookContentService = Depends(get_book_content_service)
):
    """
    Enhance existing content with additional information or improvements.

    Args:
        existing_content: The existing content to enhance
        enhancement_request: Description of what enhancements to make
        target_audience: Target audience level for the enhancement
        technical_domain: Technical domain for the enhancement

    Returns:
        Dictionary containing enhanced content
    """
    try:
        enhanced_content = await service.enhance_existing_content(
            existing_content=existing_content,
            enhancement_request=enhancement_request,
            target_audience=target_audience,
            technical_domain=technical_domain
        )

        return {
            "enhanced_content": enhanced_content,
            "target_audience": target_audience,
            "technical_domain": technical_domain
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error enhancing content: {str(e)}")


# Health check endpoint
@router.get("/health")
async def health_check():
    """Health check endpoint for the book content service"""
    return {"status": "healthy", "service": "book-content-writer-agent"}