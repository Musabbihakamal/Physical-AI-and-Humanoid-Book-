"""
Book chapter model for the multi-agent book generation system.
This model follows the Physical AI Book Constitution requirements for educational content.
"""
from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey, JSON
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from typing import Optional, Dict, Any, List
import uuid


class BookChapter(Base):
    __tablename__ = "book_chapters"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    content = Column(Text, nullable=False)  # Docusaurus-compatible Markdown
    module_focus = Column(String, nullable=True)  # the module or topic the chapter covers
    word_count = Column(Integer, default=0)
    difficulty_level = Column(String, default="INTERMEDIATE")  # BEGINNER, INTERMEDIATE, EXPERT
    technical_domain = Column(String, nullable=True)  # ROS 2, Isaac Sim, Gazebo, etc.
    validation_results = Column(JSON, nullable=True)  # Validation results following constitution
    quality_score = Column(Integer, default=0)  # Quality score from 0-100
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    generated_by = Column(UUID_TYPE, ForeignKey("agent_requests.id"), nullable=True)

    def __init__(self,
                 title: str,
                 content: str,
                 module_focus: Optional[str] = None,
                 difficulty_level: str = "INTERMEDIATE",
                 technical_domain: Optional[str] = None,
                 validation_results: Optional[Dict[str, Any]] = None,
                 quality_score: int = 0,
                 generated_by: Optional[str] = None):
        # Validation
        valid_difficulty_levels = ["BEGINNER", "INTERMEDIATE", "EXPERT"]
        if difficulty_level not in valid_difficulty_levels:
            raise ValueError(f"difficulty_level must be one of {valid_difficulty_levels}")

        if not title:
            raise ValueError("title must not be empty")

        # Validate content follows constitution requirements
        self._validate_constitution_requirements(content)

        self.title = title
        self.content = content
        self.module_focus = module_focus
        self.difficulty_level = difficulty_level
        self.technical_domain = technical_domain
        self.validation_results = validation_results or {}
        self.quality_score = quality_score
        self.generated_by = generated_by
        self.word_count = len(content.split()) if content else 0

    def _validate_constitution_requirements(self, content: str) -> None:
        """Validate that the chapter content meets constitution requirements"""
        errors = []

        # Check for required educational structure
        required_sections = [
            "## Learning Objectives",
            "## Detailed Theory",
            "## Practical Implementation",
            "## Hands-on Exercises",
            "## Glossary"
        ]

        for section in required_sections:
            if section not in content:
                errors.append(f"Missing required section: {section}")

        # Check for safety compliance
        safety_keywords = ["dangerous", "unsafe", "harmful", "bypass safety"]
        for keyword in safety_keywords:
            if keyword.lower() in content.lower():
                errors.append(f"Content contains potentially unsafe content: {keyword}")

        # Check for Docusaurus formatting
        if not content.startswith("# "):
            errors.append("Content should start with a main title using #")

        # Check for code examples in technical content
        if "code" in content.lower() and "```" not in content:
            errors.append("Technical content should include code examples in fenced blocks")

        if errors:
            raise ValueError(f"Chapter content validation failed: {'; '.join(errors)}")

    def get_docusaurus_compatible_content(self) -> str:
        """Return content formatted for Docusaurus compatibility with frontmatter"""
        # Add standard frontmatter if not present
        if not self.content.startswith("---"):
            frontmatter = f"""---
title: {self.title}
sidebar_position: 1
description: Chapter about {self.module_focus or 'the topic'}
---

"""
            return frontmatter + self.content

        return self.content

    def get_learning_objectives(self) -> List[str]:
        """Extract learning objectives from the content"""
        # Find the Learning Objectives section and extract items
        lines = self.content.split('\n')
        in_objectives = False
        objectives = []

        for line in lines:
            if "## Learning Objectives" in line:
                in_objectives = True
                continue
            elif line.startswith("## ") and in_objectives:
                # End of objectives section
                break
            elif in_objectives and line.strip().startswith("- "):
                objectives.append(line.strip()[2:])

        return objectives

    def get_glossary_terms(self) -> Dict[str, str]:
        """Extract glossary terms from the content"""
        lines = self.content.split('\n')
        in_glossary = False
        terms = {}

        for line in lines:
            if "## Glossary" in line:
                in_glossary = True
                continue
            elif line.startswith("## ") and in_glossary:
                # End of glossary section
                break
            elif in_glossary and line.strip().startswith("- **") and ": " in line:
                # Format: - **Term**: Definition
                try:
                    term_part = line.split("**")[1]  # Get content between first **
                    definition = line.split(": ")[1].strip()
                    terms[term_part] = definition
                except IndexError:
                    continue

        return terms