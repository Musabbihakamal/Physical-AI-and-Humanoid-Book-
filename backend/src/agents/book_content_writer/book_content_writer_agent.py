"""
Book Content Writer Agent

This agent generates book content following the Physical AI Book Constitution requirements.
It ensures technical accuracy, proper formatting, and educational value while maintaining
safety standards for educational content.
"""
import asyncio
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum

try:
    # Try importing from the installed package structure
    from shared.prompts.book_content_writer_system_prompt import SYSTEM_PROMPT
except ImportError:
    # Fallback to relative import for development
    import sys
    import os
    from pathlib import Path
    project_root = Path(__file__).parent.parent.parent.parent
    sys.path.insert(0, str(project_root))
    from shared.prompts.book_content_writer_system_prompt import SYSTEM_PROMPT

from ...models.agent_request import AgentRequest
from ...models.generated_content import GeneratedContent
from ...services.content_service import ContentService


class ContentComplexity(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    EXPERT = "expert"


@dataclass
class BookContentRequest:
    """Request object for book content generation"""
    module_focus: str
    chapter_title: str
    content_type: str  # "theory", "code", "exercises", "diagrams", "overview"
    target_audience: ContentComplexity
    technical_domain: str  # "ROS 2", "Isaac Sim", "Gazebo", etc.
    user_profile: Optional[Dict[str, Any]] = None
    include_glossary: bool = True
    include_exercises: bool = True
    include_code_examples: bool = True
    include_diagrams: bool = True


class BookContentWriterAgent:
    """
    Agent responsible for generating book content following constitution requirements:
    - Technical accuracy grounded in official documentation
    - Docusaurus-compatible Markdown formatting
    - Educational value with learning objectives and exercises
    - Safety compliance for minor-appropriate content
    - ROS 2 and Isaac Sim specific command highlighting
    """

    def __init__(self, content_service: ContentService):
        self.content_service = content_service
        self.system_prompt = SYSTEM_PROMPT

    async def generate_content(self, request: BookContentRequest) -> GeneratedContent:
        """
        Generate book content based on the request parameters.

        Args:
            request: BookContentRequest containing generation parameters

        Returns:
            GeneratedContent with properly formatted book content
        """
        # Validate request against constitution requirements
        self._validate_request(request)

        # Construct the prompt following constitution formatting standards
        prompt = self._construct_prompt(request)

        # Generate content using the LLM
        raw_content = await self._call_llm(prompt)

        # Validate the generated content for technical accuracy
        validated_content = await self._validate_content_accuracy(
            raw_content,
            request.technical_domain
        )

        # Format content according to Docusaurus Markdown standards
        formatted_content = self._format_docusaurus_markdown(
            validated_content,
            request
        )

        # Return a dictionary with the generated content and metadata instead of GeneratedContent object
        return {
            "content_type": "book_chapter",
            "content": formatted_content,
            "metadata": {
                "module_focus": request.module_focus,
                "chapter_title": request.chapter_title,
                "target_audience": request.target_audience.value,
                "technical_domain": request.technical_domain,
                "include_glossary": request.include_glossary,
                "include_exercises": request.include_exercises,
                "include_code_examples": request.include_code_examples,
                "include_diagrams": request.include_diagrams
            },
            "quality_score": self._calculate_quality_score(formatted_content)
        }

    def _validate_request(self, request: BookContentRequest) -> None:
        """Validate the request against constitution requirements"""
        # Ensure content type is valid
        valid_content_types = ["theory", "code", "exercises", "diagrams", "overview", "practical", "assessment", "extension", "comprehensive", "enhancement", "detailed_explanation", "advanced_concepts"]
        if request.content_type not in valid_content_types:
            raise ValueError(f"Invalid content type: {request.content_type}. Must be one of {valid_content_types}")

        # Ensure technical domain is valid
        valid_domains = ["ROS 2", "Isaac Sim", "Gazebo", "Unity", "NVIDIA Isaac", "RL", "VLA", "Physical AI"]
        if request.technical_domain not in valid_domains:
            raise ValueError(f"Invalid technical domain: {request.technical_domain}. Must be one of {valid_domains}")

    def _construct_prompt(self, request: BookContentRequest) -> str:
        """Construct the LLM prompt following constitution formatting standards"""
        prompt_parts = [
            self.system_prompt,
            f"\n# Module Focus: {request.module_focus}",
            f"\n## Chapter Title: {request.chapter_title}",
            f"\n## Content Type: {request.content_type}",
            f"\n## Target Audience: {request.target_audience.value}",
            f"\n## Technical Domain: {request.technical_domain}",
        ]

        # Add content type-specific instructions
        if request.content_type == "extension":
            prompt_parts.extend([
                f"\n## Content Extension Request:",
                f"- The content should extend existing material based on: {request.module_focus}",
                f"- Focus on expanding specific areas mentioned in the request",
                f"- Maintain consistency with existing content style and structure",
            ])
        elif request.content_type == "comprehensive":
            prompt_parts.extend([
                f"\n## Comprehensive Content Request:",
                f"- Create comprehensive coverage of: {request.module_focus} focusing on {request.chapter_title}",
                f"- Include all essential aspects of the topic for {request.target_audience.value} level",
                f"- Ensure thorough coverage with multiple sections and examples",
            ])
        elif request.content_type == "enhancement":
            prompt_parts.extend([
                f"\n## Content Enhancement Request:",
                f"- Enhance existing content based on: {request.module_focus}",
                f"- Focus on improving understanding and depth of existing concepts",
                f"- Add advanced insights or clearer explanations as requested",
            ])
        elif request.content_type == "detailed_explanation":
            prompt_parts.extend([
                f"\n## Detailed Explanation Request:",
                f"- Provide in-depth technical explanation of: {request.module_focus}",
                f"- Focus on technical depth and comprehensive understanding",
                f"- Include implementation details, best practices, and advanced concepts",
                f"- Use appropriate complexity level for {request.target_audience.value} audience",
            ])
        elif request.content_type == "advanced_concepts":
            prompt_parts.extend([
                f"\n## Advanced Concepts Request:",
                f"- Cover advanced topics related to: {request.module_focus}",
                f"- Focus on complex implementations, optimization, and edge cases",
                f"- Include advanced patterns, performance considerations, and expert-level insights",
                f"- Target {request.target_audience.value} level with sophisticated concepts",
            ])
        else:
            prompt_parts.append(f"\n## Content Requirements:")

        # Add standard requirements
        prompt_parts.extend([
            f"- Technical accuracy grounded in official documentation",
            f"- Docusaurus-compatible Markdown format",
            f"- Include Overview Summary and Learning Objectives",
            f"- Include Detailed Theory and Practical Implementation Steps",
            f"- Include Python & {request.technical_domain} code examples following proper conventions",
            f"- Include Diagrams/Flowcharts (described in Mermaid format)",
            f"- Include Hands-on Exercises and Assessment Questions",
            f"- Code samples must follow {request.technical_domain} conventions and Python PEP8 style",
            f"- Ensure content is safe for minors and educational use",
        ])

        if request.include_glossary:
            prompt_parts.append("- Automatically generate a glossary of technical terms")

        if request.include_exercises:
            prompt_parts.append("- Include relevant exercises for the target audience level")

        if request.include_code_examples:
            prompt_parts.append(f"- Include proper {request.technical_domain} code examples with explanations")

        if request.include_diagrams:
            prompt_parts.append("- Include Mermaid diagram descriptions where appropriate")

        if request.user_profile:
            prompt_parts.append(f"\n## User Profile Context: {request.user_profile}")
            prompt_parts.append("- Adapt complexity based on user profile information")

        prompt_parts.append("\n## Content:")

        return "\n".join(prompt_parts)

    async def _call_llm(self, prompt: str) -> str:
        """Call the LLM to generate raw content"""
        # This would integrate with your LLM provider (OpenAI, Claude, etc.)
        # For now, we'll simulate the call
        import time
        time.sleep(0.1)  # Simulate API call delay

        # In a real implementation, this would call the actual LLM
        # return await llm_client.generate(prompt)

        # For demonstration, return generic content (the actual content would come from LLM)
        return "This is generated content for the book chapter. In a real implementation, this would be generated by an LLM based on the provided prompt."

    async def _validate_content_accuracy(self, content: str, domain: str) -> str:
        """Validate the technical accuracy of generated content"""
        # In a real implementation, this would:
        # 1. Check against official documentation
        # 2. Verify code examples
        # 3. Validate technical concepts

        # For now, we'll return the content as-is with a basic check
        if "TODO" in content or "TKTK" in content:
            raise ValueError("Generated content contains unresolved placeholders")

        # Check for basic safety compliance
        safety_keywords = ["dangerous", "unsafe", "harmful", "bypass"]
        for keyword in safety_keywords:
            if keyword.lower() in content.lower():
                raise ValueError(f"Content contains potentially unsafe content: {keyword}")

        return content

    def _format_docusaurus_markdown(self, content: str, request: BookContentRequest) -> str:
        """Format content according to Docusaurus Markdown standards"""
        formatted_parts = []

        # Add main title
        formatted_parts.append(f"# {request.chapter_title}")
        formatted_parts.append("")

        # Add overview summary
        formatted_parts.append("## Overview Summary")
        formatted_parts.append("")
        formatted_parts.append("Brief summary of the chapter content and key concepts.")
        formatted_parts.append("")

        # Add learning objectives
        formatted_parts.append("## Learning Objectives")
        formatted_parts.append("")
        formatted_parts.append("- Understand key concepts related to " + request.module_focus)
        formatted_parts.append("- Apply " + request.technical_domain + " principles in practice")
        formatted_parts.append("- Implement practical examples and exercises")
        formatted_parts.append("")

        # Add detailed theory section
        formatted_parts.append("## Detailed Theory")
        formatted_parts.append("")
        formatted_parts.append(content)  # Insert the generated content
        formatted_parts.append("")

        # Add practical implementation if requested
        if request.include_code_examples:
            formatted_parts.append("## Practical Implementation")
            formatted_parts.append("")
            formatted_parts.append("```python")
            formatted_parts.append("# Example code following " + request.technical_domain + " conventions")
            if request.technical_domain == "ROS 2":
                formatted_parts.append("import rclpy")
                formatted_parts.append("from rclpy.node import Node")
            formatted_parts.append("# This would be properly formatted code")
            formatted_parts.append("```")
            formatted_parts.append("")

        # Add exercises if requested
        if request.include_exercises:
            formatted_parts.append("## Hands-on Exercises")
            formatted_parts.append("")
            formatted_parts.append("1. Exercise related to " + request.module_focus)
            formatted_parts.append("2. Practical application of " + request.technical_domain)
            formatted_parts.append("")

        # Add glossary if requested
        if request.include_glossary:
            formatted_parts.append("## Glossary")
            formatted_parts.append("")
            formatted_parts.append("- **Term**: Definition")
            formatted_parts.append("")

        # Add diagrams section if requested
        if request.include_diagrams:
            formatted_parts.append("## Diagrams")
            formatted_parts.append("")
            formatted_parts.append("```mermaid")
            formatted_parts.append("# Diagram description would go here")
            formatted_parts.append("```")
            formatted_parts.append("")

        return "\n".join(formatted_parts)

    def _calculate_quality_score(self, content: str) -> float:
        """Calculate a quality score for the generated content"""
        # Basic quality metrics
        score = 0.0

        # Check for required sections
        if "## Learning Objectives" in content:
            score += 20
        if "## Detailed Theory" in content:
            score += 20
        if "## Practical Implementation" in content:
            score += 15
        if "## Hands-on Exercises" in content:
            score += 15
        if "## Glossary" in content:
            score += 10

        # Check for code examples
        if "```python" in content:
            score += 10

        # Check for diagrams
        if "```mermaid" in content:
            score += 10

        # Ensure it doesn't exceed 100
        return min(score, 100.0) / 100.0  # Normalize to 0-1 scale