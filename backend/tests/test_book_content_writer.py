"""
Tests for the Book Content Writer Agent

These tests verify that the agent generates content following the constitution requirements.
"""
import pytest
import asyncio
import sys
import os
from unittest.mock import Mock, AsyncMock

# Add the project root and src directory to the path to allow imports
project_root = os.path.join(os.path.dirname(os.path.dirname(__file__)))
src_dir = os.path.join(project_root, 'src')
shared_dir = os.path.join(project_root, 'shared')
sys.path.insert(0, project_root)
sys.path.insert(0, src_dir)
sys.path.insert(0, shared_dir)

# Import using absolute paths from the src directory
from src.agents.book_content_writer.book_content_writer_agent import (
    BookContentWriterAgent,
    BookContentRequest,
    ContentComplexity
)
from src.services.content_service import ContentService
from shared.utils.content_validation import (
    validate_technical_accuracy,
    validate_docusaurus_formatting,
    validate_educational_structure,
    validate_content_safety
)


@pytest.fixture
def mock_content_service():
    """Mock content service for testing"""
    return Mock(spec=ContentService)


@pytest.fixture
def book_content_agent(mock_content_service):
    """Create a book content writer agent for testing"""
    return BookContentWriterAgent(content_service=mock_content_service)


@pytest.mark.asyncio
async def test_book_content_agent_initialization(book_content_agent):
    """Test that the book content agent initializes correctly"""
    assert book_content_agent is not None
    assert book_content_agent.system_prompt is not None
    assert "Physical AI Book Constitution" in book_content_agent.system_prompt


@pytest.mark.asyncio
async def test_book_content_request_validation(book_content_agent):
    """Test validation of book content requests"""
    # Valid request
    request = BookContentRequest(
        module_focus="ROS 2 Basics",
        chapter_title="Introduction to ROS 2",
        content_type="theory",
        target_audience=ContentComplexity.BEGINNER,
        technical_domain="ROS 2"
    )

    # Should not raise an exception
    book_content_agent._validate_request(request)

    # Invalid content type
    with pytest.raises(ValueError, match="Invalid content type"):
        invalid_request = BookContentRequest(
            module_focus="ROS 2 Basics",
            chapter_title="Introduction to ROS 2",
            content_type="invalid_type",
            target_audience=ContentComplexity.BEGINNER,
            technical_domain="ROS 2"
        )
        book_content_agent._validate_request(invalid_request)

    # Invalid technical domain
    with pytest.raises(ValueError, match="Invalid technical domain"):
        invalid_request = BookContentRequest(
            module_focus="ROS 2 Basics",
            chapter_title="Introduction to ROS 2",
            content_type="theory",
            target_audience=ContentComplexity.BEGINNER,
            technical_domain="Invalid Domain"
        )
        book_content_agent._validate_request(invalid_request)


@pytest.mark.asyncio
async def test_prompt_construction(book_content_agent):
    """Test that prompts are constructed correctly"""
    request = BookContentRequest(
        module_focus="ROS 2 Basics",
        chapter_title="Introduction to ROS 2",
        content_type="theory",
        target_audience=ContentComplexity.BEGINNER,
        technical_domain="ROS 2",
        include_glossary=True,
        include_exercises=True,
        include_code_examples=True,
        include_diagrams=True
    )

    prompt = book_content_agent._construct_prompt(request)

    assert "ROS 2 Basics" in prompt
    assert "Introduction to ROS 2" in prompt
    assert "theory" in prompt
    assert "beginner" in prompt
    assert "ROS 2" in prompt
    assert "glossary" in prompt.lower()
    assert "exercises" in prompt.lower()
    assert "code examples" in prompt.lower()
    assert "diagrams" in prompt.lower()


@pytest.mark.asyncio
async def test_content_validation():
    """Test content validation utilities"""
    # Test technical accuracy validation
    valid_content = """
# Introduction to ROS 2
## Overview Summary
This chapter introduces the fundamental concepts of ROS 2, a middleware for robotics applications.

## Learning Objectives
- Understand ROS 2 concepts

## Detailed Theory
ROS 2 is a middleware for robotics applications.

## Practical Implementation
```python
import rclpy
from rclpy.node import Node
```

## Hands-on Exercises
1. Exercise 1
2. Exercise 2

## Glossary
- **Node**: A process that performs computation
"""

    tech_validation = validate_technical_accuracy(valid_content, "ROS 2")
    assert tech_validation["valid"]
    assert tech_validation["quality_score"] > 0.5

    # Test Docusaurus formatting validation
    format_validation = validate_docusaurus_formatting(valid_content)
    assert format_validation["valid"]

    # Test educational structure validation
    edu_validation = validate_educational_structure(valid_content)
    assert edu_validation["valid"]

    # Test safety validation
    safety_validation = validate_content_safety(valid_content)
    assert safety_validation["safe"]


@pytest.mark.asyncio
async def test_unsafe_content_detection():
    """Test that unsafe content is detected"""
    unsafe_content = """
# Introduction to ROS 2
## Detailed Theory
This is dangerous content that could be unsafe for students.
"""

    safety_validation = validate_content_safety(unsafe_content)
    assert not safety_validation["safe"]
    assert len(safety_validation["issues"]) > 0


@pytest.mark.asyncio
async def test_placeholder_detection():
    """Test that placeholders are detected"""
    content_with_placeholders = """
# Introduction to ROS 2
## Detailed Theory
This content has TODO items that need to be resolved.
And also has TKTK placeholders.
"""

    tech_validation = validate_technical_accuracy(content_with_placeholders, "ROS 2")
    assert not tech_validation["valid"]
    assert any("TODO" in issue or "TKTK" in issue for issue in tech_validation["issues"])


@pytest.mark.asyncio
async def test_docusaurus_formatting_detection():
    """Test Docusaurus formatting requirements"""
    valid_content = """
# Main Title
## Subsection
This is properly formatted content.

```python
print("Hello World")
```
"""

    format_validation = validate_docusaurus_formatting(valid_content)
    assert format_validation["valid"]

    invalid_content = "Just plain text without proper formatting"
    format_validation = validate_docusaurus_formatting(invalid_content)
    assert not format_validation["valid"]


@pytest.mark.asyncio
async def test_educational_structure_detection():
    """Test educational structure requirements"""
    valid_content = """
# Main Title
## Overview Summary
Brief summary of content.

## Learning Objectives
- Objective 1
- Objective 2

## Detailed Theory
Detailed explanation of concepts.

## Practical Implementation
How to implement the concepts.

## Hands-on Exercises
Exercises for students.

## Glossary
- **Term**: Definition
"""

    edu_validation = validate_educational_structure(valid_content)
    assert edu_validation["valid"]

    # Check which elements are present
    assert edu_validation["elements_present"]["Learning Objectives"]
    assert edu_validation["elements_present"]["Detailed Theory"]
    assert edu_validation["elements_present"]["Glossary"]


if __name__ == "__main__":
    pytest.main([__file__])