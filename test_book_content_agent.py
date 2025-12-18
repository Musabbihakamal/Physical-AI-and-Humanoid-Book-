#!/usr/bin/env python3
"""
Test script for Book Content Writer Agent functionality.
This script tests the new content extension and generation features.
"""
import asyncio
import json
from typing import Dict, Any

from backend.src.agents.book_content_writer.book_content_writer_agent import (
    BookContentWriterAgent,
    BookContentRequest,
    ContentComplexity
)
from backend.src.services.content_service import ContentService
from backend.src.services.book_content_service import BookContentService


async def test_book_content_generation():
    """Test the book content generation functionality"""
    print("Testing Book Content Writer Agent functionality...")

    # Create a content service (it doesn't take any parameters)
    content_service = ContentService()
    book_service = BookContentService(content_service)

    print("\n1. Testing basic chapter generation...")
    try:
        chapter = await book_service.generate_book_chapter(
            module_focus="ROS 2 Basics",
            chapter_title="Understanding ROS 2 Nodes",
            content_type="theory",
            target_audience="intermediate",
            technical_domain="ROS 2"
        )
        print(f"[SUCCESS] Generated chapter: {chapter.title}")
        print(f"  Quality score: {chapter.quality_score}")
    except Exception as e:
        print(f"[ERROR] Error generating chapter: {e}")

    print("\n2. Testing module content generation...")
    try:
        module_content = await book_service.generate_module_content(
            module_focus="ROS 2 Advanced Concepts",
            content_focus="ROS 2 Actions and Services",
            target_audience="intermediate",
            technical_domain="ROS 2",
            include_practical_examples=True,
            include_assessment=True
        )
        print(f"[SUCCESS] Generated module content (length: {len(module_content)} chars)")
    except Exception as e:
        print(f"[ERROR] Error generating module content: {e}")

    print("\n3. Testing content extension...")
    try:
        extended_content = await book_service.extend_chapter_content(
            chapter_id="test-chapter-123",
            extension_request="Add more examples about parameter handling in ROS 2 nodes",
            target_audience="intermediate",
            technical_domain="ROS 2"
        )
        print(f"[SUCCESS] Extended content (length: {len(extended_content)} chars)")
    except Exception as e:
        print(f"[ERROR] Error extending content: {e}")

    print("\n4. Testing content enhancement...")
    try:
        existing_content = """# ROS 2 Basics

## Overview Summary
This chapter covers the fundamentals of ROS 2.

## Learning Objectives
- Understand basic concepts
- Learn about nodes and topics
"""
        enhanced_content = await book_service.enhance_existing_content(
            existing_content=existing_content,
            enhancement_request="Add detailed explanation of QoS settings and their impact",
            target_audience="intermediate",
            technical_domain="ROS 2"
        )
        print(f"[SUCCESS] Enhanced content (length: {len(enhanced_content)} chars)")
        print("  Original length:", len(existing_content))
        print("  Enhanced length:", len(enhanced_content))
    except Exception as e:
        print(f"[ERROR] Error enhancing content: {e}")

    print("\n5. Testing exercises generation...")
    try:
        exercises = await book_service.get_recommended_exercises(
            chapter_content="This chapter covers ROS 2 nodes and their lifecycle",
            target_audience="intermediate"
        )
        print(f"[SUCCESS] Generated {len(exercises)} exercises")
        for i, exercise in enumerate(exercises[:2]):  # Show first 2
            print(f"  {i+1}. {exercise['title']}: {exercise['description']}")
    except Exception as e:
        print(f"[ERROR] Error generating exercises: {e}")

    print("\nAll tests completed!")


def test_agent_directly():
    """Test the Book Content Writer Agent directly"""
    print("\nTesting Book Content Writer Agent directly...")

    # Create a simple agent instance (without full service)
    from backend.src.services.content_service import ContentService
    content_service = ContentService()
    agent = BookContentWriterAgent(content_service)

    # Test request
    request = BookContentRequest(
        module_focus="ROS 2 Services",
        chapter_title="ROS 2 Services Deep Dive",
        content_type="comprehensive",
        target_audience=ContentComplexity.INTERMEDIATE,
        technical_domain="ROS 2",
        include_glossary=True,
        include_exercises=True,
        include_code_examples=True,
        include_diagrams=True
    )

    try:
        # This will use the mock _call_llm method
        result = asyncio.run(agent.generate_content(request))
        print(f"[SUCCESS] Direct agent call successful")
        print(f"  Content type: {result['content_type']}")
        print(f"  Quality score: {result['quality_score']}")
    except Exception as e:
        print(f"[ERROR] Error in direct agent call: {e}")


if __name__ == "__main__":
    print("Book Content Writer Agent - Functionality Test")
    print("=" * 50)

    # Test the full service
    asyncio.run(test_book_content_generation())

    # Test the agent directly
    test_agent_directly()

    print("\nTest completed successfully!")