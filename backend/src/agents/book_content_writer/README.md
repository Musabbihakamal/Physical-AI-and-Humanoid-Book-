# Book Content Writer Agent

The Book Content Writer Agent is a specialized AI agent designed to generate educational content for the Physical AI & Humanoid Robotics book, following the strict requirements of the Physical AI Book Constitution.

## Features

- **Constitution Compliance**: Ensures all generated content follows the Physical AI Book Constitution requirements
- **Technical Accuracy**: Validates content against official documentation and technical standards
- **Docusaurus Compatibility**: Generates content in Docusaurus-compatible Markdown format
- **Educational Structure**: Includes required sections: Overview Summary, Learning Objectives, Detailed Theory, Practical Implementation, Exercises, and Glossary
- **Safety Validation**: Ensures content is safe for educational use, especially for minors
- **Personalization**: Adapts content complexity based on user profiles
- **Multi-Domain Support**: Handles ROS 2, Isaac Sim, Gazebo, and other robotics frameworks

## Architecture

```
backend/
├── src/
│   ├── agents/
│   │   └── book_content_writer/
│   │       ├── book_content_writer_agent.py  # Main agent implementation
│   │       └── __init__.py
│   ├── services/
│   │   └── book_content_service.py          # Service layer with validation
│   ├── models/
│   │   └── book_chapter.py                  # Enhanced chapter model
│   └── api/
│       └── book_content_routes.py           # API endpoints
├── shared/
│   ├── prompts/
│   │   └── book_content_writer_system_prompt.txt  # System prompt
│   └── utils/
│       └── content_validation.py            # Validation utilities
└── tests/
    └── test_book_content_writer.py          # Tests
```

## API Endpoints

### Generate Chapter
```
POST /api/book-content/generate-chapter
```

**Request Parameters:**
- `module_focus` (string): The focus area of the module
- `chapter_title` (string): Title of the chapter to generate
- `content_type` (string): Type of content ("theory", "code", "exercises", etc.)
- `target_audience` (string): Audience level ("beginner", "intermediate", "expert")
- `technical_domain` (string): Technical domain ("ROS 2", "Isaac Sim", etc.)
- `user_profile` (object, optional): User profile for personalization
- `include_glossary` (boolean, default: true): Whether to include a glossary
- `include_exercises` (boolean, default: true): Whether to include exercises
- `include_code_examples` (boolean, default: true): Whether to include code examples
- `include_diagrams` (boolean, default: true): Whether to include diagrams

**Example Request:**
```bash
curl -X POST "http://localhost:8000/api/book-content/generate-chapter" \
  -H "Content-Type: application/json" \
  -d '{
    "module_focus": "ROS 2 Basics",
    "chapter_title": "Introduction to ROS 2 Nodes",
    "content_type": "theory",
    "target_audience": "beginner",
    "technical_domain": "ROS 2",
    "user_profile": {
      "experience_level": "beginner",
      "hardware_access": "simulation"
    }
  }'
```

### Validate Content
```
POST /api/book-content/validate-content
```

Validates existing content against constitution requirements.

### Update Chapter
```
POST /api/book-content/update-chapter/{chapter_id}
```

Updates existing chapter content with validation.

## Constitution Compliance

The agent ensures compliance with the Physical AI Book Constitution by:

1. **Technical Accuracy**: All content is grounded in official documentation
2. **Docusaurus Formatting**: Content follows Docusaurus-compatible Markdown standards
3. **Educational Structure**: Every chapter includes required sections
4. **Safety Standards**: Content is validated to be safe for educational use
5. **Code Quality**: All code examples follow ROS 2 conventions and PEP8 style
6. **Personalization**: Content adapts to user profiles while maintaining core technical content

## Validation Process

Each piece of generated content undergoes multiple validation steps:

1. **Technical Accuracy Validation**: Checks against official documentation
2. **Docusaurus Formatting Validation**: Ensures proper Markdown structure
3. **Educational Structure Validation**: Verifies required sections are present
4. **Safety Validation**: Ensures content is appropriate for educational use
5. **Comprehensive Scoring**: Calculates overall quality score based on all validations

## Content Structure

Generated chapters follow this structure:

```markdown
---
title: Chapter Title
sidebar_position: 1
description: Chapter about the topic
---

# Chapter Title

## Overview Summary
Brief summary of the chapter content and key concepts.

## Learning Objectives
- Understand key concepts related to the module focus
- Apply technical domain principles in practice
- Implement practical examples and exercises

## Detailed Theory
[Generated content with detailed explanations]

## Practical Implementation
```python
# Example code following technical domain conventions
```

## Hands-on Exercises
1. Exercise related to the module focus
2. Practical application of the technical domain

## Glossary
- **Term**: Definition

## Diagrams
```mermaid
# Diagram description would go here
```
```

## Development

To run the tests:
```bash
cd backend
pytest tests/test_book_content_writer.py
```

## Integration

The Book Content Writer Agent integrates with:
- Content Service for storage and retrieval
- Database models for persistence
- API layer for external access
- Validation utilities for quality assurance