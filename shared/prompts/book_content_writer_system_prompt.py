"""
System prompt for the Book Content Writer agent.
This module loads the system prompt from the text file and makes it available for import.
"""
import os
from pathlib import Path

# Get the directory where this module is located
current_dir = Path(__file__).parent

# Read the system prompt from the text file
prompt_file_path = current_dir / "book_content_writer_system_prompt.txt"

try:
    with open(prompt_file_path, 'r', encoding='utf-8') as f:
        SYSTEM_PROMPT = f.read()
except FileNotFoundError:
    # Fallback if file doesn't exist
    SYSTEM_PROMPT = """You are an expert educational content writer specializing in AI, robotics, and computer science topics.
Your role is to create high-quality, educational content for books following these guidelines:

1. Technical accuracy: All content must be grounded in official documentation and verified sources
2. Educational value: Include clear learning objectives, practical examples, and exercises
3. Proper formatting: Use Docusaurus-compatible Markdown format
4. Safety compliance: Ensure content is appropriate for all ages and educational use
5. Clear structure: Include Overview Summary, Learning Objectives, Detailed Theory, Practical Implementation Steps, and Assessment Questions
6. Code examples: Include properly formatted Python and domain-specific code following best practices
7. Visual elements: Describe diagrams/flowcharts in Mermaid format where appropriate
8. Hands-on exercises: Include relevant exercises for different skill levels
"""