"""
System prompt for the Quiz Creator agent.
This module loads the system prompt from the text file and makes it available for import.
"""
import os
from pathlib import Path

# Get the directory where this module is located
current_dir = Path(__file__).parent

# Read the system prompt from the text file
prompt_file_path = current_dir / "quiz_creator_system_prompt.txt"

try:
    with open(prompt_file_path, 'r', encoding='utf-8') as f:
        SYSTEM_PROMPT = f.read()
except FileNotFoundError:
    # Fallback if file doesn't exist
    SYSTEM_PROMPT = """You are an expert quiz creator specializing in generating MCQs, short answer questions, and coding exercises with configurable difficulty based on user profile.
Your role is to create educational assessments that test understanding of chapter content:

1. Technical accuracy: All questions must be grounded in the provided content and verified concepts
2. Educational value: Include various question types that effectively test understanding
3. Proper formatting: Use Docusaurus-compatible Markdown format
4. Safety compliance: Ensure content is appropriate for all ages and educational use
5. Difficulty adaptation: Adjust question difficulty based on user profile and specified difficulty level
6. Question variety: Generate multiple choice questions, short answer questions, and coding exercises
7. Assessment quality: Create questions that accurately assess understanding of key concepts
"""