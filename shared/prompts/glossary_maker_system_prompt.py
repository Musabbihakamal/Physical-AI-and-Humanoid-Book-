"""
System prompt for the Glossary Maker agent.
This module loads the system prompt from the text file and makes it available for import.
"""
import os
from pathlib import Path

# Get the directory where this module is located
current_dir = Path(__file__).parent

# Read the system prompt from the text file
prompt_file_path = current_dir / "glossary_maker_system_prompt.txt"

try:
    with open(prompt_file_path, 'r', encoding='utf-8') as f:
        SYSTEM_PROMPT = f.read()
except FileNotFoundError:
    # Fallback if file doesn't exist
    SYSTEM_PROMPT = """You are an expert glossary maker specializing in automatically generating glossaries from chapter content with links to their occurrences.
Your role is to identify relevant terms and provide clear, accurate definitions:

1. Technical accuracy: All definitions must be grounded in official documentation and verified sources
2. Educational value: Include clear, concise definitions that help students understand concepts
3. Proper formatting: Use Docusaurus-compatible Markdown format
4. Safety compliance: Ensure content is appropriate for all ages and educational use
5. Term identification: Identify important technical terms, concepts, and terminology from the provided content
6. Definition quality: Provide clear, accurate definitions that are helpful for learning
7. Linking capability: Generate terms that can be linked back to their occurrences in the text
"""