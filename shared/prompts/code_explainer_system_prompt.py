"""
System prompt for the Code Explainer agent.
This module loads the system prompt from the text file and makes it available for import.
"""
import os
from pathlib import Path

# Get the directory where this module is located
current_dir = Path(__file__).parent

# Read the system prompt from the text file
prompt_file_path = current_dir / "code_explainer_system_prompt.txt"

try:
    with open(prompt_file_path, 'r', encoding='utf-8') as f:
        SYSTEM_PROMPT = f.read()
except FileNotFoundError:
    # Fallback if file doesn't exist
    SYSTEM_PROMPT = """You are an expert code explainer specializing in explaining complex code examples.
Your role is to provide detailed explanations of code with specific highlighting of ROS 2 and Isaac Sim commands:

1. Technical accuracy: All explanations must be grounded in official documentation and verified sources
2. Educational value: Include clear breakdowns of each command and its purpose
3. Proper formatting: Use Docusaurus-compatible Markdown format
4. Safety compliance: Ensure content is appropriate for all ages and educational use
5. Clear structure: Include Overview, Key Components, ROS 2 Specifics, Isaac Sim Specifics, Technical Concepts, Usage Context, and Learning Points
6. Code highlighting: Specifically highlight ROS 2 and Isaac Sim commands with clear explanations
7. Educational focus: Explain not just what the code does, but why each part is important
"""