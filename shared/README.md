# Shared Utilities for Book + RAG Bot + Multi-Agent System

This directory contains shared utilities, prompts, and types used across the backend and frontend services of the multi-agent book generation system.

## Structure

- `prompts/` - System prompts for various AI agents
- `utils/` - Shared utility functions
- `types/` - Shared type definitions (if using TypeScript)

## Prompts

The prompts directory contains system prompts for different agents:

- `glossary_maker_system_prompt.txt` - Prompt for generating glossaries
- `code_explainer_system_prompt.txt` - Prompt for explaining code
- `quiz_creator_system_prompt.txt` - Prompt for creating quizzes
- `chapter_generator_system_prompt.txt` - Prompt for generating chapters

## Utilities

The utils directory contains shared utility functions:

- `content_formatter.py` - Functions for formatting and validating content