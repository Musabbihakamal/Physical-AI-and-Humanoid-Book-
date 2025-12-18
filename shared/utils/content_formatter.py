"""
Content formatting utilities for the multi-agent book generation system.
Provides common formatting functions used across different agents.
"""

import re
from typing import Dict, List, Any


def format_as_markdown(content: str, title: str = None) -> str:
    """
    Format content as Docusaurus-compatible Markdown.

    Args:
        content: The content to format
        title: Optional title for the document

    Returns:
        Formatted Markdown string
    """
    if title:
        formatted = f"# {title}\n\n{content}"
    else:
        formatted = content

    # Ensure proper spacing and formatting
    formatted = re.sub(r'\n\s*\n', '\n\n', formatted)  # Remove excessive blank lines
    return formatted


def extract_terms_from_content(content: str) -> List[str]:
    """
    Extract potential glossary terms from content using pattern recognition.

    Args:
        content: The content to analyze

    Returns:
        List of potential terms
    """
    # Look for technical terms (capitalized words, code-like terms, etc.)
    patterns = [
        r'\b[A-Z][A-Z_]+\b',  # Constants like API_ROS
        r'\b[A-Z][a-z]+\b',   # CamelCase terms
        r'`[^`]+`',           # Code blocks
        r'\*\*[^*]+\*\*',     # Bold text
    ]

    terms = []
    for pattern in patterns:
        matches = re.findall(pattern, content)
        terms.extend([match.strip('`**') for match in matches])

    # Remove duplicates while preserving order
    unique_terms = []
    for term in terms:
        if term not in unique_terms and len(term) > 2:  # Filter out very short terms
            unique_terms.append(term)

    return unique_terms


def validate_content_quality(content: str, min_length: int = 100) -> Dict[str, Any]:
    """
    Validate the quality of generated content.

    Args:
        content: The content to validate
        min_length: Minimum length requirement

    Returns:
        Dictionary with validation results
    """
    result = {
        'is_valid': True,
        'issues': [],
        'quality_score': 0
    }

    if len(content) < min_length:
        result['is_valid'] = False
        result['issues'].append(f'Content too short (minimum {min_length} characters)')

    # Check for basic structure (headers, paragraphs)
    has_headers = bool(re.search(r'^#+\s', content, re.MULTILINE))
    has_paragraphs = len(content.split('\n\n')) > 1

    if not has_headers and not has_paragraphs:
        result['issues'].append('Content lacks proper structure')

    # Calculate quality score based on various factors
    score = 50  # Base score

    if has_headers:
        score += 15
    if has_paragraphs:
        score += 15
    if len(content) >= min_length * 2:
        score += 10
    if len(result['issues']) == 0:
        score += 20

    result['quality_score'] = min(100, score)

    return result


def sanitize_content(content: str) -> str:
    """
    Sanitize content to remove potentially problematic elements.

    Args:
        content: The content to sanitize

    Returns:
        Sanitized content
    """
    # Remove potentially dangerous patterns
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', content, flags=re.IGNORECASE | re.DOTALL)
    sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)

    # Normalize whitespace
    sanitized = re.sub(r'[ \t]+', ' ', sanitized)  # Multiple spaces/tabs to single space
    sanitized = re.sub(r'\n\s*\n\s*\n+', '\n\n', sanitized)  # Multiple blank lines to single

    return sanitized