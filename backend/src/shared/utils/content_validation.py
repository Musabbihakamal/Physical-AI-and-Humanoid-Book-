"""
Content validation utilities for the book generation system.
Provides validation functions for technical accuracy, formatting, and safety.
"""
from typing import Dict, Any, List


def validate_technical_accuracy(content: str, technical_domain: str) -> Dict[str, Any]:
    """
    Validate technical accuracy of content for a specific domain.

    Args:
        content: The content to validate
        technical_domain: The technical domain (e.g., "ROS 2", "Isaac Sim")

    Returns:
        Dictionary with validation results
    """
    issues = []

    # Basic validation checks
    if len(content.strip()) < 50:
        issues.append("Content too short for technical validation")

    # Check for technical keywords based on domain
    domain_keywords = {
        "ROS 2": ["node", "topic", "service", "action", "package"],
        "Isaac Sim": ["simulation", "physics", "robot", "environment"],
        "Gazebo": ["world", "model", "plugin", "sensor"]
    }

    keywords = domain_keywords.get(technical_domain, [])
    if keywords and not any(keyword.lower() in content.lower() for keyword in keywords):
        issues.append(f"Missing key technical terms for {technical_domain}")

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "quality_score": max(0, 100 - len(issues) * 20)
    }


def validate_docusaurus_formatting(content: str) -> Dict[str, Any]:
    """
    Validate Docusaurus markdown formatting.

    Args:
        content: The content to validate

    Returns:
        Dictionary with validation results
    """
    issues = []

    # Check for proper markdown structure
    if not content.strip().startswith('#'):
        issues.append("Content should start with a heading")

    # Check for code blocks
    if '```' in content:
        code_blocks = content.count('```')
        if code_blocks % 2 != 0:
            issues.append("Unclosed code blocks detected")

    # Check for proper link formatting
    if '[' in content and ']' in content:
        # Basic link validation
        pass

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "format_compliance": max(0, 100 - len(issues) * 15)
    }


def validate_educational_structure(content: str) -> Dict[str, Any]:
    """
    Validate educational structure and learning objectives.

    Args:
        content: The content to validate

    Returns:
        Dictionary with validation results
    """
    issues = []

    # Check for educational elements
    if len(content.strip()) < 100:
        issues.append("Content too brief for educational purposes")

    # Check for learning structure
    educational_markers = ["##", "###", "example", "exercise", "summary"]
    if not any(marker.lower() in content.lower() for marker in educational_markers):
        issues.append("Missing educational structure elements")

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "structure_score": max(0, 100 - len(issues) * 25)
    }


def validate_content_safety(content: str) -> Dict[str, Any]:
    """
    Validate content safety and appropriateness.

    Args:
        content: The content to validate

    Returns:
        Dictionary with validation results
    """
    issues = []

    # Basic safety checks
    unsafe_patterns = [
        "hack", "crack", "exploit", "malicious", "dangerous",
        "illegal", "harmful", "inappropriate"
    ]

    content_lower = content.lower()
    for pattern in unsafe_patterns:
        if pattern in content_lower:
            issues.append(f"Potentially unsafe content detected: {pattern}")

    # Check content length
    if len(content.strip()) == 0:
        issues.append("Empty content")

    return {
        "safe": len(issues) == 0,
        "issues": issues,
        "safety_score": max(0, 100 - len(issues) * 30)
    }