"""
Content Validation Utilities

This module provides utilities for validating book content according to the constitution requirements.
"""
import re
from typing import List, Dict, Any
from urllib.parse import urlparse


def validate_technical_accuracy(content: str, domain: str = "ROS 2") -> Dict[str, Any]:
    """
    Validate the technical accuracy of book content.

    Args:
        content: The content to validate
        domain: The technical domain to validate against

    Returns:
        Dictionary with validation results and issues found
    """
    issues = []

    # Check for common ROS 2 code patterns only if code blocks are present
    if domain in ["ROS 2", "Isaac Sim"]:
        # Only check for ROS 2 imports if there are Python code blocks
        python_code_blocks = re.findall(r'```python.*?```', content, re.DOTALL)
        if python_code_blocks:
            # Check for proper ROS 2 imports in Python code blocks
            all_code = ' '.join(python_code_blocks)
            ros_imports = re.findall(r'import\s+rclpy', all_code)
            if not ros_imports:
                issues.append("Missing rclpy import in ROS 2 Python code - ROS 2 code should import rclpy")

            # Check for proper node inheritance in classes within Python code blocks
            node_patterns = re.findall(r'class\s+\w+Node\(\w*Node\)', all_code)
            if not node_patterns and "class" in all_code:
                issues.append("ROS 2 nodes should inherit from Node class")

    # Check for safety violations
    safety_violations = [
        "dangerous", "unsafe", "harmful", "bypass safety", "override protection",
        "high voltage", "without supervision", "risk of injury"
    ]

    for violation in safety_violations:
        if violation.lower() in content.lower():
            issues.append(f"Potential safety violation detected: {violation}")

    # Check for placeholders
    placeholders = re.findall(r'TODO|TKTK|\?\?\?|<placeholder>', content)
    if placeholders:
        issues.extend([f"Unresolved placeholder found: {ph}" for ph in placeholders])

    # Check for basic formatting requirements
    required_sections = ["Overview Summary", "Learning Objectives", "Detailed Theory"]
    missing_sections = [section for section in required_sections if section not in content]
    if missing_sections:
        issues.append(f"Missing required sections: {missing_sections}")

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "quality_score": max(0, 100 - len(issues) * 10) / 100.0  # Simple scoring
    }


def validate_docusaurus_formatting(content: str) -> Dict[str, Any]:
    """
    Validate that content follows Docusaurus Markdown formatting standards.

    Args:
        content: The content to validate

    Returns:
        Dictionary with formatting validation results
    """
    issues = []

    # Check for proper heading structure
    headings = re.findall(r'^(#{1,6})\s', content, re.MULTILINE)
    if not headings:
        issues.append("No headings found - content should use #, ##, ### for structure")

    # Check for main title (should have one #)
    main_headings = [h for h in headings if h == '#']
    if not main_headings:
        issues.append("No main title found - content should start with # for main title")

    # Check for code blocks
    code_blocks = re.findall(r'```.*?```', content, re.DOTALL)
    if not code_blocks:
        issues.append("No code blocks found - technical content should include code examples")

    # Check for proper code language specification
    lang_specs = re.findall(r'```(\w+)', content)
    valid_langs = ['python', 'bash', 'xml', 'yaml', 'json', 'cpp', 'c', 'javascript', 'mermaid']
    invalid_langs = [lang for lang in lang_specs if lang.lower() not in [v.lower() for v in valid_langs]]
    if invalid_langs:
        issues.extend([f"Invalid code language: {lang}" for lang in invalid_langs])

    return {
        "valid": len(issues) == 0,
        "issues": issues,
        "format_compliance": max(0, 100 - len(issues) * 5) / 100.0
    }


def validate_educational_structure(content: str) -> Dict[str, Any]:
    """
    Validate that content follows the required educational structure.

    Args:
        content: The content to validate

    Returns:
        Dictionary with educational structure validation results
    """
    required_elements = {
        "Overview Summary": "## Overview Summary" in content,
        "Learning Objectives": "## Learning Objectives" in content,
        "Detailed Theory": "## Detailed Theory" in content,
        "Practical Implementation": "## Practical Implementation" in content,
        "Exercises": "## Hands-on Exercises" in content or "Exercises" in content,
        "Glossary": "## Glossary" in content
    }

    missing_elements = [elem for elem, present in required_elements.items() if not present]

    issues = []
    if missing_elements:
        issues.append(f"Missing required educational elements: {missing_elements}")

    return {
        "valid": len(missing_elements) == 0,
        "issues": issues,
        "elements_present": required_elements,
        "structure_score": (len(required_elements) - len(missing_elements)) / len(required_elements)
    }


def validate_content_safety(content: str) -> Dict[str, Any]:
    """
    Validate that content is safe for educational use, especially for minors.

    Args:
        content: The content to validate

    Returns:
        Dictionary with safety validation results
    """
    safety_issues = []

    # Check for potentially unsafe content
    unsafe_patterns = [
        r'dangerous\s+\w+',
        r'unsafe\s+\w+',
        r'harmful\s+\w+',
        r'bypass\s+safety',
        r'override\s+safet[yi]',
        r'high\s+voltage',
        r'without\s+supervision',
        r'risk\s+of\s+injury',
        r'without\s+protection',
        r'ignore\s+safety'
    ]

    for pattern in unsafe_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            safety_issues.extend([f"Potentially unsafe content: {match}" for match in matches])

    return {
        "safe": len(safety_issues) == 0,
        "issues": safety_issues,
        "safety_score": 1.0 if len(safety_issues) == 0 else 0.0
    }