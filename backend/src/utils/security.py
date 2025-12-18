"""
Security utilities for the multi-agent book generation system.
"""
import re
from typing import Dict, Any, Optional
from .rate_limiter import check_rate_limit
import logging


logger = logging.getLogger(__name__)


class SecurityValidator:
    """
    Security validation utilities for agent requests.
    """

    # Define allowed agent types to prevent unauthorized agent execution
    ALLOWED_AGENT_TYPES = {
        "GLOSSARY_MAKER",
        "CODE_EXPLAINER",
        "QUIZ_CREATOR",
        "CHAPTER_GENERATOR",
        "BOOK_CONTENT_WRITER"
    }

    # Maximum content length to prevent large payload attacks
    MAX_CONTENT_LENGTH = 100000  # 100KB

    # Maximum parameters length to prevent large parameter attacks
    MAX_PARAMETERS_LENGTH = 10000  # 10KB

    # Pattern for validating user IDs (UUID format)
    UUID_PATTERN = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')

    # Pattern for validating safe content (basic check for potentially dangerous content)
    DANGEROUS_PATTERNS = [
        re.compile(r'<script', re.IGNORECASE),  # Prevent XSS
        re.compile(r'javascript:', re.IGNORECASE),  # Prevent XSS
        re.compile(r'vbscript:', re.IGNORECASE),  # Prevent XSS
        re.compile(r'on\w+\s*=', re.IGNORECASE),  # Prevent event handlers
    ]

    @classmethod
    def validate_agent_type(cls, agent_type: str) -> bool:
        """
        Validate that the agent type is allowed.

        Args:
            agent_type: Type of agent to validate

        Returns:
            True if agent type is allowed, False otherwise
        """
        return agent_type in cls.ALLOWED_AGENT_TYPES

    @classmethod
    def validate_content_length(cls, content: str) -> bool:
        """
        Validate that content length is within safe limits.

        Args:
            content: Content to validate

        Returns:
            True if content length is acceptable, False otherwise
        """
        return len(content) <= cls.MAX_CONTENT_LENGTH

    @classmethod
    def validate_parameters_length(cls, parameters: Dict[str, Any]) -> bool:
        """
        Validate that parameters length is within safe limits.

        Args:
            parameters: Parameters to validate

        Returns:
            True if parameters length is acceptable, False otherwise
        """
        import json
        params_str = json.dumps(parameters)
        return len(params_str) <= cls.MAX_PARAMETERS_LENGTH

    @classmethod
    def validate_user_id_format(cls, user_id: Optional[str]) -> bool:
        """
        Validate that user ID format is correct (UUID).

        Args:
            user_id: User ID to validate

        Returns:
            True if user ID format is correct, False otherwise
        """
        if not user_id:
            return True  # User ID is optional
        return bool(cls.UUID_PATTERN.match(user_id))

    @classmethod
    def validate_content_safety(cls, content: str) -> bool:
        """
        Validate that content doesn't contain dangerous patterns.

        Args:
            content: Content to validate

        Returns:
            True if content is safe, False otherwise
        """
        for pattern in cls.DANGEROUS_PATTERNS:
            if pattern.search(content):
                return False
        return True

    @classmethod
    def sanitize_content(cls, content: str) -> str:
        """
        Basic content sanitization.

        Args:
            content: Content to sanitize

        Returns:
            Sanitized content
        """
        # Remove potentially dangerous patterns
        sanitized = content
        for pattern in cls.DANGEROUS_PATTERNS:
            # This is a basic sanitization - in production, use a proper HTML sanitizer
            sanitized = pattern.sub('', sanitized)
        return sanitized

    @classmethod
    def validate_agent_request(cls, agent_type: str, content: Optional[str] = None,
                              parameters: Optional[Dict[str, Any]] = None,
                              user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Comprehensive validation for agent requests.

        Args:
            agent_type: Type of agent
            content: Content to process
            parameters: Parameters for the agent
            user_id: User ID making the request

        Returns:
            Dictionary with validation results and sanitized data
        """
        errors = []

        # Validate agent type
        if not cls.validate_agent_type(agent_type):
            errors.append(f"Agent type '{agent_type}' is not allowed")

        # Validate user ID format
        if not cls.validate_user_id_format(user_id):
            errors.append("Invalid user ID format")

        # Validate content if provided
        if content:
            if not cls.validate_content_length(content):
                errors.append(f"Content exceeds maximum length of {cls.MAX_CONTENT_LENGTH} characters")

            if not cls.validate_content_safety(content):
                errors.append("Content contains potentially dangerous patterns")

        # Validate parameters if provided
        if parameters:
            if not cls.validate_parameters_length(parameters):
                errors.append(f"Parameters exceed maximum length of {cls.MAX_PARAMETERS_LENGTH} characters")

        # Sanitize content if validation passed
        sanitized_content = cls.sanitize_content(content) if content else content

        return {
            "is_valid": len(errors) == 0,
            "errors": errors,
            "sanitized_content": sanitized_content
        }


def validate_and_secure_request(agent_type: str, content: Optional[str] = None,
                               parameters: Optional[Dict[str, Any]] = None,
                               user_id: Optional[str] = None,
                               endpoint: str = "/api/agent") -> Dict[str, Any]:
    """
    Convenience function to validate and secure an agent request.

    Args:
        agent_type: Type of agent
        content: Content to process
        parameters: Parameters for the agent
        user_id: User ID making the request
        endpoint: API endpoint being called

    Returns:
        Dictionary with validation results and sanitized data
    """
    # First validate the request
    validation_result = SecurityValidator.validate_agent_request(agent_type, content, parameters, user_id)

    if not validation_result["is_valid"]:
        return validation_result

    # If validation passed, check rate limiting
    user_id_for_limiting = user_id or "anonymous"  # Use "anonymous" for unauthenticated users
    rate_limit_result = check_rate_limit(user_id_for_limiting, endpoint, agent_type)

    if not rate_limit_result['allowed']:
        validation_result["is_valid"] = False
        validation_result["errors"].append("Rate limit exceeded. Please try again later.")
        validation_result["rate_limit_info"] = rate_limit_result

    return validation_result


def check_rate_limit_for_request(user_id: str, endpoint: str, agent_type: str) -> Dict[str, Any]:
    """
    Check rate limit for a request without other validations.

    Args:
        user_id: User ID making the request
        endpoint: API endpoint being called
        agent_type: Type of agent being used

    Returns:
        Dictionary with rate limit status
    """
    return check_rate_limit(user_id, endpoint, agent_type)