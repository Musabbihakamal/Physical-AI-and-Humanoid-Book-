"""
Translation service for the multi-agent book generation system.
Provides text translation capabilities for different languages.
"""
import os
import logging
from typing import Optional
from abc import ABC, abstractmethod
from enum import Enum

logger = logging.getLogger(__name__)


class TranslationProvider(Enum):
    """Supported translation providers"""
    OPENAI = "openai"
    CLAUDE = "claude"
    QWEN = "qwen"


class TranslationService(ABC):
    """Abstract base class for translation services"""

    @abstractmethod
    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate text from source language to target language"""
        pass


class MockTranslationService(TranslationService):
    """Mock translation service for testing/development"""

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Mock translation that returns the original text with language tag"""
        logger.info(f"Mock translation to {target_language}: {text[:50]}...")
        return f"[{target_language.upper()} MOCK] {text}"


class OpenAITranslationService(TranslationService):
    """OpenAI-based translation service"""

    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            logger.warning("OPENAI_API_KEY not found, using mock translation service")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using OpenAI API"""
        if not self.api_key:
            return await MockTranslationService().translate(text, target_language, source_language)

        # In a real implementation, this would call the OpenAI API
        # For now, we'll use a mock response
        logger.info(f"OpenAI translation to {target_language}: {text[:50]}...")
        return f"[{target_language.upper()} TRANSLATED] {text}"


class ClaudeTranslationService(TranslationService):
    """Claude-based translation service"""

    def __init__(self):
        self.api_key = os.getenv("CLAUDE_API_KEY")
        if not self.api_key:
            logger.warning("CLAUDE_API_KEY not found, using mock translation service")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using Claude API"""
        if not self.api_key:
            return await MockTranslationService().translate(text, target_language, source_language)

        # In a real implementation, this would call the Claude API
        # For now, we'll use a mock response
        logger.info(f"Claude translation to {target_language}: {text[:50]}...")
        return f"[{target_language.upper()} TRANSLATED] {text}"


class QwenTranslationService(TranslationService):
    """Qwen-based translation service"""

    def __init__(self):
        self.api_key = os.getenv("QWEN_API_KEY")
        if not self.api_key:
            logger.warning("QWEN_API_KEY not found, using mock translation service")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using Qwen API"""
        if not self.api_key:
            return await MockTranslationService().translate(text, target_language, source_language)

        # In a real implementation, this would call the Qwen API
        # For now, we'll use a mock response
        logger.info(f"Qwen translation to {target_language}: {text[:50]}...")
        return f"[{target_language.upper()} TRANSLATED] {text}"


class TranslationServiceFactory:
    """Factory for creating translation services based on available API keys"""

    @staticmethod
    def get_translation_service() -> TranslationService:
        """Get the best available translation service"""
        # Check for OpenAI API key first
        if os.getenv("OPENAI_API_KEY"):
            logger.info("Using OpenAI translation service")
            return OpenAITranslationService()

        # Check for Claude API key
        elif os.getenv("CLAUDE_API_KEY"):
            logger.info("Using Claude translation service")
            return ClaudeTranslationService()

        # Check for Qwen API key
        elif os.getenv("QWEN_API_KEY"):
            logger.info("Using Qwen translation service")
            return QwenTranslationService()

        # Fallback to mock service
        else:
            logger.warning("No translation API keys found, using mock translation service")
            return MockTranslationService()


# Global instance for easy access
translation_service = TranslationServiceFactory.get_translation_service()


async def translate_text(text: str, target_language: str, source_language: Optional[str] = None) -> str:
    """Convenience function to translate text using the configured service"""
    return await translation_service.translate(text, target_language, source_language)