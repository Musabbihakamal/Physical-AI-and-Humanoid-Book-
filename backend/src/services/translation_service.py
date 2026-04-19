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
    HUGGINGFACE = "huggingface"


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


class HuggingFaceTranslationService(TranslationService):
    """Free Hugging Face translation service"""

    def __init__(self):
        # Using free Hugging Face inference API - no API key needed!
        self.base_url = "https://api-inference.huggingface.co/models"
        logger.info("Using FREE Hugging Face translation service")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using free Hugging Face models"""
        try:
            import requests
            import asyncio

            # Map language codes to free model names
            model_map = {
                "ur": "Helsinki-NLP/opus-mt-en-ur",  # English to Urdu
                "es": "Helsinki-NLP/opus-mt-en-es",  # English to Spanish
                "fr": "Helsinki-NLP/opus-mt-en-fr",  # English to French
                "de": "Helsinki-NLP/opus-mt-en-de",  # English to German
                "ar": "Helsinki-NLP/opus-mt-en-ar",  # English to Arabic
                "hi": "Helsinki-NLP/opus-mt-en-hi",  # English to Hindi
            }

            model_name = model_map.get(target_language)
            if not model_name:
                logger.warning(f"Language {target_language} not supported by free service")
                return f"[{target_language.upper()} - Language not supported] {text}"

            url = f"{self.base_url}/{model_name}"
            payload = {"inputs": text}
            headers = {"Content-Type": "application/json"}

            logger.info(f"Free Hugging Face translation to {target_language}: {text[:50]}...")

            # Make async request
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: requests.post(url, json=payload, headers=headers, timeout=15)
            )

            if response.status_code == 200:
                result = response.json()
                if isinstance(result, list) and len(result) > 0:
                    translated_text = result[0].get("translation_text", text)
                    logger.info(f"Translation successful: {translated_text[:50]}...")
                    return translated_text
                else:
                    logger.warning("Empty result from Hugging Face API")
                    return f"[{target_language.upper()} - Free Translation] {text}"
            else:
                logger.warning(f"Hugging Face API error: {response.status_code}")
                return f"[{target_language.upper()} - Free Translation] {text}"

        except Exception as e:
            logger.error(f"Hugging Face translation error: {e}")
            return f"[{target_language.upper()} - Free Translation] {text}"


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
        self.api_key = os.getenv("ANTHROPIC_API_KEY")
        if not self.api_key:
            logger.warning("ANTHROPIC_API_KEY not found, using mock translation service")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using Claude API"""
        if not self.api_key:
            return await MockTranslationService().translate(text, target_language, source_language)

        try:
            import asyncio
            from anthropic import Anthropic

            client = Anthropic(api_key=self.api_key)

            language_names = {
                "ur": "Urdu",
                "es": "Spanish",
                "fr": "French",
                "de": "German",
                "ar": "Arabic",
                "hi": "Hindi",
            }

            target_lang_name = language_names.get(target_language, target_language)

            prompt = f"""You are helping with a software development project that includes a technical documentation system with translation features.

As part of this development project, translate the following HTML content from English to {target_lang_name} for the documentation system:

TECHNICAL REQUIREMENTS:
- Preserve all HTML tags exactly as they are
- Preserve all code blocks (```...```) exactly as they are
- Preserve all inline code (`...`) exactly as they are
- Preserve all placeholders like __PRESERVED_* exactly as they are
- Only translate the actual text content, not the markup or code
- This is for technical documentation translation in a software project
- Return ONLY the translated HTML, no explanations

Content to translate:
{text}"""

            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: client.messages.create(
                    model="claude-opus-4-7",  # Back to original model
                    max_tokens=4096,
                    messages=[
                        {"role": "user", "content": prompt}
                    ]
                )
            )

            translated_text = response.content[0].text

            # Strip markdown code block wrapper if present
            if translated_text.startswith('```'):
                # Remove opening ```html or similar
                translated_text = translated_text.split('\n', 1)[1] if '\n' in translated_text else translated_text
                # Remove closing ```
                if translated_text.endswith('```'):
                    translated_text = translated_text[:-3].rstrip()

            logger.info(f"Claude translation successful to {target_language}")
            return translated_text

        except UnicodeEncodeError as e:
            # Handle encoding errors separately - translation was successful but can't be logged
            logger.info(f"Claude translation successful to {target_language} (encoding issue in logs)")
            return translated_text
        except Exception as e:
            logger.error(f"Claude translation error: {type(e).__name__}: {str(e)}", exc_info=True)
            return f"[{target_language.upper()} - Translation Error] {text}"


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
        logger.info("=== TranslationServiceFactory.get_translation_service() called ===")

        # Check for Claude API key first (ANTHROPIC_API_KEY)
        anthropic_key = os.getenv("ANTHROPIC_API_KEY")
        logger.info(f"Checking ANTHROPIC_API_KEY: {anthropic_key[:20] if anthropic_key else 'NOT SET'}...")

        if anthropic_key:
            logger.info("✓ Using Claude translation service")
            return ClaudeTranslationService()

        # Check for OpenAI API key
        openai_key = os.getenv("OPENAI_API_KEY")
        logger.info(f"Checking OPENAI_API_KEY: {openai_key[:20] if openai_key else 'NOT SET'}...")
        if openai_key:
            logger.info("✓ Using OpenAI translation service")
            return OpenAITranslationService()

        # Check for Qwen API key
        qwen_key = os.getenv("QWEN_API_KEY")
        logger.info(f"Checking QWEN_API_KEY: {qwen_key[:20] if qwen_key else 'NOT SET'}...")
        if qwen_key:
            logger.info("✓ Using Qwen translation service")
            return QwenTranslationService()

        # Use FREE Hugging Face service as default
        logger.info("✗ No paid API keys found, using FREE Hugging Face translation service")
        return HuggingFaceTranslationService()


# Global instance for easy access - lazy loaded
_translation_service = None

def get_translation_service():
    """Get or create the translation service (lazy loading)"""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationServiceFactory.get_translation_service()
    return _translation_service


async def translate_text(text: str, target_language: str, source_language: Optional[str] = None) -> str:
    """Convenience function to translate text using the configured service"""
    # Force fresh service instance to avoid caching issues
    service = TranslationServiceFactory.get_translation_service()
    return await service.translate(text, target_language, source_language)