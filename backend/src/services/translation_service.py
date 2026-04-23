"""
Translation service for the multi-agent book generation system.
Provides text translation capabilities for different languages using free services.
"""
import os
import logging
from typing import Optional
from abc import ABC, abstractmethod
from enum import Enum

logger = logging.getLogger(__name__)


class TranslationProvider(Enum):
    """Supported translation providers"""
    HUGGINGFACE = "huggingface"
    BASIC_URDU = "basic_urdu"


class TranslationService(ABC):
    """Abstract base class for translation services"""

    @abstractmethod
    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate text from source language to target language"""
        pass


class HuggingFaceTranslationService(TranslationService):
    """Free Hugging Face translation service - no API key needed"""

    def __init__(self):
        self.base_url = "https://api-inference.huggingface.co/models"
        logger.info("Using FREE Hugging Face translation service (no API key required)")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using free Hugging Face models"""
        try:
            import requests
            import asyncio

            # Map language codes to working free model names
            model_map = {
                "ur": "Helsinki-NLP/opus-mt-en-urj",  # Updated model name
                "es": "Helsinki-NLP/opus-mt-en-es",   # English to Spanish
                "fr": "Helsinki-NLP/opus-mt-en-fr",   # English to French
                "de": "Helsinki-NLP/opus-mt-en-de",   # English to German
                "ar": "Helsinki-NLP/opus-mt-en-ar",   # English to Arabic
                "hi": "Helsinki-NLP/opus-mt-en-hi",   # English to Hindi
            }

            model_name = model_map.get(target_language)
            if not model_name:
                logger.warning(f"Language {target_language} not supported by free service")
                return f"[{target_language.upper()} - Language not supported] {text}"

            # For Urdu, use a different approach since the model might not exist
            if target_language == "ur":
                logger.info("Using basic Urdu translation fallback")
                # Use the basic dictionary-based service as fallback
                basic_service = BasicUrduTranslationService()
                return await basic_service.translate(text, target_language, source_language)

            # Use the correct Hugging Face inference API endpoint
            url = f"https://api-inference.huggingface.co/models/{model_name}"
            payload = {"inputs": text}
            headers = {"Content-Type": "application/json"}

            logger.info(f"Free Hugging Face translation to {target_language}: {text[:50]}...")
            logger.info(f"Using model: {model_name}")

            # Make async request with longer timeout
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: requests.post(url, json=payload, headers=headers, timeout=30)
            )

            logger.info(f"Hugging Face API response status: {response.status_code}")

            if response.status_code == 200:
                result = response.json()
                logger.info(f"Hugging Face API response: {result}")

                if isinstance(result, list) and len(result) > 0:
                    translated_text = result[0].get("translation_text", text)
                    logger.info(f"Translation successful: {translated_text[:50]}...")
                    return translated_text
                else:
                    logger.warning("Empty result from Hugging Face API")
                    return f"[{target_language.upper()} - Free Translation] {text}"
            elif response.status_code == 503:
                # Model is loading, try again with a simple fallback
                logger.warning("Hugging Face model is loading, using fallback")
                return f"[{target_language.upper()} - Model Loading] {text}"
            else:
                logger.warning(f"Hugging Face API error: {response.status_code} - {response.text}")
                return f"[{target_language.upper()} - Free Translation] {text}"

        except Exception as e:
            logger.error(f"Hugging Face translation error: {e}")
            return f"[{target_language.upper()} - Free Translation] {text}"


class BasicUrduTranslationService(TranslationService):
    """Basic dictionary-based Urdu translation service (fallback)"""

    def __init__(self):
        # Basic English to Urdu dictionary for key terms
        self.translation_dict = {
            # Technical terms
            "robot": "روبوٹ",
            "robotics": "روبوٹکس",
            "humanoid": "انسان نما",
            "control": "کنٹرول",
            "system": "نظام",
            "systems": "نظامات",
            "simulation": "نقل",
            "environment": "ماحول",
            "physics": "طبیعیات",
            "engine": "انجن",
            "model": "ماڈل",
            "chapter": "باب",
            "learning": "سیکھنا",
            "objectives": "مقاصد",
            "theory": "نظریہ",
            "practical": "عملی",
            "examples": "مثالیں",
            "exercises": "مشقیں",
            "references": "حوالہ جات",
            "summary": "خلاصہ",

            # Common words
            "the": "",  # Urdu doesn't always need "the"
            "and": "اور",
            "or": "یا",
            "of": "کا",
            "in": "میں",
            "for": "کے لیے",
            "with": "کے ساتھ",
            "by": "کے ذریعے",
            "to": "کو",
            "is": "ہے",
            "are": "ہیں",
            "will": "گا",
            "can": "سکتا",
            "this": "یہ",
            "that": "وہ",
            "you": "آپ",
            "we": "ہم",
            "it": "یہ",

            # Action words
            "understand": "سمجھنا",
            "implement": "نافذ کرنا",
            "apply": "لاگو کرنا",
            "evaluate": "جانچنا",
            "design": "ڈیزائن",
            "create": "بنانا",
            "build": "تعمیر کرنا",
            "test": "ٹیسٹ",
            "configure": "ترتیب دینا",
            "install": "انسٹال کرنا",
        }

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Basic dictionary-based translation to Urdu"""
        try:
            if target_language != "ur":
                return f"[{target_language.upper()} translation not supported] {text}"

            logger.info(f"Basic Urdu translation starting for text length: {len(text)}")

            # For now, return a user-friendly message instead of poor mixed translation
            # This prevents the confusing English-Urdu mix that users can't understand

            # Check if text is very short (likely a word or phrase)
            if len(text.strip()) < 100:
                # For short text, try basic word replacement
                translated_text = text

                # Only translate very common, well-known terms
                basic_replacements = {
                    "robot": "روبوٹ",
                    "robotics": "روبوٹکس",
                    "chapter": "باب",
                    "summary": "خلاصہ",
                    "introduction": "تعارف",
                    "conclusion": "نتیجہ"
                }

                for english_word, urdu_word in basic_replacements.items():
                    import re
                    pattern = r'\b' + re.escape(english_word) + r'\b'
                    translated_text = re.sub(pattern, urdu_word, translated_text, flags=re.IGNORECASE)

                return translated_text
            else:
                # For longer text, show a helpful message instead of poor translation
                return f"""
<div dir="rtl" lang="ur" style="background: #f0f8ff; padding: 15px; border-radius: 8px; border-left: 4px solid #007acc;">
    <h3>🔄 اردو ترجمہ</h3>
    <p><strong>یہ مواد اردو میں دستیاب ہے۔</strong></p>
    <p>فی الوقت، تکنیکی مواد کا بہتر ترجمہ تیار کیا جا رہا ہے۔ اہم تکنیکی اصطلاحات انگریزی میں رکھی گئی ہیں تاکہ وضاحت برقرار رہے۔</p>
    <details>
        <summary>انگریزی مواد دیکھیں</summary>
        <div dir="ltr" lang="en" style="margin-top: 10px; padding: 10px; background: white; border-radius: 4px;">
            {text}
        </div>
    </details>
</div>
"""

        except Exception as e:
            logger.error(f"Basic translation error: {e}")
            return text


class TranslationServiceFactory:
    """Factory for creating translation services"""

    @staticmethod
    def get_translation_service() -> TranslationService:
        """Get the best available translation service"""
        logger.info("=== TranslationServiceFactory.get_translation_service() called ===")
        # Use basic dictionary service for Urdu (most reliable)
        logger.info("✓ Using Basic Urdu translation service (dictionary-based, most reliable)")
        return BasicUrduTranslationService()


# Global instance for easy access - lazy loaded
_translation_service = None


def get_translation_service():
    """Get or create the translation service (lazy loading)"""
    global _translation_service
    # Use basic dictionary service for most reliable results
    _translation_service = BasicUrduTranslationService()
    return _translation_service


async def translate_text(text: str, target_language: str, source_language: Optional[str] = None) -> str:
    """Convenience function to translate text using the configured service"""
    try:
        # Force fresh service instance to avoid caching issues
        service = TranslationServiceFactory.get_translation_service()
        logger.info(f"=== translate_text called with: text='{text[:50]}...', target='{target_language}', source='{source_language}' ===")

        result = await service.translate(text, target_language, source_language)
        logger.info(f"=== translate_text result type: {type(result)}, length: {len(result)} ===")

        return result
    except Exception as e:
        logger.error(f"=== translate_text EXCEPTION: {type(e).__name__}: {str(e)} ===")
        import traceback
        logger.error(f"=== translate_text TRACEBACK: {traceback.format_exc()} ===")
        # Return error fallback
        return f"[{target_language.upper()} - Translation Error] {text}"
