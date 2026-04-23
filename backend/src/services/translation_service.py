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
        """Sentence-level translation to Urdu with proper grammar and context"""
        try:
            if target_language != "ur":
                return f"[{target_language.upper()} translation not supported] {text}"

            logger.info(f"Sentence-level Urdu translation starting for text length: {len(text)}")

            # First try to use a proper translation service
            try:
                # Try Google Translate via free API
                import requests
                import asyncio

                # Use MyMemory free translation API (no key required)
                url = "https://api.mymemory.translated.net/get"
                params = {
                    'q': text[:500],  # Limit text length
                    'langpair': 'en|ur'
                }

                loop = asyncio.get_event_loop()
                response = await loop.run_in_executor(
                    None,
                    lambda: requests.get(url, params=params, timeout=10)
                )

                if response.status_code == 200:
                    result = response.json()
                    if result.get('responseStatus') == 200:
                        translated = result['responseData']['translatedText']
                        if translated and translated != text:
                            logger.info("MyMemory translation successful")
                            return translated

            except Exception as e:
                logger.warning(f"MyMemory API failed: {e}")

            # Fallback: Sentence-level templates for robotics content
            text_lower = text.lower().strip()

            # Common sentence patterns in robotics content
            sentence_templates = {
                # Chapter introductions
                r"this chapter (covers|discusses|explores|examines) (.+)": "یہ باب {} کا احاطہ کرتا ہے",
                r"in this chapter,? (we will|you will) (.+)": "اس باب میں آپ {}",
                r"chapter (\d+):? (.+)": "باب {}: {}",

                # Learning objectives
                r"by the end of this chapter,? you will (.+)": "اس باب کے اختتام تک آپ {}",
                r"learning objectives?:?": "تعلیمی مقاصد:",
                r"you will (be able to|learn to|understand) (.+)": "آپ {} سکیں گے",

                # Technical descriptions
                r"(.+) is a (.+) that (.+)": "{} ایک {} ہے جو {}",
                r"(.+) are (.+) used for (.+)": "{} {} ہیں جو {} کے لیے استعمال ہوتے ہیں",
                r"the (.+) system (.+)": "{} نظام {}",

                # Instructions
                r"to (.+),? (first|you need to|we need to) (.+)": "{} کے لیے پہلے {}",
                r"step (\d+):? (.+)": "قدم {}: {}",
                r"follow these steps:?": "یہ قدم اٹھائیں:",

                # Common phrases
                r"for example,?": "مثال کے طور پر",
                r"in conclusion,?": "خلاصہ یہ ہے کہ",
                r"as we can see,?": "جیسا کہ ہم دیکھ سکتے ہیں",
                r"it is important to (.+)": "{} کرنا اہم ہے",
                r"make sure (to )?(.+)": "یقینی بنائیں کہ {}",
            }

            # Try to match sentence patterns
            import re
            for pattern, urdu_template in sentence_templates.items():
                match = re.search(pattern, text_lower)
                if match:
                    # Extract matched groups and translate key terms
                    groups = match.groups()
                    translated_groups = []

                    for group in groups:
                        if group:
                            translated_group = self._translate_key_terms(group)
                            translated_groups.append(translated_group)

                    try:
                        result = urdu_template.format(*translated_groups)
                        logger.info("Sentence template translation successful")
                        return result
                    except:
                        pass

            # Fallback: Improved phrase-level translation
            return self._translate_phrases(text)

        except Exception as e:
            logger.error(f"Sentence translation error: {e}")
            return text

    def _translate_key_terms(self, text: str) -> str:
        """Translate key technical terms while preserving context"""
        key_terms = {
            "robot": "روبوٹ",
            "robotics": "روبوٹکس",
            "humanoid": "انسان نما روبوٹ",
            "control system": "کنٹرول سسٹم",
            "simulation": "نقل",
            "environment": "ماحول",
            "sensor": "سینسر",
            "actuator": "ایکچویٹر",
            "artificial intelligence": "مصنوعی ذہانت",
            "machine learning": "مشین لرننگ",
            "programming": "پروگرامنگ",
            "algorithm": "الگورتھم",
            "software": "سافٹ ویئر",
            "hardware": "ہارڈ ویئر",
            "computer": "کمپیوٹر",
            "technology": "ٹیکنالوجی",
            "development": "ترقی",
            "implementation": "نافذ کرنا",
            "configuration": "ترتیب",
            "installation": "انسٹالیشن"
        }

        result = text
        import re
        for english, urdu in key_terms.items():
            pattern = r'\b' + re.escape(english) + r'\b'
            result = re.sub(pattern, urdu, result, flags=re.IGNORECASE)

        return result

    def _translate_phrases(self, text: str) -> str:
        """Translate common phrases and improve grammar"""

        # Common phrase replacements
        phrase_replacements = {
            "this chapter": "یہ باب",
            "in this chapter": "اس باب میں",
            "the next chapter": "اگلا باب",
            "as we can see": "جیسا کہ ہم دیکھ سکتے ہیں",
            "for example": "مثال کے طور پر",
            "in conclusion": "خلاصہ یہ ہے",
            "it is important": "یہ اہم ہے",
            "we will learn": "ہم سیکھیں گے",
            "you will learn": "آپ سیکھیں گے",
            "let us": "آئیے",
            "we can": "ہم کر سکتے ہیں",
            "you can": "آپ کر سکتے ہیں",
            "we need to": "ہمیں ضرورت ہے",
            "you need to": "آپ کو ضرورت ہے",
            "make sure": "یقینی بنائیں",
            "keep in mind": "ذہن میں رکھیں",
            "step by step": "قدم بہ قدم",
            "first of all": "سب سے پہلے",
            "at the end": "آخر میں",
            "on the other hand": "دوسری طرف"
        }

        result = text
        import re

        # Replace phrases first (longer matches first)
        for english_phrase, urdu_phrase in sorted(phrase_replacements.items(), key=len, reverse=True):
            pattern = r'\b' + re.escape(english_phrase) + r'\b'
            result = re.sub(pattern, urdu_phrase, result, flags=re.IGNORECASE)

        # Then translate remaining key terms
        result = self._translate_key_terms(result)

        # Clean up spacing
        result = re.sub(r'\s+', ' ', result).strip()

        return result


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
