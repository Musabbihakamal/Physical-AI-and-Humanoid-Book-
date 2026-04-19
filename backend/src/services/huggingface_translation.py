"""
Free Hugging Face translation service
"""
import requests
import asyncio
from typing import Optional
from .translation_service import TranslationService

class HuggingFaceTranslationService(TranslationService):
    """Free Hugging Face translation service"""

    def __init__(self):
        # Using free Hugging Face inference API
        self.base_url = "https://api-inference.huggingface.co/models"
        # No API key needed for basic usage

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using free Hugging Face models"""
        try:
            # Map language codes to model names
            model_map = {
                "ur": "Helsinki-NLP/opus-mt-en-ur",  # English to Urdu
                "es": "Helsinki-NLP/opus-mt-en-es",  # English to Spanish
                "fr": "Helsinki-NLP/opus-mt-en-fr",  # English to French
                "de": "Helsinki-NLP/opus-mt-en-de",  # English to German
                "ar": "Helsinki-NLP/opus-mt-en-ar",  # English to Arabic
            }

            model_name = model_map.get(target_language)
            if not model_name:
                return f"[{target_language.upper()} - Language not supported] {text}"

            url = f"{self.base_url}/{model_name}"

            payload = {"inputs": text}
            headers = {"Content-Type": "application/json"}

            # Make async request
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: requests.post(url, json=payload, headers=headers, timeout=10)
            )

            if response.status_code == 200:
                result = response.json()
                if isinstance(result, list) and len(result) > 0:
                    translated_text = result[0].get("translation_text", text)
                    return translated_text
                else:
                    return f"[{target_language.upper()} - Free Translation] {text}"
            else:
                # Fallback to mock if API fails
                return f"[{target_language.upper()} - Free Translation] {text}"

        except Exception as e:
            # Fallback to mock translation
            return f"[{target_language.upper()} - Free Translation] {text}"