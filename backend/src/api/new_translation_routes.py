"""
Brand new translation routes to bypass caching issues
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import os
import asyncio
from dotenv import load_dotenv
from anthropic import Anthropic

router = APIRouter()

class TranslationRequest(BaseModel):
    text: str
    target_language: str
    source_language: Optional[str] = "en"

class TranslationResponse(BaseModel):
    translated_text: str
    source_language: str
    target_language: str

@router.post("/translate-new", response_model=TranslationResponse)
async def translate_new(request: TranslationRequest):
    """
    Brand new translation endpoint to bypass all caching issues
    """
    print("🚀 NEW TRANSLATION ENDPOINT HIT!")
    print(f"Text: {request.text[:50]}...")
    print(f"Target: {request.target_language}")

    try:
        # Load environment
        load_dotenv('.env', override=True)
        api_key = os.getenv("ANTHROPIC_API_KEY")

        print(f"API Key present: {bool(api_key)}")

        if not api_key:
            return TranslationResponse(
                translated_text="[ERROR] No API key found",
                source_language=request.source_language or "en",
                target_language=request.target_language
            )

        # Create client
        client = Anthropic(api_key=api_key)

        language_names = {
            "ur": "Urdu",
            "es": "Spanish",
            "fr": "French",
            "de": "German",
            "ar": "Arabic",
            "hi": "Hindi",
        }

        target_lang_name = language_names.get(request.target_language, request.target_language)
        print(f"Translating to: {target_lang_name}")

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
{request.text}"""

        print("Calling Claude API...")
        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(
            None,
            lambda: client.messages.create(
                model="claude-opus-4-7",
                max_tokens=4096,
                messages=[
                    {"role": "user", "content": prompt}
                ]
            )
        )

        translated_text = response.content[0].text
        print(f"✅ Translation successful! Length: {len(translated_text)}")

        return TranslationResponse(
            translated_text=translated_text,
            source_language=request.source_language or "en",
            target_language=request.target_language
        )

    except Exception as e:
        print(f"❌ Translation error: {type(e).__name__}: {e}")
        return TranslationResponse(
            translated_text=f"[ERROR] {str(e)}",
            source_language=request.source_language or "en",
            target_language=request.target_language
        )

@router.get("/test-new")
async def test_new():
    """Test endpoint to verify new routes are working"""
    return {"message": "New translation routes are working!", "status": "success"}