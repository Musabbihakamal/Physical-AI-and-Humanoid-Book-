"""
Translation API routes for the book generation system.
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from ..services.translation_service import translate_text
from ..api.dependencies import get_current_user
from ..models.user import User

router = APIRouter()


class TranslationRequest(BaseModel):
    text: str
    target_language: str
    source_language: Optional[str] = "en"


class TranslationResponse(BaseModel):
    translated_text: str
    source_language: str
    target_language: str


@router.post("/translate", response_model=TranslationResponse)
async def translate_content(
    request: TranslationRequest,
    current_user: User = Depends(get_current_user)
):
    """
    Translate content from source language to target language.
    """
    try:
        # Validate input
        if not request.text or len(request.text.strip()) == 0:
            raise HTTPException(status_code=400, detail="Text to translate cannot be empty")

        if not request.target_language:
            raise HTTPException(status_code=400, detail="Target language is required")

        # Perform translation using the service
        translated_text = await translate_text(
            text=request.text,
            target_language=request.target_language,
            source_language=request.source_language
        )

        return TranslationResponse(
            translated_text=translated_text,
            source_language=request.source_language or "en",
            target_language=request.target_language
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@router.get("/health")
async def translation_health():
    """
    Health check for the translation service.
    """
    return {"status": "healthy", "service": "translation"}