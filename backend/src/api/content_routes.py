"""
Content API routes for the multi-agent book generation system.
"""
from fastapi import APIRouter

router = APIRouter()

@router.get("/")
async def content_root():
    """
    Content management root endpoint.
    """
    return {"message": "Content management endpoints"}