"""
RAG API routes for the multi-agent book generation system.
"""
from fastapi import APIRouter

router = APIRouter()

@router.get("/query")
async def rag_query():
    """
    Query the RAG system.
    """
    return {"message": "RAG query endpoint"}