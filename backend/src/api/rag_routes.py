"""
Enhanced RAG API routes with authentication and chat history persistence.
"""
from fastapi import APIRouter, HTTPException, Depends, Header
from pydantic import BaseModel, Field
from typing import List, Optional
from sqlalchemy.orm import Session
import sys
import os
from datetime import datetime
from enum import Enum

# Add backend directory to path
backend_path = os.path.join(os.path.dirname(__file__), '..', '..', '..')
if backend_path not in sys.path:
    sys.path.insert(0, backend_path)

from ..database.database import get_db
from ..models.rag_session import RAGSession, RAGQuery
from ..models.user import User
from .dependencies import get_current_user_optional
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# Error codes for better error handling
class RAGErrorCode(str, Enum):
    RAG_BOT_INIT_FAILED = "rag_bot_init_failed"
    QUERY_PROCESSING_FAILED = "query_processing_failed"
    NO_RELEVANT_CHUNKS = "no_relevant_chunks"
    INVALID_SESSION = "invalid_session"
    UNAUTHORIZED = "unauthorized"
    NOT_FOUND = "not_found"

# Pydantic models
class RAGQueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000, description="Question to ask the RAG system")
    max_chunks: Optional[int] = Field(5, ge=1, le=20, description="Maximum number of chunks to retrieve")
    threshold: Optional[float] = Field(0.3, ge=0.0, le=1.0, description="Similarity threshold for chunk retrieval")
    session_id: Optional[str] = Field(None, description="Session ID for conversation continuity")

class SourceReference(BaseModel):
    url: str
    page_title: str
    section_title: str
    score: float

class RAGResponse(BaseModel):
    answer: str
    sources: List[SourceReference]
    chunks_retrieved: int
    session_id: str
    error_code: Optional[str] = None

class ErrorResponse(BaseModel):
    detail: str
    error_code: str
    status_code: int

class ChatHistoryItem(BaseModel):
    id: str
    query: str
    response: str
    sources: List[dict]
    created_at: str

class ChatHistoryResponse(BaseModel):
    session_id: str
    history: List[ChatHistoryItem]
    total_queries: int

# Initialize RAG bot (lazy loading)
_rag_bot = None

def get_rag_bot():
    """Get or initialize the RAG bot instance."""
    global _rag_bot
    if _rag_bot is None:
        try:
            # Import from the main.py at backend root
            import sys
            import os
            backend_root = os.path.join(os.path.dirname(__file__), '..', '..')
            if backend_root not in sys.path:
                sys.path.insert(0, backend_root)

            from main import RagBot, DEFAULT_CONFIG

            _rag_bot = RagBot(
                qdrant_url=DEFAULT_CONFIG["qdrant_url"],
                cohere_model=DEFAULT_CONFIG["rag_cohere_model"],
                collection_name="docusaurus_embeddings"
            )
            logger.info("RAG bot initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG bot: {str(e)}", exc_info=True)
            raise HTTPException(
                status_code=500,
                detail=f"RAG system initialization failed",
                headers={"X-Error-Code": RAGErrorCode.RAG_BOT_INIT_FAILED}
            )
    return _rag_bot

@router.post("/query", response_model=RAGResponse)
async def rag_query(
    query: RAGQueryRequest,
    db: Session = Depends(get_db),
    current_user: Optional[User] = Depends(get_current_user_optional),
    authorization: Optional[str] = Header(None)
):
    """
    Query the RAG system with authentication support.
    Saves chat history for authenticated users.
    """
    try:
        # Get RAG bot instance
        rag_bot = get_rag_bot()

        # Get or create RAG session
        session = None
        if current_user:
            # Authenticated user - get or create session
            session = db.query(RAGSession).filter(
                RAGSession.user_id == current_user.id
            ).order_by(RAGSession.updated_at.desc()).first()

            if not session or (query.session_id and str(session.id) != query.session_id):
                # Create new session or get specific session
                if query.session_id:
                    session = db.query(RAGSession).filter(
                        RAGSession.id == query.session_id,
                        RAGSession.user_id == current_user.id
                    ).first()

                if not session:
                    session = RAGSession(user_id=current_user.id)
                    db.add(session)
                    db.commit()
                    db.refresh(session)
                    logger.info(f"Created new RAG session for user {current_user.email}")
        else:
            # Anonymous user - create session with token
            if query.session_id:
                session = db.query(RAGSession).filter(
                    RAGSession.id == query.session_id
                ).first()

            if not session:
                import uuid
                session_token = str(uuid.uuid4())
                session = RAGSession(session_token=session_token)
                db.add(session)
                db.commit()
                db.refresh(session)
                logger.info(f"Created new anonymous RAG session")

        # Retrieve relevant chunks
        context_chunks = rag_bot.retrieve_relevant_chunks(
            query.question,
            limit=query.max_chunks,
            threshold=query.threshold
        )

        if not context_chunks:
            response_text = "I couldn't find any relevant information to answer your question. The documentation might not contain the information you're looking for."
            sources = []
            error_code = RAGErrorCode.NO_RELEVANT_CHUNKS
        else:
            # Generate response using the context
            response_text = rag_bot.generate_response(query.question, context_chunks)

            # Format sources
            sources = [
                SourceReference(
                    url=chunk["url"],
                    page_title=chunk["page_title"],
                    section_title=chunk["section_title"],
                    score=chunk["score"]
                )
                for chunk in context_chunks
            ]
            error_code = None

        # Save query to database if session exists
        if session:
            rag_query_record = RAGQuery(
                session_id=session.id,
                query=query.question,
                response=response_text,
                sources=[{
                    "url": s.url,
                    "page_title": s.page_title,
                    "section_title": s.section_title,
                    "score": s.score
                } for s in sources],
                confidence_score=max([s.score for s in sources]) if sources else 0.0
            )
            db.add(rag_query_record)

            # Update session timestamp
            session.updated_at = datetime.utcnow()
            db.add(session)

            db.commit()
            logger.info(f"Saved RAG query to session {session.id}")

        return RAGResponse(
            answer=response_text,
            sources=sources,
            chunks_retrieved=len(context_chunks),
            session_id=str(session.id) if session else "",
            error_code=error_code
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing RAG query: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to process query",
            headers={"X-Error-Code": RAGErrorCode.QUERY_PROCESSING_FAILED}
        )

@router.get("/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    session_id: Optional[str] = None,
    limit: int = 50,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user_optional)
):
    """
    Get chat history for the current user or session.
    Requires authentication for user-scoped history.
    """
    try:
        if not current_user and not session_id:
            raise HTTPException(
                status_code=401,
                detail="Authentication required to view chat history",
                headers={"X-Error-Code": RAGErrorCode.UNAUTHORIZED}
            )

        # Get session
        if session_id:
            session = db.query(RAGSession).filter(
                RAGSession.id == session_id
            ).first()

            # Verify ownership for authenticated users
            if current_user and session and session.user_id != current_user.id:
                raise HTTPException(
                    status_code=403,
                    detail="Access denied to this chat session",
                    headers={"X-Error-Code": RAGErrorCode.UNAUTHORIZED}
                )
        else:
            # Get most recent session for user
            session = db.query(RAGSession).filter(
                RAGSession.user_id == current_user.id
            ).order_by(RAGSession.updated_at.desc()).first()

        if not session:
            return ChatHistoryResponse(
                session_id="",
                history=[],
                total_queries=0
            )

        # Get queries for this session
        queries = db.query(RAGQuery).filter(
            RAGQuery.session_id == session.id
        ).order_by(RAGQuery.created_at.desc()).limit(limit).all()

        history = [
            ChatHistoryItem(
                id=str(q.id),
                query=q.query,
                response=q.response,
                sources=q.sources or [],
                created_at=q.created_at.isoformat()
            )
            for q in reversed(queries)  # Reverse to show oldest first
        ]

        return ChatHistoryResponse(
            session_id=str(session.id),
            history=history,
            total_queries=len(queries)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving chat history: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve chat history",
            headers={"X-Error-Code": RAGErrorCode.QUERY_PROCESSING_FAILED}
        )

@router.delete("/history/{session_id}")
async def delete_chat_history(
    session_id: str,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user_optional)
):
    """
    Delete a chat session and all its queries.
    Requires authentication and ownership.
    """
    try:
        if not current_user:
            raise HTTPException(
                status_code=401,
                detail="Authentication required",
                headers={"X-Error-Code": RAGErrorCode.UNAUTHORIZED}
            )

        # Get session and verify ownership
        session = db.query(RAGSession).filter(
            RAGSession.id == session_id,
            RAGSession.user_id == current_user.id
        ).first()

        if not session:
            raise HTTPException(
                status_code=404,
                detail="Chat session not found",
                headers={"X-Error-Code": RAGErrorCode.NOT_FOUND}
            )

        # Delete all queries in this session
        db.query(RAGQuery).filter(RAGQuery.session_id == session.id).delete()

        # Delete the session
        db.delete(session)
        db.commit()

        logger.info(f"Deleted chat session {session_id} for user {current_user.email}")

        return {"message": "Chat history deleted successfully"}

    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        logger.error(f"Error deleting chat history: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to delete chat history",
            headers={"X-Error-Code": RAGErrorCode.QUERY_PROCESSING_FAILED}
        )

@router.get("/health")
async def rag_health():
    """
    Check if the RAG system is healthy and ready to serve requests.
    """
    try:
        rag_bot = get_rag_bot()
        return {
            "status": "healthy",
            "collection": "docusaurus_embeddings",
            "ready": True
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e),
            "ready": False
        }
