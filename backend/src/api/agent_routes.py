"""
Agent API routes for the multi-agent book generation system.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
from ..database.database import get_db
from ..models.agent_request import AgentRequest
from ..models.generated_content import GeneratedContent
from ..models.user import User
from ..services.user_service import UserService
from ..services.glossary_service import GlossaryService
from ..services.code_explainer_service import CodeExplainerService
from ..services.quiz_service import QuizService
from ..utils.errors import ValidationError
from ..utils.security import validate_and_secure_request
from ..utils.rate_limiting import check_rate_limit, record_agent_request, get_user_agent_usage
from ..utils.user_session_manager import UserSessionManager, get_user_agent_history, get_user_agent_statistics
from .auth_dependencies import get_current_active_user
from pydantic import BaseModel
import uuid
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# Pydantic models for request/response validation
class AgentRequestPayload(BaseModel):
    agent_type: str
    parameters: Dict[str, Any]
    context: Optional[Dict[str, Any]] = None


class GlossaryMakerRequest(BaseModel):
    content: str
    parameters: Optional[Dict[str, Any]] = None


class CodeExplainerRequest(BaseModel):
    code: str
    language: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None


class QuizCreatorRequest(BaseModel):
    content: str
    difficulty: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None




@router.post("/glossary-maker")
async def create_glossary(
    request: GlossaryMakerRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """
    Generate a glossary from provided content using the Glossary Maker agent.
    """
    try:
        # Update session activity
        UserSessionManager.update_session_activity(str(current_user.id))

        # Check rate limit for this user and agent type
        if not check_rate_limit(db, str(current_user.id), "GLOSSARY_MAKER"):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "message": "Rate limit exceeded for Glossary Maker",
                    "usage": get_user_agent_usage(db, str(current_user.id), "GLOSSARY_MAKER")
                }
            )

        # Validate and secure the request
        security_result = validate_and_secure_request(
            agent_type="GLOSSARY_MAKER",
            content=request.content,
            parameters=request.parameters or {},
            user_id=current_user.id,
            endpoint="/api/agents/glossary-maker"
        )

        if not security_result["is_valid"]:
            logger.warning(f"Security validation failed for glossary maker: {security_result['errors']}")
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={
                    "message": "Request validation failed",
                    "errors": security_result["errors"]
                }
            )

        # Get sanitized content
        sanitized_content = security_result.get("sanitized_content", request.content)

        # Get user profile if user exists (using the UserProfile model which is linked to User)
        user_profile = UserService.get_user_profile(db, current_user.id)

        # Create agent request
        agent_request = AgentRequest(
            agent_type="GLOSSARY_MAKER",
            parameters=request.parameters or {},
            context={
                "content": sanitized_content,
                "user_profile_id": str(user_profile.user_id) if user_profile else None
            },
            user_id=current_user.id
        )
        db.add(agent_request)
        db.commit()
        db.refresh(agent_request)

        # Execute the Glossary Maker agent processing
        result = await GlossaryService.generate_glossary(
            db=db,
            content=sanitized_content,
            request_id=str(agent_request.id),
            user_profile=user_profile
        )

        # Update the agent request status to COMPLETED
        agent_request.status = "COMPLETED"
        db.commit()

        # Record the successful request for rate limiting
        record_agent_request(db, str(current_user.id), "GLOSSARY_MAKER")

        # Add to user session history
        UserSessionManager.add_to_history(str(current_user.id), str(agent_request.id), "GLOSSARY_MAKER", result)

        result["id"] = str(agent_request.id)
        result["request_id"] = str(agent_request.id)
        result["status"] = "COMPLETED"

        logger.info(f"Glossary maker request created: {agent_request.id} for user: {current_user.id}")
        return result

    except ValidationError as e:
        logger.warning(f"Validation error in glossary maker: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error in glossary maker: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during glossary generation"
        )


@router.post("/code-explainer")
async def explain_code(
    request: CodeExplainerRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """
    Explain code using the Code Explainer agent.
    """
    try:
        # Update session activity
        UserSessionManager.update_session_activity(str(current_user.id))

        # Check rate limit for this user and agent type
        if not check_rate_limit(db, str(current_user.id), "CODE_EXPLAINER"):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "message": "Rate limit exceeded for Code Explainer",
                    "usage": get_user_agent_usage(db, str(current_user.id), "CODE_EXPLAINER")
                }
            )

        # Validate and secure the request
        security_result = validate_and_secure_request(
            agent_type="CODE_EXPLAINER",
            content=request.code,
            parameters={
                "language": request.language,
                **(request.parameters or {})
            },
            user_id=current_user.id,
            endpoint="/api/agents/code-explainer"
        )

        if not security_result["is_valid"]:
            logger.warning(f"Security validation failed for code explainer: {security_result['errors']}")
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={
                    "message": "Request validation failed",
                    "errors": security_result["errors"]
                }
            )

        # Get sanitized content
        sanitized_code = security_result.get("sanitized_content", request.code)

        # Get user profile if user exists (using the UserProfile model which is linked to User)
        user_profile = UserService.get_user_profile(db, current_user.id)

        # Create agent request
        agent_request = AgentRequest(
            agent_type="CODE_EXPLAINER",
            parameters={
                "language": request.language,
                **(request.parameters or {})
            },
            context={
                "code": sanitized_code,
                "user_profile_id": str(user_profile.user_id) if user_profile else None
            },
            user_id=current_user.id
        )
        db.add(agent_request)
        db.commit()
        db.refresh(agent_request)

        # Execute the Code Explainer agent processing
        result = await CodeExplainerService.explain_code(
            db=db,
            code=sanitized_code,
            language=request.language,
            request_id=str(agent_request.id),
            user_profile=user_profile
        )

        # Update the agent request status to COMPLETED
        agent_request.status = "COMPLETED"
        db.commit()

        # Record the successful request for rate limiting
        record_agent_request(db, str(current_user.id), "CODE_EXPLAINER")

        # Add to user session history
        UserSessionManager.add_to_history(str(current_user.id), str(agent_request.id), "CODE_EXPLAINER", result)

        result["id"] = str(agent_request.id)
        result["request_id"] = str(agent_request.id)
        result["status"] = "COMPLETED"

        logger.info(f"Code explainer request created: {agent_request.id} for user: {current_user.id}")
        return result

    except ValidationError as e:
        logger.warning(f"Validation error in code explainer: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error in code explainer: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during code explanation"
        )


@router.post("/quiz-creator")
async def create_quiz(
    request: QuizCreatorRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """
    Generate a quiz from provided content using the Quiz Creator agent.
    """
    try:
        # Update session activity
        UserSessionManager.update_session_activity(str(current_user.id))

        # Check rate limit for this user and agent type
        if not check_rate_limit(db, str(current_user.id), "QUIZ_CREATOR"):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "message": "Rate limit exceeded for Quiz Creator",
                    "usage": get_user_agent_usage(db, str(current_user.id), "QUIZ_CREATOR")
                }
            )

        # Validate and secure the request
        security_result = validate_and_secure_request(
            agent_type="QUIZ_CREATOR",
            content=request.content,
            parameters={
                "difficulty": request.difficulty,
                **(request.parameters or {})
            },
            user_id=current_user.id,
            endpoint="/api/agents/quiz-creator"
        )

        if not security_result["is_valid"]:
            logger.warning(f"Security validation failed for quiz creator: {security_result['errors']}")
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={
                    "message": "Request validation failed",
                    "errors": security_result["errors"]
                }
            )

        # Get sanitized content
        sanitized_content = security_result.get("sanitized_content", request.content)

        # Get user profile if user exists (using the UserProfile model which is linked to User)
        user_profile = UserService.get_user_profile(db, current_user.id)

        # Create agent request
        agent_request = AgentRequest(
            agent_type="QUIZ_CREATOR",
            parameters={
                "difficulty": request.difficulty,
                **(request.parameters or {})
            },
            context={
                "content": sanitized_content,
                "user_profile_id": str(user_profile.user_id) if user_profile else None
            },
            user_id=current_user.id
        )
        db.add(agent_request)
        db.commit()
        db.refresh(agent_request)

        # Execute the Quiz Creator agent processing
        result = await QuizService.generate_quiz(
            db=db,
            content=sanitized_content,
            request_id=str(agent_request.id),
            user_profile=user_profile,
            difficulty=request.difficulty
        )

        # Update the agent request status to COMPLETED
        agent_request.status = "COMPLETED"
        db.commit()

        # Record the successful request for rate limiting
        record_agent_request(db, str(current_user.id), "QUIZ_CREATOR")

        # Add to user session history
        UserSessionManager.add_to_history(str(current_user.id), str(agent_request.id), "QUIZ_CREATOR", result)

        result["id"] = str(agent_request.id)
        result["request_id"] = str(agent_request.id)
        result["status"] = "COMPLETED"

        logger.info(f"Quiz creator request created: {agent_request.id} for user: {current_user.id}")
        return result

    except ValidationError as e:
        logger.warning(f"Validation error in quiz creator: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error in quiz creator: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during quiz generation"
        )




@router.get("/status/{request_id}")
async def get_request_status(
    request_id: str,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """
    Get the status of an agent request.
    """
    try:
        # Validate UUID format
        try:
            uuid.UUID(request_id)
        except ValueError:
            raise ValidationError("Invalid request ID format", "request_id")

        # Get the agent request
        agent_request = db.query(AgentRequest).filter(AgentRequest.id == request_id).first()
        if not agent_request:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Request not found"
            )

        # Check if the current user is authorized to access this request
        if agent_request.user_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to access this request status"
            )

        # Check if there are generated contents associated with this request
        from ..models.generated_content import GeneratedContent
        generated_content = db.query(GeneratedContent).filter(
            GeneratedContent.request_id == agent_request.id
        ).first()

        result = {
            "id": str(agent_request.id),
            "agent_type": agent_request.agent_type,
            "status": agent_request.status,
            "created_at": agent_request.created_at
        }

        if generated_content:
            result["generated_content_id"] = str(generated_content.id)
            result["content_type"] = generated_content.content_type
            result["quality_score"] = generated_content.quality_score

        if agent_request.user_id:
            result["user_id"] = str(agent_request.user_id)

        return result

    except ValidationError as e:
        logger.warning(f"Validation error getting request status: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error getting request status: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving request status"
        )


@router.get("/session-info")
async def get_session_info(
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """
    Get user session information and agent usage statistics.
    """
    try:
        # Update session activity
        UserSessionManager.update_session_activity(str(current_user.id))

        # Get session stats
        session_stats = UserSessionManager.get_user_session_stats(str(current_user.id))

        # Get database statistics
        db_stats = get_user_agent_statistics(db, str(current_user.id))

        # Get recent requests
        recent_requests = get_user_agent_history(db, str(current_user.id), limit=10)

        # Get user preferences
        user_preferences = UserSessionManager.get_user_preferences(str(current_user.id))

        result = {
            "session_stats": session_stats,
            "database_stats": db_stats,
            "recent_requests": recent_requests,
            "preferences": user_preferences,
            "user_id": str(current_user.id),
            "username": current_user.email
        }

        logger.info(f"Session info requested for user: {current_user.id}")
        return result

    except Exception as e:
        logger.error(f"Error getting session info: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving session information"
        )