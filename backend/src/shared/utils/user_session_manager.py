"""
User session management for agent interactions.
"""
from typing import Dict, Any, Optional, List
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
import logging
from ..models.agent_request import AgentRequest
from ..models.user import User
from ..models.generated_content import GeneratedContent

logger = logging.getLogger(__name__)

# In-memory session storage (in production, use Redis or database)
USER_SESSIONS: Dict[str, Dict[str, Any]] = {}


class UserSessionManager:
    """
    Manages user sessions for agent interactions.
    """

    @staticmethod
    def get_user_session(user_id: str) -> Dict[str, Any]:
        """
        Get or create a user session.
        """
        if user_id not in USER_SESSIONS:
            USER_SESSIONS[user_id] = {
                "created_at": datetime.utcnow(),
                "last_activity": datetime.utcnow(),
                "active_requests": [],
                "recent_requests": [],
                "preferences": {},
                "history": []
            }
        return USER_SESSIONS[user_id]

    @staticmethod
    def update_session_activity(user_id: str):
        """
        Update the last activity timestamp for a user session.
        """
        session = UserSessionManager.get_user_session(user_id)
        session["last_activity"] = datetime.utcnow()

    @staticmethod
    def add_active_request(user_id: str, request_id: str, agent_type: str):
        """
        Add an active request to the user session.
        """
        session = UserSessionManager.get_user_session(user_id)
        session["active_requests"].append({
            "request_id": request_id,
            "agent_type": agent_type,
            "timestamp": datetime.utcnow()
        })

    @staticmethod
    def remove_active_request(user_id: str, request_id: str):
        """
        Remove an active request from the user session.
        """
        session = UserSessionManager.get_user_session(user_id)
        session["active_requests"] = [
            req for req in session["active_requests"]
            if req["request_id"] != request_id
        ]

    @staticmethod
    def add_to_history(user_id: str, request_id: str, agent_type: str, result: Dict[str, Any]):
        """
        Add a completed request to the user's history.
        """
        session = UserSessionManager.get_user_session(user_id)

        # Add to recent requests
        session["recent_requests"].append({
            "request_id": request_id,
            "agent_type": agent_type,
            "timestamp": datetime.utcnow(),
            "result_summary": result.get("summary", "")[:100] if result.get("summary") else ""
        })

        # Limit recent requests to 50
        if len(session["recent_requests"]) > 50:
            session["recent_requests"] = session["recent_requests"][-50:]

        # Add to history
        session["history"].append({
            "request_id": request_id,
            "agent_type": agent_type,
            "timestamp": datetime.utcnow(),
            "result_summary": result.get("summary", "")[:100] if result.get("summary") else ""
        })

        # Limit history to 200
        if len(session["history"]) > 200:
            session["history"] = session["history"][-200:]

    @staticmethod
    def get_user_preferences(user_id: str) -> Dict[str, Any]:
        """
        Get user preferences from session.
        """
        session = UserSessionManager.get_user_session(user_id)
        return session.get("preferences", {})

    @staticmethod
    def set_user_preference(user_id: str, key: str, value: Any):
        """
        Set a user preference in session.
        """
        session = UserSessionManager.get_user_session(user_id)
        session["preferences"][key] = value

    @staticmethod
    def get_recent_requests(user_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get recent requests for a user.
        """
        session = UserSessionManager.get_user_session(user_id)
        return session["recent_requests"][-limit:]

    @staticmethod
    def cleanup_expired_sessions():
        """
        Clean up sessions that have been inactive for more than 24 hours.
        """
        cutoff_time = datetime.utcnow() - timedelta(hours=24)
        expired_users = [
            user_id for user_id, session in USER_SESSIONS.items()
            if session["last_activity"] < cutoff_time
        ]

        for user_id in expired_users:
            del USER_SESSIONS[user_id]

        logger.info(f"Cleaned up {len(expired_users)} expired sessions")

    @staticmethod
    def get_user_session_stats(user_id: str) -> Dict[str, Any]:
        """
        Get statistics for a user's session.
        """
        session = UserSessionManager.get_user_session(user_id)
        return {
            "active_requests_count": len(session["active_requests"]),
            "recent_requests_count": len(session["recent_requests"]),
            "total_history_count": len(session["history"]),
            "session_age_hours": (datetime.utcnow() - session["created_at"]).total_seconds() / 3600,
            "last_activity": session["last_activity"]
        }


def get_user_agent_history(db: Session, user_id: str, agent_type: Optional[str] = None, limit: int = 10) -> List[Dict[str, Any]]:
    """
    Get user's agent request history from database.
    """
    query = db.query(AgentRequest).filter(AgentRequest.user_id == user_id)

    if agent_type:
        query = query.filter(AgentRequest.agent_type == agent_type)

    recent_requests = query.order_by(AgentRequest.created_at.desc()).limit(limit).all()

    return [
        {
            "id": str(req.id),
            "agent_type": req.agent_type,
            "status": req.status,
            "created_at": req.created_at,
            "parameters": req.parameters,
            "context": req.context
        }
        for req in recent_requests
    ]


def get_user_agent_statistics(db: Session, user_id: str) -> Dict[str, Any]:
    """
    Get user's agent usage statistics.
    """
    total_requests = db.query(AgentRequest).filter(AgentRequest.user_id == user_id).count()

    agent_stats = {}
    for agent_type in ["GLOSSARY_MAKER", "CODE_EXPLAINER", "QUIZ_CREATOR", "CHAPTER_GENERATOR"]:
        count = db.query(AgentRequest).filter(
            AgentRequest.user_id == user_id,
            AgentRequest.agent_type == agent_type
        ).count()
        agent_stats[agent_type] = count

    completed_requests = db.query(AgentRequest).filter(
        AgentRequest.user_id == user_id,
        AgentRequest.status == "COMPLETED"
    ).count()

    return {
        "total_requests": total_requests,
        "completed_requests": completed_requests,
        "agent_breakdown": agent_stats,
        "success_rate": (completed_requests / total_requests * 100) if total_requests > 0 else 0
    }