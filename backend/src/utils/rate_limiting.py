"""
Rate limiting utilities for agent endpoints.
"""
from fastapi import HTTPException, status
from sqlalchemy.orm import Session
import time
import logging
from typing import Dict, Tuple
from ..models.agent_request import AgentRequest

logger = logging.getLogger(__name__)

# In-memory rate limiting storage (in production, use Redis or similar)
REQUEST_COUNTS: Dict[str, Tuple[int, int]] = {}  # {user_id: (last_reset_time, count)}
RATE_LIMIT_WINDOW = 3600  # 1 hour window
MAX_REQUESTS_PER_HOUR = 50  # Max 50 requests per hour per user


def check_rate_limit(db: Session, user_id: str, agent_type: str) -> bool:
    """
    Check if user has exceeded rate limit for agent requests.
    """
    global REQUEST_COUNTS

    current_time = int(time.time())
    window_start = current_time - RATE_LIMIT_WINDOW

    # Clean up old entries
    REQUEST_COUNTS = {
        key: (timestamp, count)
        for key, (timestamp, count) in REQUEST_COUNTS.items()
        if timestamp > window_start
    }

    # Create a key specific to user and agent type
    user_agent_key = f"{user_id}:{agent_type}"

    if user_agent_key in REQUEST_COUNTS:
        timestamp, count = REQUEST_COUNTS[user_agent_key]
        if count >= MAX_REQUESTS_PER_HOUR and timestamp > window_start:
            logger.warning(f"Rate limit exceeded for user {user_id}, agent {agent_type}")
            return False
    return True


def record_agent_request(db: Session, user_id: str, agent_type: str) -> bool:
    """
    Record an agent request for rate limiting purposes.
    """
    global REQUEST_COUNTS

    current_time = int(time.time())
    user_agent_key = f"{user_id}:{agent_type}"

    if user_agent_key in REQUEST_COUNTS:
        timestamp, count = REQUEST_COUNTS[user_agent_key]
        if current_time - timestamp < RATE_LIMIT_WINDOW:
            REQUEST_COUNTS[user_agent_key] = (timestamp, count + 1)
        else:
            REQUEST_COUNTS[user_agent_key] = (current_time, 1)
    else:
        REQUEST_COUNTS[user_agent_key] = (current_time, 1)

    logger.info(f"Recorded agent request for user {user_id}, type: {agent_type}")
    return True


def get_user_agent_usage(db: Session, user_id: str, agent_type: str) -> Dict[str, int]:
    """
    Get usage statistics for a user and agent type.
    """
    global REQUEST_COUNTS

    current_time = int(time.time())
    window_start = current_time - RATE_LIMIT_WINDOW

    user_agent_key = f"{user_id}:{agent_type}"

    if user_agent_key in REQUEST_COUNTS:
        timestamp, count = REQUEST_COUNTS[user_agent_key]
        if timestamp > window_start:
            remaining = max(0, MAX_REQUESTS_PER_HOUR - count)
            return {
                "used": count,
                "remaining": remaining,
                "limit": MAX_REQUESTS_PER_HOUR,
                "reset_time": timestamp + RATE_LIMIT_WINDOW
            }

    return {
        "used": 0,
        "remaining": MAX_REQUESTS_PER_HOUR,
        "limit": MAX_REQUESTS_PER_HOUR,
        "reset_time": current_time + RATE_LIMIT_WINDOW
    }