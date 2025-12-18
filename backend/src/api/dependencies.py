"""
Authentication dependencies for the multi-agent book generation system.
"""
from fastapi import Depends, HTTPException, status, Header
from sqlalchemy.orm import Session
from typing import Optional
from ..database.database import get_db
from ..models.user import User
from ..services.auth_service import AuthService


def get_token_from_header(authorization: str = Header(default=None, description="Authorization header with Bearer token")) -> str:
    """
    Extract the JWT token from the Authorization header.

    Args:
        authorization: Authorization header value

    Returns:
        JWT token string

    Raises:
        HTTPException: If authorization header is missing or invalid
    """
    if authorization is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing",
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        scheme, token = authorization.split()
        if scheme.lower() != "bearer":
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication scheme",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return token
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authorization header format",
            headers={"WWW-Authenticate": "Bearer"},
        )


def get_current_user(
    token: str = Depends(get_token_from_header),
    db: Session = Depends(get_db)
) -> User:
    """
    Dependency to get the current authenticated user from the token.

    Args:
        token: JWT token from Authorization header
        db: Database session

    Returns:
        Current user object

    Raises:
        HTTPException: If token is invalid or user doesn't exist
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    user = AuthService.get_current_user(db, token)
    if user is None:
        raise credentials_exception

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Inactive user",
        )

    return user


def get_optional_current_user(
    token: Optional[str] = Depends(get_token_from_header),
    db: Session = Depends(get_db)
) -> Optional[User]:
    """
    Dependency to get the current user, but returns None if not authenticated.
    Useful for endpoints that work for both authenticated and anonymous users.

    Args:
        token: JWT token from Authorization header (optional)
        db: Database session

    Returns:
        Current user object or None if not authenticated
    """
    if token is None:
        return None

    user = AuthService.get_current_user(db, token)
    if user is None or not user.is_active:
        return None

    return user