"""
Authentication API routes for the multi-agent book generation system.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr
from typing import Optional
from ..database.database import get_db
from ..services.auth_service import AuthService
from ..models.user import User
from ..models.user_profile import UserProfile
from ..utils.errors import ValidationError
from .dependencies import get_current_user, get_token_from_header
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# Pydantic models for request/response validation
class RegisterRequest(BaseModel):
    email: EmailStr
    password: str
    full_name: str
    experience_level: str = "BEGINNER"
    technical_background: Optional[str] = None
    preferred_difficulty: Optional[str] = None
    learning_goals: Optional[list] = []
    hardware_access: Optional[list] = []
    language_preference: str = "en"

    class Config:
        schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securepassword123",
                "full_name": "John Doe",
                "experience_level": "INTERMEDIATE",
                "technical_background": "Software developer with 5 years experience",
                "preferred_difficulty": "MEDIUM",
                "learning_goals": ["Learn AI concepts", "Build robotics applications"],
                "hardware_access": ["Raspberry Pi", "Arduino"],
                "language_preference": "en"
            }
        }


class LoginRequest(BaseModel):
    email: EmailStr
    password: str

    class Config:
        schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securepassword123"
            }
        }


class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str
    user_id: str
    email: str
    full_name: str


class RefreshTokenRequest(BaseModel):
    refresh_token: str


class RefreshTokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str


@router.post("/register", response_model=TokenResponse)
async def register(
    request: RegisterRequest,
    db: Session = Depends(get_db)
):
    """
    Register a new user with authentication and profile.
    """
    try:
        # Validate experience level
        valid_experience_levels = ["BEGINNER", "INTERMEDIATE", "EXPERT"]
        if request.experience_level not in valid_experience_levels:
            raise ValidationError(
                f"experience_level must be one of {valid_experience_levels}",
                "experience_level"
            )

        # Validate difficulty level if provided
        if request.preferred_difficulty:
            valid_difficulty_levels = ["EASY", "MEDIUM", "HARD"]
            if request.preferred_difficulty not in valid_difficulty_levels:
                raise ValidationError(
                    f"preferred_difficulty must be one of {valid_difficulty_levels}",
                    "preferred_difficulty"
                )

        # Register user with profile
        user, access_token, refresh_token = AuthService.register_user(
            db=db,
            email=request.email,
            password=request.password,
            full_name=request.full_name,
            experience_level=request.experience_level,
            technical_background=request.technical_background,
            preferred_difficulty=request.preferred_difficulty,
            learning_goals=request.learning_goals,
            hardware_access=request.hardware_access,
            language_preference=request.language_preference
        )

        logger.info(f"User registered successfully: {user.email}")

        return TokenResponse(
            access_token=access_token,
            refresh_token=refresh_token,
            token_type="bearer",
            user_id=str(user.id),
            email=user.email,
            full_name=user.full_name
        )

    except ValidationError as e:
        logger.warning(f"Validation error during registration: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error during registration: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during registration"
        )


@router.post("/login", response_model=TokenResponse)
async def login(
    request: LoginRequest,
    db: Session = Depends(get_db)
):
    """
    Authenticate user and return access/refresh tokens.
    """
    try:
        result = AuthService.authenticate_user(
            db=db,
            email=request.email,
            password=request.password
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        user, access_token, refresh_token = result

        logger.info(f"User logged in successfully: {user.email}")

        return TokenResponse(
            access_token=access_token,
            refresh_token=refresh_token,
            token_type="bearer",
            user_id=str(user.id),
            email=user.email,
            full_name=user.full_name
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error during login: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during login"
        )


@router.post("/refresh", response_model=RefreshTokenResponse)
async def refresh_token(
    request: RefreshTokenRequest,
    db: Session = Depends(get_db)
):
    """
    Refresh access token using refresh token.
    """
    try:
        result = AuthService.refresh_access_token(
            db=db,
            refresh_token=request.refresh_token
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid refresh token",
                headers={"WWW-Authenticate": "Bearer"},
            )

        new_access_token, new_refresh_token = result

        return RefreshTokenResponse(
            access_token=new_access_token,
            refresh_token=new_refresh_token,
            token_type="bearer"
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error during token refresh: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during token refresh"
        )


@router.post("/logout")
async def logout(
    # We'll get the current user from the token in headers
    current_user: User = Depends(get_current_user),
    token: str = Depends(get_token_from_header),
    db: Session = Depends(get_db)
):
    """
    Logout user by blacklisting refresh token.
    """
    try:
        success = AuthService.logout_user(
            db=db,
            user_id=str(current_user.id),
            token=token
        )

        if not success:
            # Token might not be a refresh token, but that's okay
            logger.info(f"User logged out: {current_user.email}")

        return {"message": "Successfully logged out"}

    except Exception as e:
        logger.error(f"Error during logout: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during logout"
        )


@router.get("/me")
async def get_user_profile(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get current user's profile information.
    """
    try:
        # Get the user's profile
        user_profile = db.query(UserProfile).filter(
            UserProfile.user_id == current_user.id
        ).first()

        return {
            "user_id": str(current_user.id),
            "email": current_user.email,
            "full_name": current_user.full_name,
            "is_active": current_user.is_active,
            "created_at": current_user.created_at,
            "profile": {
                "experience_level": user_profile.experience_level if user_profile else None,
                "technical_background": user_profile.technical_background if user_profile else None,
                "preferred_difficulty": user_profile.preferred_difficulty if user_profile else None,
                "learning_goals": user_profile.learning_goals if user_profile else [],
                "hardware_access": user_profile.hardware_access if user_profile else [],
                "language_preference": user_profile.language_preference if user_profile else "en"
            } if user_profile else None
        }

    except Exception as e:
        logger.error(f"Error getting user profile: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving user profile"
        )