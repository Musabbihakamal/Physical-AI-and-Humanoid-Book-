"""
Authentication API routes for the multi-agent book generation system.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
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
import secrets
import hashlib
import base64

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


class OAuthLoginRequest(BaseModel):
    provider: str  # "google" or "github"
    access_token: str
    email: str
    full_name: str
    avatar_url: Optional[str] = None


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


class OAuthLoginResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str
    user_id: str
    email: str
    full_name: str
    is_new_user: bool  # Whether this was a new user registration


class UpdateProfileRequest(BaseModel):
    full_name: Optional[str] = None
    experience_level: Optional[str] = None
    technical_background: Optional[str] = None
    preferred_difficulty: Optional[str] = None
    learning_goals: Optional[list] = None
    hardware_access: Optional[list] = None
    language_preference: Optional[str] = None


@router.post("/oauth-login", response_model=OAuthLoginResponse)
async def oauth_login(
    request: OAuthLoginRequest,
    db: Session = Depends(get_db)
):
    """
    OAuth login/registration endpoint for Google and GitHub.
    """
    try:
        # Check if user already exists with this email
        existing_user = db.query(User).filter(User.email == request.email).first()

        if existing_user:
            # User exists, generate tokens
            access_token, refresh_token = AuthService.generate_tokens(existing_user, db)

            logger.info(f"OAuth login successful for existing user: {request.email}")

            return OAuthLoginResponse(
                access_token=access_token,
                refresh_token=refresh_token,
                token_type="bearer",
                user_id=str(existing_user.id),
                email=existing_user.email,
                full_name=existing_user.full_name,
                is_new_user=False
            )
        else:
            # User doesn't exist, create new user
            # Generate a random password for OAuth users (not used for login)
            random_password = secrets.token_urlsafe(32)

            user, access_token, refresh_token = AuthService.register_user(
                db=db,
                email=request.email,
                password=random_password,  # Random password for OAuth users
                full_name=request.full_name,
                experience_level="BEGINNER",  # Default for new OAuth users
                technical_background=f"Registered via {request.provider}",
                preferred_difficulty="MEDIUM",
                learning_goals=[],
                hardware_access=[],
                language_preference="en"
            )

            logger.info(f"OAuth registration successful for new user: {user.email}")

            return OAuthLoginResponse(
                access_token=access_token,
                refresh_token=refresh_token,
                token_type="bearer",
                user_id=str(user.id),
                email=user.email,
                full_name=user.full_name,
                is_new_user=True
            )

    except ValidationError as e:
        logger.warning(f"Validation error during OAuth login: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"message": e.message, "field": e.field}
        )
    except Exception as e:
        logger.error(f"Error during OAuth login: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during OAuth login"
        )


# OAuth Configuration - Add these routes for Google and GitHub OAuth
@router.get("/google")
async def google_login(request: Request):
    """
    Initiate Google OAuth flow.
    This endpoint redirects the user to Google's OAuth consent screen.
    """
    # Get Google OAuth configuration from environment variables
    from urllib.parse import urlencode
    import os

    google_client_id = os.getenv("GOOGLE_CLIENT_ID")
    google_client_secret = os.getenv("GOOGLE_CLIENT_SECRET")

    # Check if the dummy values are set (meaning OAuth is not configured)
    if google_client_id and "dummy_for_now" in google_client_id:
        google_client_id = None

    redirect_uri = os.getenv("GOOGLE_REDIRECT_URI", f"{request.url.scheme}://{request.url.netloc}/api/auth/google/callback")

    if not google_client_id:
        # Return configuration error instead of raising exception to ensure route exists
        return {
            "error": "Google OAuth is not configured",
            "detail": "Google OAuth is not configured. Please contact the administrator to set up Google OAuth integration.",
            "configured": False
        }

    # Generate a state parameter for security
    state = secrets.token_urlsafe(32)

    # Save state in session or database for validation (simplified here)
    # In a real app, you'd use a proper session store

    # Build the authorization URL
    params = {
        "client_id": google_client_id,
        "redirect_uri": redirect_uri,
        "response_type": "code",
        "scope": "openid email profile",
        "state": state,
    }

    auth_url = f"https://accounts.google.com/o/oauth2/auth?{urlencode(params)}"
    return {"auth_url": auth_url, "configured": True}


@router.get("/github")
async def github_login(request: Request):
    """
    Initiate GitHub OAuth flow.
    This endpoint redirects the user to GitHub's OAuth consent screen.
    """
    # Get GitHub OAuth configuration from environment variables
    from urllib.parse import urlencode
    import os

    github_client_id = os.getenv("GITHUB_CLIENT_ID")
    github_client_secret = os.getenv("GITHUB_CLIENT_SECRET")

    # Check if the dummy values are set (meaning OAuth is not configured)
    if github_client_id and "dummy_for_now" in github_client_id:
        github_client_id = None

    redirect_uri = os.getenv("GITHUB_REDIRECT_URI", f"{request.url.scheme}://{request.url.netloc}/api/auth/github/callback")

    if not github_client_id:
        # Return configuration error instead of raising exception to ensure route exists
        return {
            "error": "GitHub OAuth is not configured",
            "detail": "GitHub OAuth is not configured. Please contact the administrator to set up GitHub OAuth integration.",
            "configured": False
        }

    # Generate a state parameter for security
    state = secrets.token_urlsafe(32)

    # Build the authorization URL
    params = {
        "client_id": github_client_id,
        "redirect_uri": redirect_uri,
        "scope": "user:email",
        "state": state,
    }

    auth_url = f"https://github.com/login/oauth/authorize?{urlencode(params)}"
    return {"auth_url": auth_url, "configured": True}


# OAuth Callback endpoints - These handle the response from OAuth providers
@router.get("/google/callback")
async def google_callback(
    request: Request,
    code: str,
    state: str,
    db: Session = Depends(get_db)
):
    """
    Handle Google OAuth callback.
    Exchange the authorization code for an access token and get user info.
    """
    import os
    import requests
    from urllib.parse import urlencode, quote
    from fastapi.responses import RedirectResponse

    google_client_id = os.getenv("GOOGLE_CLIENT_ID")
    google_client_secret = os.getenv("GOOGLE_CLIENT_SECRET")
    redirect_uri = os.getenv("GOOGLE_REDIRECT_URI")

    if not google_client_id or not google_client_secret:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Google OAuth is not properly configured. Please set GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET environment variables."
        )

    # Exchange authorization code for access token
    token_url = "https://oauth2.googleapis.com/token"
    token_data = {
        "code": code,
        "client_id": google_client_id,
        "client_secret": google_client_secret,
        "redirect_uri": redirect_uri,
        "grant_type": "authorization_code",
    }

    token_response = requests.post(token_url, data=token_data)
    token_json = token_response.json()

    if "access_token" not in token_json:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to get access token from Google."
        )

    # Get user info from Google
    access_token = token_json["access_token"]
    user_info_response = requests.get(
        "https://www.googleapis.com/oauth2/v2/userinfo",
        headers={"Authorization": f"Bearer {access_token}"}
    )
    user_info = user_info_response.json()

    if "email" not in user_info:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to get user info from Google."
        )

    # Process OAuth login and get tokens
    existing_user = db.query(User).filter(User.email == user_info["email"]).first()

    if existing_user:
        # User exists, generate tokens
        access_token, refresh_token = AuthService.generate_tokens(existing_user, db)
        is_new_user = False
    else:
        # User doesn't exist, create new user
        random_password = secrets.token_urlsafe(32)
        user, access_token, refresh_token = AuthService.register_user(
            db=db,
            email=user_info["email"],
            password=random_password,
            full_name=user_info.get("name", user_info.get("email", "Unknown")),
            experience_level="BEGINNER",
            technical_background="Registered via Google OAuth",
            preferred_difficulty="MEDIUM",
            learning_goals=[],
            hardware_access=[],
            language_preference="en"
        )
        is_new_user = True

    # Redirect to frontend with tokens as URL parameters
    # Determine the frontend URL based on the request
    frontend_base_url = str(request.url).split('/api/auth/google/callback')[0].rstrip('/')

    # For development, typically the frontend runs on port 3000
    if 'localhost' in frontend_base_url or '127.0.0.1' in frontend_base_url:
        # Try common frontend ports
        frontend_base_url = f"{request.url.scheme}://{request.url.hostname}:3000"

    redirect_url = f"{frontend_base_url}/auth/callback?provider=google&access_token={access_token}&refresh_token={refresh_token}&user_id={str(existing_user.id if existing_user else user.id)}&email={quote(user_info['email'])}&full_name={quote(user_info.get('name', user_info.get('email', 'Unknown')))}&is_new_user={is_new_user}"

    return RedirectResponse(url=redirect_url)


@router.get("/github/callback")
async def github_callback(
    request: Request,
    code: str,
    state: str,
    db: Session = Depends(get_db)
):
    """
    Handle GitHub OAuth callback.
    Exchange the authorization code for an access token and get user info.
    """
    import os
    import requests
    from urllib.parse import urlencode, quote
    from fastapi.responses import RedirectResponse

    github_client_id = os.getenv("GITHUB_CLIENT_ID")
    github_client_secret = os.getenv("GITHUB_CLIENT_SECRET")
    redirect_uri = os.getenv("GITHUB_REDIRECT_URI")

    if not github_client_id or not github_client_secret:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="GitHub OAuth is not properly configured. Please set GITHUB_CLIENT_ID and GITHUB_CLIENT_SECRET environment variables."
        )

    # Exchange authorization code for access token
    token_url = "https://github.com/login/oauth/access_token"
    token_data = {
        "code": code,
        "client_id": github_client_id,
        "client_secret": github_client_secret,
        "redirect_uri": redirect_uri,
    }

    headers = {"Accept": "application/json"}
    token_response = requests.post(token_url, data=token_data, headers=headers)
    token_json = token_response.json()

    if "access_token" not in token_json:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to get access token from GitHub."
        )

    # Get user info from GitHub
    access_token = token_json["access_token"]
    user_headers = {
        "Authorization": f"token {access_token}",
        "Accept": "application/vnd.github.v3+json"
    }
    user_info_response = requests.get("https://api.github.com/user", headers=user_headers)
    user_info = user_info_response.json()

    # Get user's primary email from GitHub
    email_response = requests.get("https://api.github.com/user/emails", headers=user_headers)
    emails = email_response.json()
    primary_email = next((email["email"] for email in emails if email["primary"] and email["verified"]), None)

    if not primary_email:
        # If no primary email found, get the first verified email
        primary_email = next((email["email"] for email in emails if email["verified"]), user_info.get("login", "") + "@github.com")

    # Process OAuth login and get tokens
    existing_user = db.query(User).filter(User.email == primary_email).first()

    if existing_user:
        # User exists, generate tokens
        access_token, refresh_token = AuthService.generate_tokens(existing_user, db)
        is_new_user = False
    else:
        # User doesn't exist, create new user
        random_password = secrets.token_urlsafe(32)
        user, access_token, refresh_token = AuthService.register_user(
            db=db,
            email=primary_email,
            password=random_password,
            full_name=user_info.get("name", user_info.get("login", "Unknown")),
            experience_level="BEGINNER",
            technical_background="Registered via GitHub OAuth",
            preferred_difficulty="MEDIUM",
            learning_goals=[],
            hardware_access=[],
            language_preference="en"
        )
        is_new_user = True

    # Redirect to frontend with tokens as URL parameters
    # Determine the frontend URL based on the request
    frontend_base_url = str(request.url).split('/api/auth/github/callback')[0].rstrip('/')

    # For development, typically the frontend runs on port 3000
    if 'localhost' in frontend_base_url or '127.0.0.1' in frontend_base_url:
        # Try common frontend ports
        frontend_base_url = f"{request.url.scheme}://{request.url.hostname}:3000"

    redirect_url = f"{frontend_base_url}/auth/callback?provider=github&access_token={access_token}&refresh_token={refresh_token}&user_id={str(existing_user.id if existing_user else user.id)}&email={quote(primary_email)}&full_name={quote(user_info.get('name', user_info.get('login', 'Unknown')))}&is_new_user={is_new_user}"

    return RedirectResponse(url=redirect_url)


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


@router.put("/me")
async def update_user_profile(
    request: UpdateProfileRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update current user's profile information.
    """
    from ..services.user_service import UserService

    try:
        # Update user's basic information if provided
        if request.full_name is not None:
            current_user.full_name = request.full_name
            db.add(current_user)

        # Get or create user profile
        user_profile = db.query(UserProfile).filter(
            UserProfile.user_id == current_user.id
        ).first()

        if user_profile:
            # Update existing profile
            updated_profile = UserService.update_user_profile(
                db=db,
                user_id=current_user.id,
                experience_level=request.experience_level,
                technical_background=request.technical_background,
                preferred_difficulty=request.preferred_difficulty,
                learning_goals=request.learning_goals,
                hardware_access=request.hardware_access,
                language_preference=request.language_preference
            )
        else:
            # Create new profile if it doesn't exist
            updated_profile = UserService.create_user_profile(
                db=db,
                experience_level=request.experience_level or "BEGINNER",
                technical_background=request.technical_background,
                preferred_difficulty=request.preferred_difficulty,
                learning_goals=request.learning_goals or [],
                hardware_access=request.hardware_access or [],
                language_preference=request.language_preference or "en"
            )
            # Link the profile to the user by setting the user_id to match the user's id
            updated_profile.user_id = current_user.id
            db.add(updated_profile)

        db.commit()

        # Return updated user information
        return {
            "user_id": str(current_user.id),
            "email": current_user.email,
            "full_name": current_user.full_name,
            "is_active": current_user.is_active,
            "profile": {
                "experience_level": updated_profile.experience_level,
                "technical_background": updated_profile.technical_background,
                "preferred_difficulty": updated_profile.preferred_difficulty,
                "learning_goals": updated_profile.learning_goals,
                "hardware_access": updated_profile.hardware_access,
                "language_preference": updated_profile.language_preference
            }
        }

    except Exception as e:
        db.rollback()
        logger.error(f"Error updating user profile: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while updating user profile"
        )