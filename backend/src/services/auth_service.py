"""
Authentication service for the multi-agent book generation system.
Handles user registration, login, JWT token generation, and user management.
"""
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from datetime import datetime, timedelta
from typing import Optional, Tuple, Dict, Any
import jwt
import uuid
from ..models.user import User
from ..models.user_profile import UserProfile
from ..models.token import Token
from ..config import settings
from ..utils.errors import ValidationError
import logging
from passlib.context import CryptContext

logger = logging.getLogger(__name__)

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class AuthService:
    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a plain text password against its hash."""
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def get_password_hash(password: str) -> str:
        """Hash a plain text password."""
        return pwd_context.hash(password)

    @staticmethod
    def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token."""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(
                minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES
            )
        to_encode.update({"exp": expire, "type": "access"})
        encoded_jwt = jwt.encode(
            to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM
        )
        return encoded_jwt

    @staticmethod
    def create_refresh_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT refresh token."""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(days=30)  # Refresh tokens last 30 days
        to_encode.update({"exp": expire, "type": "refresh"})
        encoded_jwt = jwt.encode(
            to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM
        )
        return encoded_jwt

    @staticmethod
    def verify_token(token: str) -> Optional[Dict[str, Any]]:
        """Verify a JWT token and return its payload."""
        try:
            payload = jwt.decode(
                token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM]
            )
            return payload
        except jwt.exceptions.ExpiredSignatureError:
            logger.warning("Token has expired")
            return None
        except jwt.exceptions.JWTError as e:
            logger.warning(f"Token verification failed: {str(e)}")
            return None

    @staticmethod
    def register_user(
        db: Session,
        email: str,
        password: str,
        full_name: str,
        experience_level: str = "BEGINNER",
        technical_background: Optional[str] = None,
        preferred_difficulty: Optional[str] = None,
        learning_goals: Optional[list] = None,
        hardware_access: Optional[list] = None,
        language_preference: str = "en"
    ) -> Tuple[User, str, str]:
        """
        Register a new user with authentication and profile.

        Args:
            db: Database session
            email: User's email
            password: User's plain text password
            full_name: User's full name
            experience_level: User's experience level for profile
            technical_background: User's technical background for profile
            preferred_difficulty: User's preferred difficulty for profile
            learning_goals: User's learning goals for profile
            hardware_access: User's hardware access for profile
            language_preference: User's language preference for profile

        Returns:
            Tuple of (User object, access_token, refresh_token)
        """
        # Validate password length (bcrypt limitation is 72 bytes)
        if len(password.encode('utf-8')) > 72:
            raise ValidationError("Password cannot be longer than 72 bytes", "password")
        try:
            # Check if user already exists
            existing_user = db.query(User).filter(User.email == email).first()
            if existing_user:
                raise ValidationError("Email already registered", "email")

            # Create the user
            try:
                user = User(
                    email=email,
                    password=password,
                    full_name=full_name
                )
            except ValueError as ve:
                # Handle bcrypt password length error
                if "password cannot be longer than 72 bytes" in str(ve):
                    raise ValidationError("Password cannot be longer than 72 bytes", "password")
                else:
                    raise ve
            db.add(user)
            db.flush()  # Get the user ID without committing

            # Create user profile
            user_profile = UserProfile(
                experience_level=experience_level,
                technical_background=technical_background,
                preferred_difficulty=preferred_difficulty,
                learning_goals=learning_goals,
                hardware_access=hardware_access,
                language_preference=language_preference
            )
            # Set the user_profile's user_id to match the user's id
            user_profile.user_id = user.id
            db.add(user_profile)

            db.commit()
            db.refresh(user)

            # Generate tokens
            access_token = AuthService.create_access_token(
                data={"sub": str(user.id), "email": user.email}
            )
            refresh_token = AuthService.create_refresh_token(
                data={"sub": str(user.id), "email": user.email}
            )

            # Store refresh token in database
            token_expires = datetime.utcnow() + timedelta(days=30)
            token_record = Token(
                user_id=str(user.id),
                token=refresh_token,
                token_type="refresh",
                expires_at=token_expires
            )
            db.add(token_record)
            db.commit()

            logger.info(f"User registered successfully: {user.email}")
            return user, access_token, refresh_token

        except ValidationError:
            # Re-raise validation errors
            raise
        except IntegrityError:
            db.rollback()
            raise ValidationError("Email already registered", "email")
        except Exception as e:
            db.rollback()
            logger.error(f"Error registering user: {str(e)}")
            raise

    @staticmethod
    def authenticate_user(db: Session, email: str, password: str) -> Optional[Tuple[User, str, str]]:
        """
        Authenticate a user with email and password.

        Args:
            db: Database session
            email: User's email
            password: User's plain text password

        Returns:
            Tuple of (User object, access_token, refresh_token) or None if authentication fails
        """
        # Validate password length (bcrypt limitation is 72 bytes)
        if len(password.encode('utf-8')) > 72:
            logger.warning(f"Authentication failed: password too long for user {email}")
            return None

        try:
            # Find user by email
            user = db.query(User).filter(User.email == email).first()
            if not user:
                logger.warning(f"Authentication failed: user {email} not found")
                return None

            # Check if user is active
            if not user.is_active:
                logger.warning(f"Authentication failed: user {email} is inactive")
                return None

            # Verify password (with bcrypt error handling)
            try:
                password_valid = user.check_password(password)
            except ValueError as ve:
                # Handle bcrypt password length error during verification
                if "password cannot be longer than 72 bytes" in str(ve):
                    logger.warning(f"Authentication failed: password too long for user {email}")
                    return None
                else:
                    raise ve

            if not password_valid:
                logger.warning(f"Authentication failed: incorrect password for user {email}")
                return None

            # Generate tokens
            access_token = AuthService.create_access_token(
                data={"sub": str(user.id), "email": user.email}
            )
            refresh_token = AuthService.create_refresh_token(
                data={"sub": str(user.id), "email": user.email}
            )

            # Store refresh token in database
            token_expires = datetime.utcnow() + timedelta(days=30)
            token_record = Token(
                user_id=str(user.id),
                token=refresh_token,
                token_type="refresh",
                expires_at=token_expires
            )
            db.add(token_record)
            db.commit()

            logger.info(f"User authenticated successfully: {user.email}")
            return user, access_token, refresh_token

        except Exception as e:
            logger.error(f"Error authenticating user: {str(e)}")
            return None

    @staticmethod
    def refresh_access_token(db: Session, refresh_token: str) -> Optional[Tuple[str, str]]:
        """
        Refresh an access token using a refresh token.

        Args:
            db: Database session
            refresh_token: The refresh token

        Returns:
            Tuple of (new access_token, new refresh_token) or None if invalid
        """
        try:
            # Verify the refresh token
            payload = AuthService.verify_token(refresh_token)
            if not payload or payload.get("type") != "refresh":
                logger.warning("Invalid refresh token")
                return None

            # Check if refresh token exists in database and is not blacklisted
            token_record = db.query(Token).filter(
                Token.token == refresh_token,
                Token.is_blacklisted == False
            ).first()

            if not token_record or token_record.is_expired():
                logger.warning("Refresh token not found in DB or is expired/blacklisted")
                return None

            # Generate new tokens
            new_access_token = AuthService.create_access_token(
                data={"sub": payload["sub"], "email": payload["email"]}
            )
            new_refresh_token = AuthService.create_refresh_token(
                data={"sub": payload["sub"], "email": payload["email"]}
            )

            # Blacklist the old refresh token
            token_record.is_blacklisted = True

            # Store the new refresh token in database
            new_token_expires = datetime.utcnow() + timedelta(days=30)
            new_token_record = Token(
                user_id=token_record.user_id,
                token=new_refresh_token,
                token_type="refresh",
                expires_at=new_token_expires
            )
            db.add(new_token_record)
            db.commit()

            logger.info(f"Access token refreshed for user: {payload['sub']}")
            return new_access_token, new_refresh_token

        except Exception as e:
            logger.error(f"Error refreshing access token: {str(e)}")
            return None

    @staticmethod
    def logout_user(db: Session, user_id: str, token: str) -> bool:
        """
        Logout a user by blacklisting their refresh token.

        Args:
            db: Database session
            user_id: User ID
            token: Token to blacklist

        Returns:
            True if successful, False otherwise
        """
        try:
            # Find the token in the database
            token_record = db.query(Token).filter(
                Token.user_id == user_id,
                Token.token == token,
                Token.token_type == "refresh",
                Token.is_blacklisted == False
            ).first()

            if token_record:
                # Blacklist the token
                token_record.is_blacklisted = True
                db.commit()
                logger.info(f"User logged out: {user_id}")
                return True

            return False

        except Exception as e:
            logger.error(f"Error logging out user: {str(e)}")
            return False

    @staticmethod
    def get_current_user(db: Session, token: str) -> Optional[User]:
        """
        Get the current user from a token.

        Args:
            db: Database session
            token: JWT token

        Returns:
            User object or None if invalid token
        """
        try:
            payload = AuthService.verify_token(token)
            if not payload:
                return None

            user_id = payload.get("sub")
            if not user_id:
                return None

            user = db.query(User).filter(User.id == user_id).first()
            return user

        except Exception as e:
            logger.error(f"Error getting current user: {str(e)}")
            return None