"""
User authentication model for the multi-agent book generation system.
"""
from sqlalchemy import Column, String, DateTime, Boolean, Text
from sqlalchemy.sql import func
from ..database.database import Base, UUID_TYPE
from passlib.context import CryptContext
import uuid
import hashlib


# Password hashing context - must be identical to AuthService
try:
    # Try to initialize bcrypt context
    pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
    # Test if bcrypt works properly
    test_hash = pwd_context.hash("test")
except Exception:
    # Fallback to sha256_crypt if bcrypt is not working properly
    pwd_context = CryptContext(schemes=["sha256_crypt"], deprecated="auto")


class User(Base):
    __tablename__ = "users"

    # Fields
    id = Column(UUID_TYPE, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, nullable=False, index=True)
    hashed_password = Column(String, nullable=False)
    full_name = Column(String, nullable=False)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __init__(self, email: str = None, password: str = None, full_name: str = None, **kwargs):
        # If password is provided, this is a new user being created
        if password is not None:
            # Validate password length (bcrypt limitation is 72 bytes)
            if len(password.encode('utf-8')) > 72:
                raise ValueError("Password cannot be longer than 72 bytes")

            super().__init__(**kwargs)
            self.email = email
            self.hashed_password = self.get_password_hash(password)
            self.full_name = full_name
        else:
            # This is being instantiated by SQLAlchemy from DB, let the ORM handle it
            super().__init__(**kwargs)

    @staticmethod
    def get_password_hash(password: str) -> str:
        """Hash a plain text password."""
        try:
            return pwd_context.hash(password)
        except ValueError as e:
            # Handle bcrypt specific errors (like password length issues)
            if "password cannot be longer than 72 bytes" in str(e):
                raise ValueError("Password cannot be longer than 72 bytes")
            else:
                # Re-raise other bcrypt-related errors
                raise e

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a plain text password against its hash."""
        return pwd_context.verify(plain_password, hashed_password)

    def check_password(self, password: str) -> bool:
        """Check if the provided password matches the stored hash."""
        return User.verify_password(password, self.hashed_password)