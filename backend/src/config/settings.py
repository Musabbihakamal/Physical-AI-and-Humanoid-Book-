"""
Settings configuration for the multi-agent book generation system.
"""
from pydantic_settings import BaseSettings
from typing import List, Optional


class Settings(BaseSettings):
    # Database settings
    DATABASE_URL: str = "postgresql://user:password@localhost/book_agent_db"

    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Book + RAG Bot + Multi-Agent System"

    # Security settings
    SECRET_KEY: str = "your-secret-key-here"
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # CORS settings
    ALLOWED_ORIGINS: List[str] = ["*"]  # In production, specify exact origins

    # External API keys
    OPENAI_API_KEY: Optional[str] = None
    QDRANT_URL: Optional[str] = None
    QDRANT_API_KEY: Optional[str] = None
    CLAUDE_API_KEY: Optional[str] = None
    ANTHROPIC_API_KEY: Optional[str] = None

    # OAuth settings
    GOOGLE_CLIENT_ID: Optional[str] = None
    GOOGLE_CLIENT_SECRET: Optional[str] = None
    GITHUB_CLIENT_ID: Optional[str] = None
    GITHUB_CLIENT_SECRET: Optional[str] = None
    GOOGLE_REDIRECT_URI: Optional[str] = None
    GITHUB_REDIRECT_URI: Optional[str] = None

    # Agent settings
    DEFAULT_AGENT_TIMEOUT: int = 30  # seconds
    MAX_CONTENT_LENGTH: int = 100000  # characters

    # RAG settings
    RAG_CHUNK_SIZE: int = 512
    RAG_OVERLAP: int = 64
    RAG_TOP_K: int = 5

    # Additional settings that might be in environment
    DEBUG: Optional[bool] = None
    LOG_LEVEL: Optional[str] = "INFO"

    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()