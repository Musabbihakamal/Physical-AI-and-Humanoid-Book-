"""
Main API application for the book generation system.
"""
import sys
import os
from dotenv import load_dotenv
import logging

# Configure logging early
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load .env file FIRST before any other imports
# Try multiple paths to find .env
env_paths = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), '../../.env')),  # backend/.env
    os.path.abspath('.env'),  # Current directory
    os.path.abspath('backend/.env'),  # From project root
]

for env_path in env_paths:
    if os.path.exists(env_path):
        logger.info(f"Loading .env from: {env_path}")
        load_dotenv(env_path)
        break

# Verify ANTHROPIC_API_KEY is loaded
anthropic_key = os.getenv("ANTHROPIC_API_KEY")
logger.info(f"ANTHROPIC_API_KEY loaded: {anthropic_key[:20] if anthropic_key else 'NOT SET'}...")

# Add the project root directory to the Python path so shared modules can be imported
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from fastapi import FastAPI, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from slowapi.middleware import SlowAPIMiddleware
from ..database.database import engine, Base
from . import rag_routes, content_routes, book_content_routes, auth_routes, translation_routes
from ..config import settings
from .rate_limiting import limiter, custom_rate_limit_handler
from slowapi.errors import RateLimitExceeded
import logging
from contextlib import asynccontextmanager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('logs/app.log', mode='a')
    ]
)
logger = logging.getLogger(__name__)

# Set specific loggers to INFO level
logging.getLogger('src.api.translation_routes').setLevel(logging.INFO)
logging.getLogger('src.services.translation_service').setLevel(logging.INFO)

# Create database tables using lifespan
@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    logger.info("Database tables created successfully")
    yield
    logger.info("Application shutdown")

# Create the FastAPI app
app = FastAPI(
    title="Book + RAG Bot System API",
    description="API for book generation and RAG with rate limiting",
    version="1.0.0",
    lifespan=lifespan
)

# Add rate limiting - MUST be done before adding middleware
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, custom_rate_limit_handler)

# Add SlowAPI middleware FIRST (before CORS)
app.add_middleware(SlowAPIMiddleware)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import all model modules to ensure they're registered with Base.metadata
# This ensures that all models are registered with Base.metadata before table creation
from ..models import generated_content, book_chapter, content_link, rag_session, user_profile, user, token


# For testing purposes, create tables immediately if in test mode
# This needs to happen after models are imported but before the app is used
if 'pytest' in sys.modules:
    logger.info("Running in test mode, creating tables immediately...")
    Base.metadata.create_all(bind=engine)
    logger.info("Test database tables created successfully")

# Include API routes
app.include_router(rag_routes.router, prefix="/api/rag", tags=["rag"])
app.include_router(content_routes.router, prefix="/api/content", tags=["content"])
app.include_router(book_content_routes.router, prefix="/api/book-content", tags=["book-content"])
app.include_router(auth_routes.router, prefix="/api/auth", tags=["auth"])
app.include_router(translation_routes.router, prefix="/api/translate", tags=["translation"])

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "book-system"}

# Root endpoint
@app.get("/")
async def root():
    return {"message": "Welcome to the Book + RAG Bot System API"}