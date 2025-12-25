"""
Main API application for the book generation system.
"""
import sys
import os
# Add the project root directory to the Python path so shared modules can be imported
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from ..database.database import engine, Base
from . import rag_routes, content_routes, book_content_routes, auth_routes
from ..config import settings
import logging
from contextlib import asynccontextmanager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
    description="API for book generation and RAG",
    version="1.0.0",
    lifespan=lifespan
)

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

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "book-system"}

# Root endpoint
@app.get("/")
async def root():
    return {"message": "Welcome to the Book + RAG Bot System API"}