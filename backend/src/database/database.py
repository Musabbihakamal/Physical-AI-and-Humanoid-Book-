"""
Database setup and configuration for the multi-agent book generation system.
"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/book_agent_db")

# Check if we're in test mode
import sys
TESTING = 'pytest' in sys.modules
if TESTING:
    # Use in-memory SQLite for testing
    DATABASE_URL = "sqlite:///:memory:"

# Import appropriate types based on database
# Check if we're using SQLite (which doesn't support native UUID)
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/book_agent_db")
if 'sqlite' in DATABASE_URL.lower() or TESTING:
    # For SQLite, use String type for UUID
    from sqlalchemy import String, DateTime, Text, JSON
    from sqlalchemy.sql import func
    UUID_TYPE = String(36)  # Standard UUID length
else:
    # For PostgreSQL, use the PostgreSQL UUID type
    from sqlalchemy import String, DateTime, Text, JSON
    from sqlalchemy.dialects.postgresql import UUID
    from sqlalchemy.sql import func
    UUID_TYPE = UUID(as_uuid=True)

# Create the database engine
if TESTING:
    engine = create_engine(
        DATABASE_URL,
        connect_args={"check_same_thread": False}  # Required for SQLite
    )
else:
    engine = create_engine(
        DATABASE_URL,
        poolclass=QueuePool,
        pool_size=10,
        max_overflow=20,
        pool_pre_ping=True,
        pool_recycle=300,
    )


# Create a configured "Session" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create a Base class for declarative models
Base = declarative_base()


def get_db():
    """
    Dependency function to get database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Export the UUID type for use in models
# (UUID_TYPE already defined above based on TESTING flag)