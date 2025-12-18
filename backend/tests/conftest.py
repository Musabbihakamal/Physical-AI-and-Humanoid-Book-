"""
Pytest configuration for the multi-agent book generation system API tests.
"""
import sys
import os
# Add the project root directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from fastapi.testclient import TestClient
from src.database.database import Base, engine
from src.models import agent_request, generated_content, book_chapter, content_link, rag_session, user_profile

# Ensure all models are imported to register them with Base.metadata
_ = agent_request, generated_content, book_chapter, content_link, rag_session, user_profile

# Create the tables for testing
Base.metadata.create_all(bind=engine)

from src.api.main import app


@pytest.fixture(scope="module")
def client():
    """Create a test client with the app."""
    return TestClient(app)