"""
Pytest configuration for the multi-agent book generation system API tests.
"""
import sys
import os

# Absolute path to repo root
REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../..")
)

# Add backend/src to PYTHONPATH
sys.path.insert(0, os.path.join(REPO_ROOT, "backend", "src"))

# Add shared to PYTHONPATH
sys.path.insert(0, os.path.join(REPO_ROOT, "shared"))

import pytest
from fastapi.testclient import TestClient
from src.database.database import Base, engine
from src.models import (
    agent_request,
    generated_content,
    book_chapter,
    content_link,
    rag_session,
    user_profile,
    user,
    token,
)
from src.api.main import app

# Ensure all models are registered
_ = agent_request, generated_content, book_chapter, content_link, rag_session, user_profile, user, token

Base.metadata.create_all(bind=engine)


@pytest.fixture(scope="module")
def client():
    return TestClient(app)

