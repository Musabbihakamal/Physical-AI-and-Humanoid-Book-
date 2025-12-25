"""
Pytest configuration for the multi-agent book generation system API tests.
"""
import sys
import os

# -----------------------------
# Fix Python import paths
# -----------------------------

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../..")
)

# backend/src -> for `src.*`
sys.path.insert(0, os.path.join(REPO_ROOT, "backend", "src"))

# shared -> for `shared.*`
sys.path.insert(0, os.path.join(REPO_ROOT, "shared"))

# -----------------------------
# Normal imports
# -----------------------------

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

# Register models
_ = agent_request, generated_content, book_chapter, content_link, rag_session, user_profile, user, token

# Create tables for tests
Base.metadata.create_all(bind=engine)


@pytest.fixture(scope="module")
def client():
    return TestClient(app)


@pytest.fixture(scope="function")
def authenticated_client():
    test_client = TestClient(app)

    original_request = test_client.request

    def patched_request(method, url, **kwargs):
        headers = kwargs.setdefault("headers", {})
        headers["Authorization"] = "Bearer fake-test-token"
        return original_request(method, url, **kwargs)

    test_client.request = patched_request
    test_client.get = lambda url, **kw: patched_request("GET", url, **kw)
    test_client.post = lambda url, **kw: patched_request("POST", url, **kw)
    test_client.put = lambda url, **kw: patched_request("PUT", url, **kw)
    test_client.delete = lambda url, **kw: patched_request("DELETE", url, **kw)

    return test_client
