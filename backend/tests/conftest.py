"""
Pytest configuration for the multi-agent book generation system API tests.
"""
import sys
import os

# Get repo root
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

# Add paths for imports
sys.path.insert(0, os.path.join(REPO_ROOT, "backend", "src"))
sys.path.insert(0, os.path.join(REPO_ROOT, "shared"))

# Import the application and models


import pytest
from fastapi.testclient import TestClient
from src.database.database import Base, engine
from src.models import generated_content, book_chapter, content_link, rag_session, user_profile, user, token
from src.api.main import app

# Ensure all models are imported to register them with Base.metadata
_ = generated_content, book_chapter, content_link, rag_session, user_profile, user, token

# Create the tables for testing
Base.metadata.create_all(bind=engine)


@pytest.fixture(scope="module")
def client():
    """Create a test client with the app."""
    return TestClient(app)


@pytest.fixture(scope="function")  # Changed to function scope to ensure fresh client for each test
def authenticated_client():
    """
    Create an authenticated test client.
    For testing purposes, we'll create a client that can bypass authentication
    by using a test token or by temporarily disabling auth for test endpoints.
    """
    test_client = TestClient(app)

    # Add a fake authorization header to trigger the authentication flow
    # The actual authentication will be bypassed in test mode via the auth_dependencies
    original_request = test_client.request
    def patched_request(method, url, **kwargs):
        # Add the Authorization header to trigger the authentication flow
        if 'headers' not in kwargs:
            kwargs['headers'] = {}
        # Use a fake token - the actual validation will be bypassed in test mode
        kwargs['headers']['Authorization'] = f'Bearer fake-test-token'
        return original_request(method, url, **kwargs)

    test_client.request = patched_request
    test_client.get = lambda url, **kwargs: patched_request('GET', url, **kwargs)
    test_client.post = lambda url, **kwargs: patched_request('POST', url, **kwargs)
    test_client.put = lambda url, **kwargs: patched_request('PUT', url, **kwargs)
    test_client.delete = lambda url, **kwargs: patched_request('DELETE', url, **kwargs)

    return test_client
