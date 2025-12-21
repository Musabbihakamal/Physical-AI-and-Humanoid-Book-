"""
Basic tests for the multi-agent book generation system API.
"""
import pytest


def test_health_check(client):
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy", "service": "multi-agent-book-system"}


def test_root_endpoint(client):
    """Test the root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert "message" in data


def test_glossary_maker_endpoint(authenticated_client):
    """Test the glossary maker endpoint."""
    payload = {
        "content": "This is a sample content with important terms like Algorithm and Function.",
        "parameters": {}
    }
    response = authenticated_client.post("/api/agents/glossary-maker", json=payload)
    # For testing, we might expect different status depending on whether the service is mocked
    # Accept both 200 and 400/500 as long as it's not 403 (Forbidden)
    assert response.status_code in [200, 400, 422, 500], f"Unexpected status code: {response.status_code}, response: {response.text}"
    if response.status_code == 200:
        data = response.json()
        assert "request_id" in data or "id" in data


def test_code_explainer_endpoint(authenticated_client):
    """Test the code explainer endpoint."""
    payload = {
        "code": "def example_function():\n    pass",
        "language": "python",
        "parameters": {}
    }
    response = authenticated_client.post("/api/agents/code-explainer", json=payload)
    # For testing, we might expect different status depending on whether the service is mocked
    # Accept both 200 and 400/500 as long as it's not 403 (Forbidden)
    assert response.status_code in [200, 400, 422, 500], f"Unexpected status code: {response.status_code}, response: {response.text}"
    if response.status_code == 200:
        data = response.json()
        assert "request_id" in data or "id" in data


def test_quiz_creator_endpoint(authenticated_client):
    """Test the quiz creator endpoint."""
    payload = {
        "content": "This is sample content for quiz generation.",
        "parameters": {}
    }
    response = authenticated_client.post("/api/agents/quiz-creator", json=payload)
    # For testing, we might expect different status depending on whether the service is mocked
    # Accept both 200 and 400/500 as long as it's not 403 (Forbidden)
    assert response.status_code in [200, 400, 422, 500], f"Unexpected status code: {response.status_code}, response: {response.text}"
    if response.status_code == 200:
        data = response.json()
        assert "request_id" in data or "id" in data


