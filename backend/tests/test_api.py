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


def test_glossary_maker_endpoint(client):
    """Test the glossary maker endpoint."""
    payload = {
        "content": "This is a sample content with important terms like Algorithm and Function.",
        "parameters": {}
    }
    response = client.post("/api/agents/glossary-maker", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert "request_id" in data
    assert data["status"] == "PENDING"


def test_code_explainer_endpoint(client):
    """Test the code explainer endpoint."""
    payload = {
        "code": "def example_function():\n    pass",
        "language": "python",
        "parameters": {}
    }
    response = client.post("/api/agents/code-explainer", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert "request_id" in data
    assert data["status"] == "PENDING"


def test_quiz_creator_endpoint(client):
    """Test the quiz creator endpoint."""
    payload = {
        "content": "This is sample content for quiz generation.",
        "parameters": {}
    }
    response = client.post("/api/agents/quiz-creator", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert "request_id" in data
    assert data["status"] == "PENDING"


def test_chapter_generator_endpoint(client):
    """Test the chapter generator endpoint."""
    payload = {
        "module_focus": "Introduction to Robotics",
        "parameters": {}
    }
    response = client.post("/api/agents/chapter-generator", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert "request_id" in data
    assert data["status"] == "PENDING"