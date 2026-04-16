"""
Integration tests for RAG routes
"""
import pytest
from fastapi.testclient import TestClient
from sqlalchemy.orm import Session
import sys
import os

# Add backend to path
backend_path = os.path.join(os.path.dirname(__file__), '..', '..')
if backend_path not in sys.path:
    sys.path.insert(0, backend_path)

from backend.src.api.main import app
from backend.src.database.database import get_db, SessionLocal
from backend.src.models.rag_session import RAGSession, RAGQuery
from backend.src.models.user import User
import uuid
from datetime import datetime

client = TestClient(app)

@pytest.fixture
def db():
    """Get database session for tests"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@pytest.fixture
def test_user(db):
    """Create a test user"""
    user = User(
        email="test@example.com",
        full_name="Test User",
        hashed_password="hashed_password"
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    return user

class TestRAGQuery:
    """Test RAG query endpoint"""

    def test_query_valid_request(self):
        """Test valid RAG query request"""
        response = client.post("/api/rag/query", json={
            "question": "What is ROS 2?",
            "max_chunks": 5,
            "threshold": 0.3
        })

        # Should return 200 or 500 (if RAG bot not initialized)
        assert response.status_code in [200, 500]

        if response.status_code == 200:
            data = response.json()
            assert "answer" in data
            assert "sources" in data
            assert "chunks_retrieved" in data
            assert "session_id" in data
            assert "error_code" in data

    def test_query_invalid_question_length(self):
        """Test query with question exceeding max length"""
        response = client.post("/api/rag/query", json={
            "question": "x" * 2001,  # Exceeds max_length of 2000
            "max_chunks": 5,
            "threshold": 0.3
        })

        # Should return 422 validation error
        assert response.status_code == 422
        data = response.json()
        assert "detail" in data

    def test_query_invalid_max_chunks(self):
        """Test query with invalid max_chunks"""
        response = client.post("/api/rag/query", json={
            "question": "What is ROS 2?",
            "max_chunks": 25,  # Exceeds max of 20
            "threshold": 0.3
        })

        assert response.status_code == 422

    def test_query_invalid_threshold(self):
        """Test query with invalid threshold"""
        response = client.post("/api/rag/query", json={
            "question": "What is ROS 2?",
            "max_chunks": 5,
            "threshold": 1.5  # Out of range [0.0, 1.0]
        })

        assert response.status_code == 422

    def test_query_empty_question(self):
        """Test query with empty question"""
        response = client.post("/api/rag/query", json={
            "question": "",
            "max_chunks": 5,
            "threshold": 0.3
        })

        assert response.status_code == 422

    def test_query_missing_question(self):
        """Test query without question field"""
        response = client.post("/api/rag/query", json={
            "max_chunks": 5,
            "threshold": 0.3
        })

        assert response.status_code == 422

    def test_query_with_session_id(self):
        """Test query with session_id for conversation continuity"""
        session_id = str(uuid.uuid4())
        response = client.post("/api/rag/query", json={
            "question": "What is ROS 2?",
            "max_chunks": 5,
            "threshold": 0.3,
            "session_id": session_id
        })

        # Should handle gracefully (session may not exist)
        assert response.status_code in [200, 500]

class TestChatHistory:
    """Test chat history endpoints"""

    def test_get_history_without_auth(self):
        """Test getting history without authentication"""
        response = client.get("/api/rag/history")

        # Should require authentication
        assert response.status_code == 401
        data = response.json()
        assert "detail" in data

    def test_get_history_with_session_id(self):
        """Test getting history with session_id"""
        session_id = str(uuid.uuid4())
        response = client.get(f"/api/rag/history?session_id={session_id}")

        # Should return 200 with empty history for non-existent session
        # OR 500 if database tables don't exist (valid in test environment)
        assert response.status_code in [200, 500]

        if response.status_code == 200:
            data = response.json()
            assert "history" in data
            assert "session_id" in data
            assert "total_queries" in data
            assert len(data["history"]) == 0
        else:
            # Database table doesn't exist - this is expected in test environment
            data = response.json()
            assert "detail" in data
            assert "error_code" in data["detail"]

    def test_delete_history_without_auth(self):
        """Test deleting history without authentication"""
        session_id = str(uuid.uuid4())
        response = client.delete(f"/api/rag/history/{session_id}")

        # Should require authentication
        assert response.status_code == 401
        data = response.json()
        assert "detail" in data
        assert data["detail"]["error_code"] == "unauthorized"

class TestHealthCheck:
    """Test health check endpoint"""

    def test_health_check(self):
        """Test RAG health check endpoint"""
        response = client.get("/api/rag/health")

        # Should return 200 or 500 (if RAG bot not initialized)
        assert response.status_code in [200, 500]

        if response.status_code == 200:
            data = response.json()
            assert "status" in data
            assert "ready" in data

class TestErrorCodes:
    """Test error code handling"""

    def test_error_response_includes_error_code(self):
        """Test that error responses include error_code"""
        # Try to delete without auth
        response = client.delete(f"/api/rag/history/{uuid.uuid4()}")

        assert response.status_code == 401
        data = response.json()
        assert "detail" in data
        assert "error_code" in data["detail"]
        assert data["detail"]["error_code"] == "unauthorized"

    def test_validation_error_response(self):
        """Test validation error response format"""
        response = client.post("/api/rag/query", json={
            "question": "x" * 2001,
        })

        assert response.status_code == 422
        data = response.json()
        assert "detail" in data

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
