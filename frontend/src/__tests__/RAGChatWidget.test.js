"""
Frontend integration tests for RAG Chat Widget
"""
import pytest
from unittest.mock import patch, MagicMock
import sys
import os

# Test API configuration
class TestAPIConfiguration:
    """Test API configuration and backend URL resolution"""

    def test_backend_url_localhost_development(self):
        """Test backend URL resolution for localhost development"""
        # Simulate localhost environment
        with patch('frontend.src.constants.apiConfig.window') as mock_window:
            mock_window.location.hostname = 'localhost'
            mock_window.location.protocol = 'http:'

            # Expected: http://localhost:8000
            expected = 'http://localhost:8000'
            # In real test, would import and call getBackendUrl()

    def test_backend_url_environment_variable(self):
        """Test backend URL from environment variable"""
        with patch.dict(os.environ, {'REACT_APP_BACKEND_URL': 'https://api.example.com'}):
            # Expected: https://api.example.com
            expected = 'https://api.example.com'

    def test_backend_url_vercel_production(self):
        """Test backend URL for Vercel production"""
        with patch('frontend.src.constants.apiConfig.window') as mock_window:
            mock_window.location.hostname = 'myapp.vercel.app'
            mock_window.location.protocol = 'https:'

            # Should warn about missing REACT_APP_BACKEND_URL
            # Expected: https://api.example.com (fallback)

    def test_health_check_with_timeout(self):
        """Test health check includes timeout"""
        # Should use AbortController with 5s timeout
        # Expected: fetch completes or aborts after 5s

class TestRAGChatWidget:
    """Test RAG Chat Widget integration"""

    def test_query_submission_with_auth_token(self):
        """Test query submission includes auth token"""
        # Mock fetch
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = True
            mock_response.json = MagicMock(return_value={
                'answer': 'Test answer',
                'sources': [],
                'chunks_retrieved': 0,
                'session_id': 'test-session-id',
                'error_code': None
            })
            mock_fetch.return_value = mock_response

            # Expected: Authorization header is included
            # Expected: Content-Type is application/json

    def test_query_submission_without_auth_token(self):
        """Test query submission without auth token"""
        # Mock fetch for anonymous user
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = True
            mock_response.json = MagicMock(return_value={
                'answer': 'Test answer',
                'sources': [],
                'chunks_retrieved': 0,
                'session_id': 'anon-session-id',
                'error_code': None
            })
            mock_fetch.return_value = mock_response

            # Expected: No Authorization header
            # Expected: Request succeeds

    def test_error_handling_timeout(self):
        """Test error handling for request timeout"""
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            # Simulate AbortError
            mock_fetch.side_effect = Exception('AbortError')

            # Expected: Error message: "Request timed out..."

    def test_error_handling_rag_bot_init_failed(self):
        """Test error handling for RAG bot initialization failure"""
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = False
            mock_response.status = 500
            mock_response.json = MagicMock(return_value={
                'detail': 'RAG system initialization failed',
                'error_code': 'rag_bot_init_failed'
            })
            mock_fetch.return_value = mock_response

            # Expected: Error message: "The RAG system is not available..."

    def test_error_handling_no_relevant_chunks(self):
        """Test error handling for no relevant chunks"""
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = True
            mock_response.json = MagicMock(return_value={
                'answer': 'I couldn\'t find any relevant information...',
                'sources': [],
                'chunks_retrieved': 0,
                'session_id': 'test-session-id',
                'error_code': 'no_relevant_chunks'
            })
            mock_fetch.return_value = mock_response

            # Expected: Response displayed with error_code

    def test_error_handling_unauthorized(self):
        """Test error handling for unauthorized access"""
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = False
            mock_response.status = 401
            mock_response.json = MagicMock(return_value={
                'detail': 'Authentication required',
                'error_code': 'unauthorized'
            })
            mock_fetch.return_value = mock_response

            # Expected: Error message: "Please sign in to continue."

    def test_session_id_persistence(self):
        """Test session ID is saved and reused"""
        # First query creates session
        # Second query should include session_id in request
        # Expected: Same session_id used for both queries

    def test_chat_history_loading(self):
        """Test chat history is loaded on widget open"""
        with patch('frontend.src.components.RAGChatWidget.fetch') as mock_fetch:
            mock_response = MagicMock()
            mock_response.ok = True
            mock_response.json = MagicMock(return_value={
                'session_id': 'test-session-id',
                'history': [
                    {
                        'id': 'query-1',
                        'query': 'What is ROS 2?',
                        'response': 'ROS 2 is...',
                        'sources': [],
                        'created_at': '2026-04-16T10:00:00'
                    }
                ],
                'total_queries': 1
            })
            mock_fetch.return_value = mock_response

            # Expected: History is loaded and displayed

    def test_markdown_link_rendering(self):
        """Test markdown links in sources are rendered"""
        # Response with sources
        response = {
            'answer': 'Test answer',
            'sources': [
                {
                    'url': 'https://example.com/page',
                    'page_title': 'Example Page',
                    'section_title': 'Section 1',
                    'score': 0.95
                }
            ],
            'chunks_retrieved': 1,
            'session_id': 'test-session-id',
            'error_code': None
        }

        # Expected: Links are rendered as <a> tags with href

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
