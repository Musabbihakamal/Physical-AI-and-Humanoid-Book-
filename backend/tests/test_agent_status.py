"""
Tests for the agent request status endpoint.
"""
import pytest
import uuid


def test_get_request_status(authenticated_client):
    """Test getting the status of an agent request with an invalid ID."""
    # Use an invalid UUID to test error handling
    invalid_uuid = "invalid-uuid"
    response = authenticated_client.get(f"/api/agents/status/{invalid_uuid}")
    # Should return 422 for validation error or 404 for not found, but not 403 (unauthorized)
    assert response.status_code in [400, 404, 422], f"Expected validation error, got {response.status_code}: {response.text}"


def test_get_request_status_not_found(authenticated_client):
    """Test getting the status of a non-existent agent request."""
    # Use a valid but non-existent UUID
    valid_uuid = str(uuid.uuid4())
    response = authenticated_client.get(f"/api/agents/status/{valid_uuid}")
    # Should return 404 for not found, but not 403 (unauthorized)
    assert response.status_code in [404], f"Expected not found error, got {response.status_code}: {response.text}"