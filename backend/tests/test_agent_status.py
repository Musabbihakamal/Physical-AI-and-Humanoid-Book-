"""
Tests for the agent request status endpoint.
"""
import pytest
import uuid


def test_get_request_status(client):
    """Test getting the status of an agent request with an invalid ID."""
    # Use an invalid UUID to test error handling
    invalid_uuid = "invalid-uuid"
    response = client.get(f"/api/agents/status/{invalid_uuid}")
    assert response.status_code == 422  # Validation error


def test_get_request_status_not_found(client):
    """Test getting the status of a non-existent agent request."""
    # Use a valid but non-existent UUID
    valid_uuid = str(uuid.uuid4())
    response = client.get(f"/api/agents/status/{valid_uuid}")
    assert response.status_code == 404  # Not found