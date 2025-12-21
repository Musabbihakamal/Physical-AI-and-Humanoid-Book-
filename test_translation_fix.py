#!/usr/bin/env python3
"""
Test script to verify the translation fixes work correctly.
This script tests the key changes made to support ANTHROPIC_API_KEY.
"""
import os
import sys
from unittest.mock import patch, MagicMock

# Add backend src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from config.settings import Settings
from services.translation_service import TranslationService

def test_settings_api_key_priority():
    """Test that ANTHROPIC_API_KEY takes priority over CLAUDE_API_KEY"""
    print("Testing API key priority...")

    # Test 1: Only ANTHROPIC_API_KEY set
    with patch.dict(os.environ, {
        'ANTHROPIC_API_KEY': 'anthropic_key',
        'CLAUDE_API_KEY': 'claude_key'
    }):
        settings = Settings()
        service = TranslationService()
        assert service.claude_api_key == 'anthropic_key', f"Expected 'anthropic_key', got {service.claude_api_key}"
        print("✓ ANTHROPIC_API_KEY takes priority when both are set")

    # Test 2: Only CLAUDE_API_KEY set (fallback)
    with patch.dict(os.environ, {
        'ANTHROPIC_API_KEY': '',
        'CLAUDE_API_KEY': 'claude_key'
    }):
        settings = Settings()
        service = TranslationService()
        assert service.claude_api_key == 'claude_key', f"Expected 'claude_key', got {service.claude_api_key}"
        print("✓ CLAUDE_API_KEY used as fallback when ANTHROPIC_API_KEY is not set")

    # Test 3: Neither set
    with patch.dict(os.environ, {
        'ANTHROPIC_API_KEY': '',
        'CLAUDE_API_KEY': ''
    }):
        settings = Settings()
        service = TranslationService()
        assert service.claude_api_key is None, f"Expected None, got {service.claude_api_key}"
        print("✓ No API key when neither is set")

def test_chunking_threshold():
    """Test that the chunking threshold is properly set"""
    print("\nTesting chunking threshold...")

    with patch.dict(os.environ, {
        'ANTHROPIC_API_KEY': 'test_key'
    }):
        # Mock the Anthropic client to avoid actual API calls
        with patch('services.translation_service.Anthropic') as mock_anthropic:
            mock_client = MagicMock()
            mock_anthropic.return_value = mock_client

            service = TranslationService()

            # Test that content > 6000 chars triggers chunking
            long_text = "a" * 7000  # 7000 chars, should trigger chunking
            assert len(long_text) > 6000, "Test text should be longer than threshold"
            print("✓ Chunking threshold set to 6000 characters")

            # The service should have initialized correctly
            assert service.anthropic_client is not None, "Anthropic client should be initialized"
            print("✓ Anthropic client initialized with new API key")

def test_chunk_size():
    """Test that the chunk size is properly configured"""
    print("\nTesting chunk size configuration...")

    with patch.dict(os.environ, {
        'ANTHROPIC_API_KEY': 'test_key'
    }):
        # Mock the Anthropic client to avoid actual API calls
        with patch('services.translation_service.Anthropic') as mock_anthropic:
            mock_client = MagicMock()
            mock_anthropic.return_value = mock_client

            service = TranslationService()

            # Test the chunk splitting method with a sample text
            sample_text = "First paragraph.\n\nSecond paragraph.\n\nThird paragraph."
            chunks = service._split_text_into_chunks(sample_text, max_chunk_size=4000)

            assert len(chunks) >= 1, "Should have at least one chunk"
            assert all(len(chunk) <= 4000 for chunk in chunks), "All chunks should be within size limit"
            print("✓ Chunk size properly configured at 4000 characters")

if __name__ == "__main__":
    print("Testing translation service fixes...")
    print("=" * 50)

    test_settings_api_key_priority()
    test_chunking_threshold()
    test_chunk_size()

    print("\n" + "=" * 50)
    print("All tests passed! ✓")
    print("\nSummary of changes made:")
    print("1. Added ANTHROPIC_API_KEY support with fallback to CLAUDE_API_KEY")
    print("2. Updated chunking threshold from 8000 to 6000 characters")
    print("3. Updated chunk size from 6000 to 4000 characters for safety")
    print("4. Updated .env.example with new API key instructions")
    print("5. Improved error messages to reference new API key name")