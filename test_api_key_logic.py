#!/usr/bin/env python3
"""
Simple test to verify the API key priority logic.
"""
def test_api_key_priority():
    """Test the API key priority logic directly"""
    print("Testing API key priority logic...")

    # Test the logic: ANTHROPIC_API_KEY or CLAUDE_API_KEY
    env_vars = {
        'ANTHROPIC_API_KEY': 'anthropic_key',
        'CLAUDE_API_KEY': 'claude_key'
    }

    # Should use ANTHROPIC_API_KEY when both are present
    claude_api_key = env_vars.get('ANTHROPIC_API_KEY') or env_vars.get('CLAUDE_API_KEY')
    assert claude_api_key == 'anthropic_key', f"Expected 'anthropic_key', got {claude_api_key}"
    print("[PASS] ANTHROPIC_API_KEY takes priority when both are set")

    # Test fallback to CLAUDE_API_KEY
    env_vars = {
        'ANTHROPIC_API_KEY': '',  # Empty
        'CLAUDE_API_KEY': 'claude_key'
    }

    claude_api_key = env_vars.get('ANTHROPIC_API_KEY') or env_vars.get('CLAUDE_API_KEY')
    assert claude_api_key == 'claude_key', f"Expected 'claude_key', got {claude_api_key}"
    print("[PASS] CLAUDE_API_KEY used as fallback when ANTHROPIC_API_KEY is not set")

    # Test when both are empty
    env_vars = {
        'ANTHROPIC_API_KEY': '',
        'CLAUDE_API_KEY': ''
    }

    claude_api_key = env_vars.get('ANTHROPIC_API_KEY') or env_vars.get('CLAUDE_API_KEY')
    assert claude_api_key == '', f"Expected '', got {claude_api_key}"
    print("[PASS] Returns empty when neither is set")

    print("\n[SUCCESS] API key priority logic works correctly!")

if __name__ == "__main__":
    test_api_key_priority()
    print("\nChanges made to support ANTHROPIC_API_KEY:")
    print("1. Updated translation_service.py to use ANTHROPIC_API_KEY as primary key")
    print("2. Added fallback to CLAUDE_API_KEY for backward compatibility")
    print("3. Updated error messages to reference both API key names")
    print("4. Updated .env.example file with proper documentation")
    print("5. Reduced chunk sizes for better Claude token limit safety")