#!/usr/bin/env python3
"""
Simple test script to verify translation functionality works.
"""
import asyncio
import sys
import os

# Add the backend source to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from services.translation_service import translate_text

async def test_translation():
    """Test the translation service"""
    print("🧪 Testing translation service...")

    test_text = "Hello, this is a test of the robotics translation system."
    target_language = "ur"  # Urdu

    try:
        print(f"📝 Original text: {test_text}")
        print(f"🎯 Target language: {target_language}")
        print("⏳ Translating...")

        result = await translate_text(test_text, target_language)

        print(f"✅ Translation result: {result}")
        print(f"📊 Result type: {type(result)}")
        print(f"📏 Result length: {len(result)}")

        if result and result != test_text:
            print("🎉 Translation service is working!")
            return True
        else:
            print("⚠️ Translation returned same text or empty result")
            return False

    except Exception as e:
        print(f"❌ Translation failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_translation())
    if success:
        print("\n✅ Translation test PASSED")
        sys.exit(0)
    else:
        print("\n❌ Translation test FAILED")
        sys.exit(1)