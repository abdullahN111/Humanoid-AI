#!/usr/bin/env python3
"""
Test script to verify Gemini API connection with OpenAI-compatible endpoint
"""
import os
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

from openai import OpenAI
from src.config.settings import settings

def test_gemini_connection():
    print("Testing Gemini API connection...")
    print(f"GEMINI_API_KEY set: {bool(settings.GEMINI_API_KEY)}")
    print(f"GEMINI_BASE_URL: {settings.GEMINI_BASE_URL}")
    print(f"Model: gemini-2.5-flash")

    if not settings.GEMINI_API_KEY or not settings.GEMINI_BASE_URL:
        print("ERROR: Missing Gemini API key or base URL in settings")
        return False

    try:
        # Create OpenAI client with Gemini configuration
        client = OpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url=settings.GEMINI_BASE_URL
        )

        print("OpenAI client created successfully")

        # Test the connection with a simple request
        response = client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "user", "content": "Hello, are you working?"}
            ],
            max_tokens=50,
            temperature=0.3
        )

        print(f"API call successful! Response: {response.choices[0].message.content[:100]}...")
        print("Gemini API connection is working!")
        return True

    except Exception as e:
        print(f"ERROR: Gemini API connection failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_gemini_connection()
    sys.exit(0 if success else 1)