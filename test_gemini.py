import os
import sys
from dotenv import load_dotenv

# Add backend to path
sys.path.insert(0, 'backend')

# Load environment
load_dotenv('backend/.env')

from openai import OpenAI
from src.config.settings import settings

def test_gemini_connection():
    print("Testing Gemini API connection...")

    print(f"GEMINI_API_KEY: {'SET' if settings.GEMINI_API_KEY else 'NOT SET'}")
    print(f"GEMINI_BASE_URL: {settings.GEMINI_BASE_URL}")

    if not settings.GEMINI_API_KEY or not settings.GEMINI_BASE_URL:
        print("Missing Gemini API configuration!")
        return

    try:
        # Create OpenAI client with Gemini configuration
        client = OpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url=settings.GEMINI_BASE_URL
        )

        print("OpenAI client created successfully")

        # Try a simple test call
        response = client.chat.completions.create(
            model="models/gemini-2.5-flash",  # Standard format for Gemini via API
            messages=[
                {"role": "user", "content": "Say 'hello' in one word."}
            ],
            max_tokens=10
        )

        print(f"API call successful: {response.choices[0].message.content}")

    except Exception as e:
        print(f"API call failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_gemini_connection()