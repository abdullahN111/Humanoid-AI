import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.config.settings import settings
from openai import OpenAI

def test_llm_call():
    print("Testing direct LLM call...")

    try:
        # Initialize OpenAI client - can work with external LLMs via base URL
        if settings.GEMINI_API_KEY and settings.GEMINI_BASE_URL:
            # Use Gemini via OpenAI-compatible endpoint
            client = OpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url=settings.GEMINI_BASE_URL
            )
            model = "gemini-2.5-flash"  # For Google API via OpenAI-compatible endpoint
            print(f"Using Gemini API with model: {model}")
        else:
            # Fallback to OpenAI
            client = OpenAI(api_key=settings.OPENAI_API_KEY)
            model = "gpt-4-turbo"  # Default model for OpenAI
            print(f"Using OpenAI with model: {model}")

        print("OpenAI client created successfully")

        # Test the connection with a simple request
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": "Hello, are you working?"}
            ],
            max_tokens=50,
            temperature=0.3
        )

        print(f"LLM call successful! Response: {response.choices[0].message.content}")
        return True

    except Exception as e:
        print(f"ERROR: LLM call failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_llm_call()
    sys.exit(0 if success else 1)