import sys
import os
sys.path.insert(0, 'backend')

import requests
import time

def test_backend_connection():
    """Test if the backend server is running and responding to requests."""
    url = "http://localhost:8000/api/health"

    try:
        # Wait a moment for server to start if it's running
        time.sleep(2)

        response = requests.get(url)
        print(f"Health check response: {response.status_code}")
        print(f"Response: {response.json() if response.content else 'No content'}")
        return True
    except requests.exceptions.ConnectionError:
        print("Could not connect to backend server. It may not be running.")
        return False
    except Exception as e:
        print(f"Error testing backend connection: {e}")
        return False

if __name__ == "__main__":
    test_backend_connection()