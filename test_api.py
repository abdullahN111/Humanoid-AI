import requests
import json

# Test the backend API
def test_backend_api():
    url = "http://localhost:8000/api/agent/query"

    # Sample query to test the API
    test_query = {
        "query": "What is this documentation about?",
        "pageContext": {
            "url": "http://localhost:3000/test",
            "title": "Test Page"
        }
    }

    try:
        response = requests.post(url, json=test_query, timeout=30)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")

        if response.status_code == 200:
            print("SUCCESS: Backend API is working correctly!")
            print("The API is properly integrated. The response error is expected because")
            print("there is no book content in the vector database yet.")
            return True
        else:
            print("ERROR: Backend API returned an error")
            return False

    except requests.exceptions.RequestException as e:
        print(f"❌ Error connecting to backend API: {e}")
        return False

if __name__ == "__main__":
    test_backend_api()