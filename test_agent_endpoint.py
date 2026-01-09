import sys
import os
import json
import requests
import time

def test_agent_endpoint():
    """Test the agent endpoint to make sure it's working properly."""
    url = "http://localhost:8000/api/agent/query"

    # Test query - using the retrieval_params structure expected by the API
    payload = {
        "query": "What are Docusaurus blogging features?",
        "retrieval_params": {
            "top_k": 2,
            "threshold": -0.1  # Low threshold to accommodate negative similarity scores
        },
        "grounding_required": False  # Try without strict grounding first
    }

    try:
        print("Testing agent endpoint...")
        print(f"Sending query: {payload['query']}")

        response = requests.post(
            url,
            json=payload,
            headers={"Content-Type": "application/json"}
        )

        print(f"Agent endpoint response: {response.status_code}")
        print(f"Response headers: {response.headers}")
        print(f"Full response text: {response.text}")

        if response.status_code == 200:
            try:
                result = response.json()
                print(f"Answer: {result.get('answer', 'No answer field')[:200]}...")
                print(f"Sources: {len(result.get('sources', []))}")
                print(f"Confidence: {result.get('confidence_score', 'N/A')}")
                if 'error' in result:
                    print(f"Error in response: {result['error']}")
            except ValueError:
                print("Response is not valid JSON")
        else:
            print(f"Error response: {response.text}")

        return response.status_code == 200
    except requests.exceptions.ConnectionError:
        print("Could not connect to agent endpoint. Server may not be running properly.")
        return False
    except Exception as e:
        print(f"Error testing agent endpoint: {e}")
        return False

if __name__ == "__main__":
    test_agent_endpoint()