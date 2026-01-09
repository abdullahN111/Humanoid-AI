import sys
import os
import requests
import time

# Add the backend directory to Python path
sys.path.insert(0, '.')

def test_api_endpoints():
    print("Testing API endpoints for the chatbot...")

    # Test if the backend server is running
    try:
        # Try to make a request to the health check endpoint
        response = requests.get("http://localhost:8000/health", timeout=5)
        print(f"Health check: {response.status_code}")
        if response.status_code == 200:
            print("API server is running")
        else:
            print("API server may not be running")
    except requests.exceptions.ConnectionError:
        print("API server is not running. Let's start it...")

        # Start the FastAPI server in a subprocess
        import subprocess
        import threading

        # Start the server in a separate process
        server_process = subprocess.Popen([
            sys.executable, "-c",
            "import uvicorn; from main import app; uvicorn.run(app, host='0.0.0.0', port=8000)"
        ], cwd=".")

        # Wait a few seconds for the server to start
        time.sleep(5)

        # Test the endpoints
        try:
            # Test health endpoint
            response = requests.get("http://localhost:8000/health", timeout=5)
            print(f"Health check: {response.status_code}")

            # Test ingestion endpoint
            try:
                response = requests.post("http://localhost:8000/api/ingest", timeout=10)
                print(f"Ingestion endpoint: {response.status_code}")
            except:
                print("Ingestion endpoint not available or server still starting")

            # Test chat endpoint
            try:
                response = requests.post(
                    "http://localhost:8000/api/chat",
                    json={"message": "What is humanoid robotics?", "context": {}},
                    timeout=10
                )
                print(f"Chat endpoint: {response.status_code}")
                if response.status_code == 200:
                    print("Chat endpoint is working!")
                    print(f"Response: {response.json()}")
            except Exception as e:
                print(f"Chat endpoint error: {e}")

        except Exception as e:
            print(f"Error testing API: {e}")
        finally:
            # Terminate the server process
            server_process.terminate()
            server_process.wait()

    print("\nAPI endpoint testing completed!")

if __name__ == "__main__":
    test_api_endpoints()