import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.api.main import app
import uvicorn

if __name__ == "__main__":
    print("Starting backend server on port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=False)