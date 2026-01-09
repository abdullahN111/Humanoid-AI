from src.api.main import app
import uvicorn

def main():
    print("Starting backend server...")
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="debug"  # Enable debug logging
    )


if __name__ == "__main__":
    main()
