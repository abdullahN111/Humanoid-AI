#!/usr/bin/env python3
"""
Test script to run the ingestion pipeline and debug issues.
"""
import sys
import os

# Add the backend directory to the Python path
backend_path = os.path.join(os.path.dirname(__file__), 'backend')
sys.path.insert(0, backend_path)

# Add backend to the Python path for relative imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.ingestion.pipeline import validate_and_run_ingestion

def main():
    print("Starting ingestion test...")
    try:
        result = validate_and_run_ingestion()
        print(f"Ingestion result: {result}")
    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()