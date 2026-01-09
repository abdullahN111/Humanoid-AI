import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.config.settings import settings

def test_ingestion():
    print("Testing ingestion pipeline...")
    print(f"Settings: SITEMAP_URL={settings.SITEMAP_URL}, QDRANT_URL={settings.QDRANT_URL}")

    try:
        pipeline = IngestionPipeline()
        print("Pipeline created successfully")

        # Test with a simple sitemap URL (or use the default)
        result = pipeline.run_ingestion()
        print(f"Ingestion completed: {result}")
    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ingestion()