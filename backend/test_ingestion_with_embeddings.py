import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.config.settings import settings

def test_ingestion_with_embeddings():
    print("Testing ingestion with embeddings...")
    try:
        pipeline = IngestionPipeline()

        # Override batch size to just process one URL
        original_batch_size = settings.INGESTION_BATCH_SIZE
        settings.INGESTION_BATCH_SIZE = 1  # Process only 1 URL at a time

        print(f"Settings: SITEMAP_URL={settings.SITEMAP_URL}")
        print(f"Batch size: {settings.INGESTION_BATCH_SIZE}")

        # Run ingestion for just the first URL
        result = pipeline.run_ingestion()
        print(f"Ingestion completed: {result}")

        # Restore original batch size
        settings.INGESTION_BATCH_SIZE = original_batch_size

    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ingestion_with_embeddings()