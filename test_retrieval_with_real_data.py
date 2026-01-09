import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import run_ingestion_pipeline
from src.storage.vector_db import get_qdrant_storage
from src.embeddings.generator import get_embedding_generator

def test_ingestion_with_real_embeddings():
    print("Testing ingestion with real embeddings (will use API)...")
    try:
        # First, let's check how many points are currently in the database
        storage = get_qdrant_storage()
        points_count = storage.client.count(storage.collection_name)
        print(f"Initial points in Qdrant: {points_count.count}")

        # Run a small ingestion to populate with real data
        print("Starting ingestion pipeline...")
        result = run_ingestion_pipeline()
        print(f"Ingestion result: {result}")

        # Check count after ingestion
        points_count_after = storage.client.count(storage.collection_name)
        print(f"Points in Qdrant after ingestion: {points_count_after.count}")

        # Test embedding generation to ensure API works
        print("Testing embedding generation...")
        embedding_gen = get_embedding_generator()
        test_text = "humanoid robotics"
        vector_embedding = embedding_gen.generate_embedding_for_chunk(test_text, "test_chunk")

        if vector_embedding:
            print(f"Successfully generated embedding: {len(vector_embedding.vector)} dimensions")
        else:
            print("Failed to generate embedding (possibly due to rate limiting)")

    except Exception as e:
        print(f"Error in ingestion test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ingestion_with_real_embeddings()