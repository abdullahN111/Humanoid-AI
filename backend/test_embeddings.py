import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.embeddings.generator import get_embedding_generator
from src.storage.vector_db import get_qdrant_storage

def test_embedding_generation():
    print("Testing embedding generation...")
    try:
        # Test embedding generator
        embedding_gen = get_embedding_generator()
        print("Embedding generator created successfully")

        # Test with a simple text
        test_text = "This is a test sentence for embedding generation."
        print(f"Generating embedding for: '{test_text}'")

        vector_embedding = embedding_gen.generate_embedding_for_chunk(test_text, "test_chunk_1")
        if vector_embedding:
            print(f"Generated embedding: {len(vector_embedding.vector)} dimensions")
            print(f"Chunk ID: {vector_embedding.chunk_id}")
            print(f"Model: {vector_embedding.model_name}")
        else:
            print("Failed to generate embedding")

        # Test Qdrant storage
        storage = get_qdrant_storage()
        print("Qdrant storage created successfully")

        # Test if collection exists
        storage.ensure_collection_exists()
        print("Collection exists check passed")

    except Exception as e:
        print(f"Error in embedding generation test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_embedding_generation()