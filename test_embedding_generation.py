import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.embeddings.generator import get_embedding_generator

def test_embedding_generation():
    print("Testing embedding generation...")

    try:
        # Get embedding generator
        embedding_gen = get_embedding_generator()

        # Test embedding generation
        query_text = "Docusaurus blogging features"
        vector_embedding = embedding_gen.generate_embedding_for_chunk(query_text, "test_query")

        if vector_embedding:
            print(f"Successfully generated embedding with {len(vector_embedding.vector)} dimensions")
            print(f"Chunk ID: {vector_embedding.chunk_id}")
            print(f"Model: {vector_embedding.model_name}")
        else:
            print("Failed to generate embedding")

    except Exception as e:
        print(f"Error in embedding generation: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_embedding_generation()