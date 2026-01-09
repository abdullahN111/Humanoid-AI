import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.storage.vector_db import get_qdrant_storage
from src.embeddings.generator import get_embedding_generator

def test_direct_search():
    print("Testing direct search in Qdrant...")

    try:
        # Get storage instance
        storage = get_qdrant_storage()

        # Get embedding generator
        embedding_gen = get_embedding_generator()

        # Create embedding for a search query
        query_text = "Docusaurus blogging features"
        query_embedding = embedding_gen.generate_embedding_for_chunk(query_text, "query")

        if query_embedding:
            print(f"Generated query embedding with {len(query_embedding.vector)} dimensions")

            # Perform search with a lower threshold
            results = storage.search_similar(query_embedding.vector, limit=5)
            print(f"Found {len(results)} results for query: '{query_text}'")

            for i, result in enumerate(results):
                print(f"\nResult {i+1}:")
                print(f"  ID: {result['id']}")
                print(f"  Score: {result['score']}")
                print(f"  Payload keys: {list(result['payload'].keys())}")
                print(f"  Content preview: {result['payload'].get('content_preview', 'No preview')[:150]}...")
                print(f"  Source: {result['payload'].get('source_url', 'No source')}")
        else:
            print("Could not generate query embedding")

    except Exception as e:
        print(f"Error in direct search: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_direct_search()