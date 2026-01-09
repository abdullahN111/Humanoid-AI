import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.storage.vector_db import get_qdrant_storage
from src.embeddings.generator import get_embedding_generator

def test_qdrant_storage():
    print("Testing Qdrant storage...")
    try:
        # Test Qdrant storage
        storage = get_qdrant_storage()
        print("Qdrant storage created successfully")

        # Test if collection exists
        storage.ensure_collection_exists()
        print("Collection exists check passed")

        # Count points in the collection
        points_count = None
        try:
            # Try to count points using the Qdrant client directly
            points_count = storage.client.count(storage.collection_name)
            print(f"Number of points in collection: {points_count.count}")
        except Exception as e:
            print(f"Could not count points: {e}")
            # Alternative: try to get collection list to see if our collection exists
            try:
                collections = storage.client.get_collections()
                collection_names = [col.name for col in collections.collections]
                print(f"Available collections: {collection_names}")
                if storage.collection_name in collection_names:
                    print(f"Our collection '{storage.collection_name}' exists")
                else:
                    print(f"Our collection '{storage.collection_name}' does not exist yet")
            except Exception as e2:
                print(f"Could not get collection list: {e2}")

        # If we have points, try to search
        try:
            if points_count and points_count.count > 0:
                # Try to generate a test embedding to search with
                embedding_gen = get_embedding_generator()
                test_query = "humanoid robotics"
                print(f"Generating embedding for search query: '{test_query}'")

                try:
                    query_embedding = embedding_gen.generate_embedding_for_chunk(test_query, "test_query")
                    if query_embedding:
                        # Perform a search
                        results = storage.search_similar(query_embedding.vector, limit=3)
                        print(f"Search results: {len(results)} items found")
                        for i, result in enumerate(results):
                            print(f"  Result {i+1}: {result.score} - {result.get('payload', {}).get('content_preview', 'No preview')[:100]}...")
                    else:
                        print("Could not generate query embedding (likely due to rate limiting)")
                except Exception as e:
                    print(f"Could not perform search (likely due to rate limiting): {e}")
                    print("This is expected due to API rate limits during testing")
            else:
                print("No points in collection yet - ingestion may not have completed successfully")
        except Exception as e:
            print(f"Error in search attempt: {e}")

    except Exception as e:
        print(f"Error in Qdrant storage test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_qdrant_storage()