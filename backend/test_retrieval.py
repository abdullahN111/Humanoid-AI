import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.storage.vector_db import get_qdrant_storage
from src.embeddings.generator import get_embedding_generator

def test_retrieval():
    print("Testing retrieval functionality...")
    try:
        # Test if we have data in Qdrant
        storage = get_qdrant_storage()
        points_count = storage.client.count(storage.collection_name)
        print(f"Points in Qdrant: {points_count.count}")

        if points_count.count > 0:
            print("Testing search functionality...")

            # Try to generate a test embedding to search with
            embedding_gen = get_embedding_generator()
            test_query = "humanoid robotics"  # This is a relevant query for the book
            print(f"Generating embedding for search query: '{test_query}'")

            try:
                # Generate embedding for the query
                query_embedding = embedding_gen.generate_embedding_for_chunk(test_query, "test_query")
                if query_embedding:
                    # Perform a search
                    results = storage.search_similar(query_embedding.vector, limit=3)
                    print(f"Search results: {len(results)} items found")
                    for i, result in enumerate(results):
                        print(f"  Result {i+1}: Score={result['score']}")
                        print(f"    Content preview: {result['payload'].get('content_preview', 'No preview')[:100]}...")
                        print(f"    Source: {result['payload'].get('source_url', 'No source')}")
                        print(f"    Title: {result['payload'].get('page_title', 'No title')}")
                else:
                    print("Could not generate query embedding (likely due to rate limiting)")
                    # Try a direct search without using embeddings (for testing purposes)
                    print("Attempting direct search without embeddings...")
                    # Just get a few points to verify the data is there
                    search_results = storage.client.scroll(
                        collection_name=storage.collection_name,
                        limit=3
                    )
                    points, next_page = search_results
                    print(f"Direct retrieval found {len(points)} points in the collection")
                    for i, point in enumerate(points):
                        print(f"  Point {i+1}: ID={point.id}")
                        print(f"    Content preview: {point.payload.get('content_preview', 'No preview')[:100]}...")
                        print(f"    Source: {point.payload.get('source_url', 'No source')}")
            except Exception as e:
                print(f"Could not perform search (likely due to rate limiting): {e}")
                print("Attempting direct retrieval instead...")
                # Just get a few points to verify the data is there
                search_results = storage.client.scroll(
                    collection_name=storage.collection_name,
                    limit=3
                )
                points, next_page = search_results
                print(f"Direct retrieval found {len(points)} points in the collection")
                for i, point in enumerate(points):
                    print(f"  Point {i+1}: ID={point.id}")
                    print(f"    Content preview: {point.payload.get('content_preview', 'No preview')[:100]}...")
                    print(f"    Source: {point.payload.get('source_url', 'No source')}")
                    print(f"    Title: {point.payload.get('page_title', 'No title')}")
        else:
            print("No data in Qdrant yet - run ingestion first")

        # Test the retrieval service
        print("\nTesting retrieval service...")
        try:
            # Import the retrieval service
            from src.services.retrieval import retrieval_service

            # Test connection validation
            is_valid = retrieval_service.validate_connection()
            print(f"Retrieval service connection valid: {is_valid}")

            if is_valid:
                # Test with a simple query
                retrieved_content = retrieval_service.retrieve_by_text("humanoid robotics", top_k=2)
                print(f"Retrieved {len(retrieved_content)} content items using retrieval service")
                for i, item in enumerate(retrieved_content):
                    print(f"  Item {i+1}: Score={item.similarity_score}")
                    print(f"    Content: {item.content[:100]}...")
                    print(f"    Source: {item.source_url}")
                    print(f"    Title: {item.page_title}")
            else:
                print("Retrieval service connection is not valid - check collection name and model dimensions")
        except Exception as e:
            print(f"Error using retrieval service: {e}")
            import traceback
            traceback.print_exc()

    except Exception as e:
        print(f"Error in retrieval test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_retrieval()