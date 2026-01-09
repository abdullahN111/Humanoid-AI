import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.storage.vector_db import get_qdrant_storage

def test_direct_retrieval():
    print("Testing direct retrieval from Qdrant...")
    try:
        # Check how many points are in the database
        storage = get_qdrant_storage()
        points_count = storage.client.count(storage.collection_name)
        print(f"Points in Qdrant: {points_count.count}")

        if points_count.count > 0:
            # Get a sample of points to see what's stored
            search_results = storage.client.scroll(
                collection_name=storage.collection_name,
                limit=2
            )
            points, next_page = search_results
            print(f"Retrieved {len(points)} sample points")

            for i, point in enumerate(points):
                print(f"\nPoint {i+1}:")
                print(f"  ID: {point.id}")
                print(f"  Content preview: {point.payload.get('content_preview', 'No preview')[:100]}...")
                print(f"  Source: {point.payload.get('source_url', 'No source')}")

        else:
            print("No points found in Qdrant collection")

    except Exception as e:
        print(f"Error in direct retrieval test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_direct_retrieval()