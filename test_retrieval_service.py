import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.services.retrieval import retrieval_service

def test_retrieval_service():
    print("Testing retrieval service directly...")

    try:
        # Test connection validation
        is_valid = retrieval_service.validate_connection()
        print(f"Retrieval service connection valid: {is_valid}")

        # Test retrieval
        query_text = "Docusaurus blogging features"
        results = retrieval_service.retrieve_by_text(query_text, top_k=2, threshold=0.5)

        print(f"Retrieved {len(results)} results for query: '{query_text}'")

        for i, result in enumerate(results):
            print(f"\nResult {i+1}:")
            print(f"  ID: {result.id}")
            print(f"  Content: {result.content[:100]}...")
            print(f"  Source URL: {result.source_url}")
            print(f"  Page Title: {result.page_title}")
            print(f"  Similarity Score: {result.similarity_score}")
            print(f"  Retrieval Rank: {result.retrieval_rank}")

    except Exception as e:
        print(f"Error in retrieval service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_retrieval_service()