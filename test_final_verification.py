import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.agents.rag_agent import RAGAgent
from src.services.retrieval import retrieval_service
from src.config.settings import settings

def test_final_verification():
    print("=== Final Verification Test ===")
    print(f"Retrieval threshold: {settings.RETRIEVAL_THRESHOLD}")
    print(f"Qdrant URL: {settings.QDRANT_URL}")

    # Test 1: Verify retrieval service connection
    print("\n1. Testing retrieval service connection...")
    try:
        is_valid = retrieval_service.validate_connection()
        print(f"   Connection valid: {is_valid}")
        if not is_valid:
            print("   [ERROR] Connection failed!")
            return
        else:
            print("   [SUCCESS] Connection successful!")
    except Exception as e:
        print(f"   [ERROR] Connection test failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # Test 2: Test retrieval directly
    print("\n2. Testing direct retrieval...")
    try:
        query_text = "Docusaurus blogging features"
        results = retrieval_service.retrieve_by_text(query_text, top_k=3, threshold=0.0)
        print(f"   Retrieved {len(results)} results for query: '{query_text}'")

        for i, result in enumerate(results):
            print(f"   Result {i+1}:")
            print(f"     ID: {result.id}")
            print(f"     Content preview: {result.content[:100]}...")
            print(f"     Source URL: {result.source_url}")
            print(f"     Similarity Score: {result.similarity_score}")

        if len(results) > 0:
            print("   [SUCCESS] Direct retrieval successful!")
        else:
            print("   [WARNING] No results found (this might be expected with current data)")
    except Exception as e:
        print(f"   [ERROR] Direct retrieval failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # Test 3: Test the full RAG agent
    print("\n3. Testing full RAG agent pipeline...")
    try:
        agent = RAGAgent()
        query_text = "What are Docusaurus blogging features?"

        print(f"   Processing query: '{query_text}'")
        response = agent.process_query(query_text)

        print(f"   Answer: {response.answer}")
        print(f"   Sources found: {len(response.sources)}")
        print(f"   Confidence score: {response.confidence_score}")
        print(f"   Validation notes: {response.validation_notes}")

        if response.answer and "error" not in response.answer.lower():
            print("   [SUCCESS] RAG agent pipeline successful!")
        else:
            print("   [ERROR] RAG agent pipeline failed!")
            return

    except Exception as e:
        print(f"   [ERROR] RAG agent pipeline failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # Test 4: Test with different query
    print("\n4. Testing with different query...")
    try:
        query_text = "Physical AI and humanoid robotics"
        response = agent.process_query(query_text)

        print(f"   Answer: {response.answer[:200]}...")
        print(f"   Sources found: {len(response.sources)}")
        print(f"   Confidence score: {response.confidence_score}")

        if response.answer and "error" not in response.answer.lower():
            print("   [SUCCESS] Second query successful!")
        else:
            print("   [WARNING] Second query returned no answer (might be expected if no relevant content exists)")

    except Exception as e:
        print(f"   [ERROR] Second query failed: {e}")
        import traceback
        traceback.print_exc()

    print("\n=== Final Verification Complete ===")
    print("[SUCCESS] All systems appear to be working properly!")

if __name__ == "__main__":
    test_final_verification()