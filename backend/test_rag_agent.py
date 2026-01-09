import sys
import os

# Add the parent directory to Python path so we can import from src
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.agents.rag_agent import RAGAgent

def test_rag_agent():
    print("Testing RAG Agent with query...")

    try:
        # Create RAG agent instance
        agent = RAGAgent()
        print("RAG Agent created successfully")

        # Test query
        query_text = "What are Docusaurus blogging features?"

        # Use low threshold to accommodate negative similarity scores
        retrieval_params = {
            "top_k": 2,
            "threshold": -0.1  # Very low threshold to allow negative similarity scores
        }

        print(f"Processing query: {query_text}")

        # Process the query
        result = agent.process_query(query_text, retrieval_params)

        print(f"Query processed successfully")
        print(f"Answer: {result.answer}")
        print(f"Sources found: {len(result.sources)}")
        if result.sources:
            for i, source in enumerate(result.sources):
                print(f"  Source {i+1}: {source.content[:100]}... from {source.source_url}")
                print(f"    Score: {source.similarity_score}")
        print(f"Confidence score: {result.confidence_score}")
        print(f"Validation notes: {result.validation_notes}")

    except Exception as e:
        print(f"Error in RAG agent: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_rag_agent()