import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.agents.agent_service import AgentService

def test_agent_service():
    print("Testing Agent Service...")

    try:
        # Create agent service instance
        agent_service = AgentService()

        print("Agent service created successfully")

        # Test the process_query method directly
        query_text = "What is humanoid robotics?"
        retrieval_params = {
            "top_k": 2,
            "threshold": 0.7
        }

        print(f"Processing query: {query_text}")

        # This should trigger both retrieval and generation
        result = agent_service.process_query(
            query_text=query_text,
            retrieval_params=retrieval_params
        )

        print(f"Query processed successfully")
        print(f"Answer: {result.answer}")
        print(f"Sources found: {len(result.sources)}")
        print(f"Confidence score: {result.confidence_score}")
        print(f"Validation notes: {result.validation_notes}")

    except Exception as e:
        print(f"Error in agent service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_agent_service()