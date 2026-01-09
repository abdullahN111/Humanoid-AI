import sys
import os
import asyncio

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.agents.agent_service import AgentService

async def test_agent_service_with_validation():
    print("Testing agent service with validation...")

    try:
        # Create agent service instance
        agent_service = AgentService()

        print("Agent service created successfully")

        # Test with a query that should match our known content, with low threshold
        query_text = "What are Docusaurus blogging features?"
        retrieval_params = {
            "top_k": 2,
            "threshold": -0.1  # Very low threshold to allow negative similarity scores
        }

        print(f"Processing query: {query_text} with threshold {retrieval_params['threshold']}")

        # Test without grounding first
        result = await agent_service.process_query(
            query_text=query_text,
            retrieval_params=retrieval_params,
            grounding_required=False  # Disable grounding validation
        )

        print(f"Query processed successfully WITHOUT grounding")
        print(f"Answer: {result.answer}")
        print(f"Sources found: {len(result.sources)}")
        if result.sources:
            for i, source in enumerate(result.sources):
                print(f"  Source {i+1}: {source.content[:100]}... from {source.source_url}")
                print(f"    Score: {source.similarity_score}")
        print(f"Confidence score: {result.confidence_score}")
        print(f"Validation notes: {result.validation_notes}")

        print("\n--- Now testing WITH grounding ---")

        # Test with grounding enabled (default)
        result2 = await agent_service.process_query(
            query_text=query_text,
            retrieval_params=retrieval_params,
            grounding_required=True  # Enable grounding validation (default)
        )

        print(f"Query processed successfully WITH grounding")
        print(f"Answer: {result2.answer}")
        print(f"Sources found: {len(result2.sources)}")
        if result2.sources:
            for i, source in enumerate(result2.sources):
                print(f"  Source {i+1}: {source.content[:100]}... from {source.source_url}")
                print(f"    Score: {source.similarity_score}")
        print(f"Confidence score: {result2.confidence_score}")
        print(f"Validation notes: {result2.validation_notes}")

    except Exception as e:
        print(f"Error in agent service: {e}")
        import traceback
        traceback.print_exc()

def main():
    asyncio.run(test_agent_service_with_validation())

if __name__ == "__main__":
    main()