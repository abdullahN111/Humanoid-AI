import sys
import os
import asyncio

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.agents.agent_service import AgentService

async def test_blog_features():
    print("Testing retrieval for blog features content...")

    try:
        # Create agent service instance
        agent_service = AgentService()

        # Test with a query that should match our known content
        query_text = "Docusaurus blogging features"
        retrieval_params = {
            "top_k": 2,
            "threshold": 0.5
        }

        print(f"Processing query: {query_text}")

        # This should find the content we stored earlier
        result = await agent_service.process_query(
            query_text=query_text,
            retrieval_params=retrieval_params
        )

        print(f"Query processed successfully")
        print(f"Answer: {result.answer}")
        print(f"Sources found: {len(result.sources)}")
        if result.sources:
            for i, source in enumerate(result.sources):
                print(f"  Source {i+1}: {source.content[:100]}... from {source.source_url}")
        print(f"Confidence score: {result.confidence_score}")
        print(f"Validation notes: {result.validation_notes}")

    except Exception as e:
        print(f"Error in agent service: {e}")
        import traceback
        traceback.print_exc()

def main():
    asyncio.run(test_blog_features())

if __name__ == "__main__":
    main()