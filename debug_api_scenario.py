import sys
import os
import asyncio
import uuid
from datetime import datetime

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.agents.agent_service import AgentService
from src.models.agent import Query, RetrievedContent, AgentResponse
from src.models.api import APIRequest, APIResponse
from src.utils.helpers import format_sources_for_response

async def debug_api_scenario():
    print("Debugging API scenario...")

    try:
        # Simulate the exact flow that happens in the API route
        query_id = str(uuid.uuid4())
        timestamp = datetime.utcnow().isoformat()

        # Create the exact request object that would come from the API
        request = {
            "query": "What are Docusaurus blogging features?",
            "retrieval_params": {
                "top_k": 2,
                "threshold": -0.1
            },
            "grounding_required": True
        }

        print(f"Processing query: {request['query']}")

        # Initialize the agent service (same as API route)
        agent_service = AgentService()

        # Process the query using the agent (same as API route)
        result = await agent_service.process_query(
            query_text=request["query"],
            retrieval_params=request["retrieval_params"],
            grounding_required=request["grounding_required"]
        )

        print(f"Result from agent service:")
        print(f"  Answer: {result.answer[:100]}...")
        print(f"  Sources: {len(result.sources)}")
        print(f"  Confidence: {result.confidence_score}")
        print(f"  Validation: {result.validation_notes}")

        # Convert result to response format (same as API route)
        sources = []
        if hasattr(result, 'sources') and result.sources:
            for source in result.sources:
                sources.append({
                    "id": source.id,
                    "content": source.content[:200] + "..." if len(source.content) > 200 else source.content,  # Truncate for response
                    "source_url": source.source_url,
                    "page_title": source.page_title,
                    "similarity_score": source.similarity_score,
                    "chunk_order": source.chunk_order,
                    "retrieval_rank": source.retrieval_rank
                })

        api_response = APIResponse(
            success=True,
            answer=result.answer if hasattr(result, 'answer') and result.answer else "No answer generated",
            sources=sources,
            timestamp=timestamp,
            query_id=query_id
        )

        print(f"API Response created successfully!")
        print(f"  Success: {api_response.success}")
        print(f"  Answer: {api_response.answer[:100]}...")
        print(f"  Sources count: {len(api_response.sources)}")

    except Exception as e:
        print(f"Error in debug scenario: {e}")
        import traceback
        traceback.print_exc()

def main():
    asyncio.run(debug_api_scenario())

if __name__ == "__main__":
    main()