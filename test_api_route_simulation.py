import sys
import os
import uuid
from datetime import datetime

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.agents.agent_service import AgentService
from src.models.api import APIResponse
from src.utils.helpers import format_sources_for_response

async def test_api_route_logic():
    print("Testing API route logic directly...")

    try:
        # Create agent service instance
        agent_service = AgentService()

        # Test with a query that should match our known content, with low threshold
        query_text = "What are Docusaurus blogging features?"
        retrieval_params = {
            "top_k": 2,
            "threshold": -0.1  # Very low threshold to allow negative similarity scores
        }

        print(f"Processing query: {query_text}")

        # Process the query using the agent
        result = await agent_service.process_query(
            query_text=query_text,
            retrieval_params=retrieval_params
        )

        print(f"Agent result processed successfully")
        print(f"Answer: {result.answer}")
        print(f"Has sources attribute: {hasattr(result, 'sources')}")
        print(f"Sources type: {type(result.sources)}")
        print(f"Sources length: {len(result.sources) if result.sources else 0}")

        # Now simulate what the API route does
        query_id = str(uuid.uuid4())
        timestamp = datetime.utcnow().isoformat()

        # Convert result to response format (this is what's in the API route)
        sources = []
        if hasattr(result, 'sources') and result.sources:
            for source in result.sources:
                print(f"Processing source: {type(source)}")
                print(f"Source attributes: {[attr for attr in dir(source) if not attr.startswith('_')]}")

                source_dict = {
                    "id": source.id,
                    "content": source.content[:200] + "..." if len(source.content) > 200 else source.content,  # Truncate for response
                    "source_url": source.source_url,
                    "page_title": source.page_title,
                    "similarity_score": source.similarity_score,
                    "chunk_order": source.chunk_order,
                    "retrieval_rank": source.retrieval_rank
                }
                sources.append(source_dict)

        print(f"Converted sources: {len(sources)}")

        api_response = APIResponse(
            success=True,
            answer=result.answer if hasattr(result, 'answer') and result.answer else "No answer generated",
            sources=sources,
            timestamp=timestamp,
            query_id=query_id
        )

        print(f"API Response created successfully: {api_response.success}")
        print(f"Answer: {api_response.answer[:200]}...")
        print(f"Sources count: {len(api_response.sources)}")

    except Exception as e:
        print(f"Error in API route simulation: {e}")
        import traceback
        traceback.print_exc()

import asyncio
asyncio.run(test_api_route_logic())