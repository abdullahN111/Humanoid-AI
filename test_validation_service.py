import sys
import os
import asyncio

# Add the backend directory to Python path
sys.path.insert(0, 'backend')

from src.agents.agent_service import AgentService
from src.models.agent import RetrievedContent
from datetime import datetime
import uuid

async def test_validation_service():
    print("Testing validation service directly...")

    try:
        # Create agent service instance
        agent_service = AgentService()

        print("Agent service created successfully")

        # Create a mock response to test validation
        mock_sources = [
            RetrievedContent(
                id="test-id-1",
                content="Docusaurus blogging features are powered by the blog plugin.",
                source_url="https://humanoidai.vercel.app/blog",
                page_title="Blog | ROS 2 as a Robotic Nervous System",
                similarity_score=-0.03898558393120766,
                chunk_order=0,
                retrieval_rank=1
            )
        ]

        # Test the validation directly
        print("Testing comprehensive validation...")
        validation_result = agent_service._AgentService__rag_agent.validate_grounding(
            "Docusaurus blogging features are powered by the blog plugin.",
            mock_sources
        )
        print(f"Basic grounding validation result: {validation_result}")

        # Now test the comprehensive validation through the validation service
        from src.services.validation import validation_service

        mock_response = type('MockResponse', (), {})()
        mock_response.answer = "Docusaurus blogging features are powered by the blog plugin [Source 1]."
        mock_response.sources = mock_sources
        mock_response.timestamp = datetime.utcnow()

        print("Testing comprehensive validation through validation service...")
        comprehensive_result = validation_service.perform_comprehensive_validation(mock_response)
        print(f"Comprehensive validation result: {comprehensive_result}")

        print("Validation service test completed successfully!")

    except Exception as e:
        print(f"Error in validation service test: {e}")
        import traceback
        traceback.print_exc()

def main():
    asyncio.run(test_validation_service())

if __name__ == "__main__":
    main()