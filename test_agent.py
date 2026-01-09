"""
Basic test to verify the AI agent components work together.
"""
import asyncio
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.agent.orchestrator import AgentOrchestrator
from backend.src.models.validation_config import ValidationConfig
from backend.src.validation.utils import setup_logger


async def test_agent_components():
    """Test that the agent components work together properly."""
    logger = setup_logger("TestAgent")
    logger.info("Starting agent components test...")

    try:
        # Create orchestrator with default configuration
        config = ValidationConfig()
        orchestrator = AgentOrchestrator(config=config)

        # Test 1: Create a session
        logger.info("Test 1: Creating session...")
        session_id = await orchestrator.create_session()
        logger.info(f"✓ Session created: {session_id}")

        # Test 2: Process a simple query
        logger.info("Test 2: Processing query...")
        query = "What is humanoid robotics?"
        result = await orchestrator.process_query(
            query_text=query,
            session_id=session_id
        )

        logger.info(f"✓ Query processed successfully")
        logger.info(f"  - Relevance score: {result.relevance_score:.3f}")
        logger.info(f"  - Is valid: {result.is_valid}")
        logger.info(f"  - Retrieved chunks: {len(result.retrieved_chunks)}")
        logger.info(f"  - Response preview: {result.response_text[:100]}...")

        # Test 3: Check session state
        logger.info("Test 3: Checking session state...")
        session_info = await orchestrator.get_session(session_id)
        if session_info:
            logger.info(f"✓ Session info retrieved")
            logger.info(f"  - Conversation history length: {len(session_info['conversation_history'])}")

        # Test 4: Get conversation history
        logger.info("Test 4: Getting conversation history...")
        history = await orchestrator.get_session_conversation_history(session_id)
        logger.info(f"✓ Retrieved {len(history)} conversation entries")

        # Test 5: Close session
        logger.info("Test 5: Closing session...")
        success = await orchestrator.close_session(session_id)
        if success:
            logger.info("✓ Session closed successfully")
        else:
            logger.warning("⚠ Session not found when trying to close")

        logger.info("✓ All tests completed successfully!")
        return True

    except Exception as e:
        logger.error(f"✗ Test failed with error: {str(e)}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return False


async def test_batch_processing():
    """Test batch processing functionality."""
    logger = setup_logger("TestBatch")
    logger.info("Starting batch processing test...")

    try:
        # Create orchestrator with default configuration
        config = ValidationConfig()
        orchestrator = AgentOrchestrator(config=config)

        # Test batch processing
        queries = [
            "What is AI?",
            "Explain robotics",
            "What are neural networks?"
        ]

        logger.info(f"Processing {len(queries)} queries in batch...")
        # Create an agent core instance to test batch processing
        from backend.src.agent.core import AgentCore
        from backend.src.services.qdrant_service import QdrantService
        from backend.src.services.cohere_service import CohereService

        agent_core = AgentCore(
            qdrant_service=QdrantService(),
            cohere_service=CohereService(),
            config=config
        )
        results = await agent_core.process_batch_queries(queries)

        logger.info(f"✓ Batch processing completed")
        logger.info(f"  - Processed {len(results)} queries")

        for i, result in enumerate(results):
            logger.info(f"  - Query {i+1}: Score={result.relevance_score:.3f}, Valid={result.is_valid}")

        logger.info("✓ Batch processing test completed successfully!")
        return True

    except Exception as e:
        logger.error(f"✗ Batch test failed with error: {str(e)}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return False


async def main():
    """Run all tests."""
    logger = setup_logger("TestMain")
    logger.info("Starting AI Agent Integration Tests...")

    test_results = []

    # Run individual tests
    logger.info("\nRunning component test...")
    result1 = await test_agent_components()
    test_results.append(("Component Test", result1))

    logger.info("\nRunning batch processing test...")
    result2 = await test_batch_processing()
    test_results.append(("Batch Processing Test", result2))

    # Summary
    logger.info("\n" + "="*50)
    logger.info("TEST SUMMARY")
    logger.info("="*50)

    passed = 0
    for test_name, result in test_results:
        status = "PASS" if result else "FAIL"
        logger.info(f"{test_name}: {status}")
        if result:
            passed += 1

    logger.info(f"\nOverall: {passed}/{len(test_results)} tests passed")

    if passed == len(test_results):
        logger.info("🎉 All tests passed!")
        return True
    else:
        logger.error("❌ Some tests failed!")
        return False


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)