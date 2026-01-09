from typing import List, Dict, Any
from src.validation.models import RetrievalQuery, RetrievedChunk
from datetime import datetime

class ValidationTestDataset:
    """Test dataset with known queries and expected results for validation."""

    def __init__(self):
        self.semantic_queries = self._create_semantic_test_queries()
        self.metadata_filter_queries = self._create_metadata_filter_test_queries()
        self.consistency_verification_queries = self._create_consistency_verification_queries()

    def _create_semantic_test_queries(self) -> List[RetrievalQuery]:
        """Create a dataset of semantic test queries with expected results."""
        queries = []

        # Test query for humanoid robotics concepts
        query1 = RetrievalQuery(
            query_text="What are the key components of humanoid robotics?",
            test_scenario="Humanoid robotics fundamentals",
            expected_results=[
                {
                    "content_keywords": ["actuators", "sensors", "control systems", "kinematics"],
                    "source_url": "https://humanoidai.vercel.app/docs/components"
                }
            ]
        )
        queries.append(query1)

        # Test query for control systems
        query2 = RetrievalQuery(
            query_text="Explain the control systems for humanoid robots",
            test_scenario="Control systems explanation",
            expected_results=[
                {
                    "content_keywords": ["feedback", "PID controller", "motion planning", "balance control"],
                    "source_url": "https://humanoidai.vercel.app/docs/control-systems"
                }
            ]
        )
        queries.append(query2)

        # Test query for AI and machine learning in robotics
        query3 = RetrievalQuery(
            query_text="How is AI used in humanoid robot decision making?",
            test_scenario="AI decision making",
            expected_results=[
                {
                    "content_keywords": ["neural networks", "reinforcement learning", "decision trees", "behavior trees"],
                    "source_url": "https://humanoidai.vercel.app/docs/ai-decision-making"
                }
            ]
        )
        queries.append(query3)

        # Test query for locomotion
        query4 = RetrievalQuery(
            query_text="What are the different types of locomotion in humanoid robots?",
            test_scenario="Locomotion types",
            expected_results=[
                {
                    "content_keywords": ["walking", "bipedal", "gait", "stability", "balance"],
                    "source_url": "https://humanoidai.vercel.app/docs/locomotion"
                }
            ]
        )
        queries.append(query4)

        # General knowledge query
        query5 = RetrievalQuery(
            query_text="What is the history of humanoid robotics?",
            test_scenario="History overview",
            expected_results=[
                {
                    "content_keywords": ["history", "development", "timeline", "evolution", "early robots"],
                    "source_url": "https://humanoidai.vercel.app/docs/history"
                }
            ]
        )
        queries.append(query5)

        return queries

    def _create_metadata_filter_test_queries(self) -> List[RetrievalQuery]:
        """Create a dataset of queries with metadata filters."""
        queries = []

        # Query with URL filter
        query1 = RetrievalQuery(
            query_text="What are the specifications of the robot actuators?",
            test_scenario="URL-filtered query",
            metadata_filters={
                "source_url": "https://humanoidai.vercel.app/docs/actuator-specs"
            },
            expected_results=[
                {
                    "content_keywords": ["actuator", "specifications", "torque", "speed", "power consumption"],
                    "source_url": "https://humanoidai.vercel.app/docs/actuator-specs"
                }
            ]
        )
        queries.append(query1)

        # Query with section filter
        query2 = RetrievalQuery(
            query_text="How does the robot perceive its environment?",
            test_scenario="Section-filtered query",
            metadata_filters={
                "section": "sensing-perception"
            },
            expected_results=[
                {
                    "content_keywords": ["sensors", "perception", "cameras", "lidar", "computer vision"],
                    "section": "sensing-perception"
                }
            ]
        )
        queries.append(query2)

        # Query with multiple filters
        query3 = RetrievalQuery(
            query_text="What safety mechanisms are implemented?",
            test_scenario="Multiple filter query",
            metadata_filters={
                "source_url": "https://humanoidai.vercel.app/docs/safety",
                "section": "safety-protocols"
            },
            expected_results=[
                {
                    "content_keywords": ["safety", "emergency stop", "collision detection", "force limiting"],
                    "source_url": "https://humanoidai.vercel.app/docs/safety"
                }
            ]
        )
        queries.append(query3)

        return queries

    def _create_consistency_verification_queries(self) -> List[RetrievalQuery]:
        """Create a dataset of queries for content consistency verification."""
        queries = []

        # Query to test content consistency
        query1 = RetrievalQuery(
            query_text="Provide the complete text about bipedal walking mechanics",
            test_scenario="Content consistency verification",
            expected_results=[
                {
                    "exact_content": "Bipedal walking mechanics involve complex control systems that coordinate multiple joints to achieve stable locomotion. The center of mass must be maintained within the support polygon formed by the feet during walking. Key factors include step timing, foot placement, and balance control.",
                    "source_url": "https://humanoidai.vercel.app/docs/walking-mechanics"
                }
            ]
        )
        queries.append(query1)

        # Query to test source reference verification
        query2 = RetrievalQuery(
            query_text="What are the ethical considerations in humanoid robotics?",
            test_scenario="Source reference verification",
            expected_results=[
                {
                    "content_keywords": ["ethics", "human-robot interaction", "autonomy", "responsibility"],
                    "source_url": "https://humanoidai.vercel.app/docs/ethics",
                    "expected_title": "Ethical Considerations in Humanoid Robotics"
                }
            ]
        )
        queries.append(query2)

        return queries

    def get_all_queries(self) -> List[RetrievalQuery]:
        """Get all queries from all test scenarios."""
        all_queries = []
        all_queries.extend(self.semantic_queries)
        all_queries.extend(self.metadata_filter_queries)
        all_queries.extend(self.consistency_verification_queries)
        return all_queries

    def get_semantic_queries(self) -> List[RetrievalQuery]:
        """Get semantic test queries only."""
        return self.semantic_queries

    def get_metadata_filter_queries(self) -> List[RetrievalQuery]:
        """Get metadata filter test queries only."""
        return self.metadata_filter_queries

    def get_consistency_verification_queries(self) -> List[RetrievalQuery]:
        """Get consistency verification test queries only."""
        return self.consistency_verification_queries

    def get_query_by_id(self, query_id: str) -> RetrievalQuery:
        """Get a specific query by its ID."""
        all_queries = self.get_all_queries()
        for query in all_queries:
            if query.id == query_id:
                return query
        raise ValueError(f"Query with ID {query_id} not found")


def get_test_dataset() -> ValidationTestDataset:
    """Get a singleton instance of the test dataset."""
    return ValidationTestDataset()