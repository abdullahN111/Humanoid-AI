from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Optional
import logging
from src.config.settings import settings

logger = logging.getLogger(__name__)

class QdrantConnection:
    """Qdrant connection manager for validation purposes."""

    def __init__(self):
        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True  # Use gRPC for better performance if available
        )

        self.collection_name = "book_content_chunks"

    def get_client(self) -> QdrantClient:
        """Return the Qdrant client instance."""
        return self.client

    def get_collection_name(self) -> str:
        """Return the collection name."""
        return self.collection_name

    def validate_connection(self) -> bool:
        """Validate that we can connect to Qdrant and access the collection."""
        try:
            # Try to get collection info to verify access
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Successfully connected to collection '{self.collection_name}'")
            logger.info(f"Collection points count: {collection_info.points_count}")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant connection: {e}")
            return False

    def test_search(self, query_vector: list, limit: int = 5) -> Optional[list]:
        """Test search functionality to ensure retrieval works."""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )
            logger.info(f"Test search successful, retrieved {len(results)} results")
            return results
        except Exception as e:
            logger.error(f"Test search failed: {e}")
            return None

def get_qdrant_connection() -> QdrantConnection:
    """Get a singleton instance of the Qdrant connection."""
    return QdrantConnection()