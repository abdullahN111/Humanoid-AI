from typing import List, Dict, Any, Optional
from src.validation.connection import get_qdrant_connection
from src.validation.models import RetrievedChunk
from src.embeddings.client import get_cohere_client
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)

class SimilaritySearch:
    """Similarity search functionality for validation purposes."""

    def __init__(self):
        self.qdrant_conn = get_qdrant_connection()
        self.client = self.qdrant_conn.get_client()
        self.collection_name = self.qdrant_conn.get_collection_name()
        self.embedding_client = get_cohere_client()

    def search_by_text(self, query_text: str, limit: int = 5, metadata_filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Perform semantic search using text query.

        Args:
            query_text: Text to search for similar content
            limit: Maximum number of results to return
            metadata_filters: Optional filters to apply during search

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Generate embedding for the query text
            query_embedding = self.embedding_client.generate_embedding(query_text)

            # Prepare filters for Qdrant
            qdrant_filters = self._prepare_metadata_filters(metadata_filters)

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=qdrant_filters,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for i, result in enumerate(search_results):
                chunk = self._convert_to_retrieved_chunk(result, i + 1)  # Rank starts from 1
                retrieved_chunks.append(chunk)

            logger.info(f"Search for '{query_text[:50]}...' returned {len(retrieved_chunks)} results")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error performing similarity search: {e}")
            return []

    def search_by_embedding(self, query_embedding: List[float], limit: int = 5, metadata_filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Perform semantic search using a pre-computed embedding.

        Args:
            query_embedding: Pre-computed embedding vector to search for similar content
            limit: Maximum number of results to return
            metadata_filters: Optional filters to apply during search

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Prepare filters for Qdrant
            qdrant_filters = self._prepare_metadata_filters(metadata_filters)

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=qdrant_filters,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for i, result in enumerate(search_results):
                chunk = self._convert_to_retrieved_chunk(result, i + 1)  # Rank starts from 1
                retrieved_chunks.append(chunk)

            logger.info(f"Search by embedding returned {len(retrieved_chunks)} results")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error performing similarity search by embedding: {e}")
            return []

    def _prepare_metadata_filters(self, metadata_filters: Optional[Dict[str, Any]]) -> Optional[Any]:
        """Prepare metadata filters for Qdrant search."""
        if not metadata_filters:
            return None

        # Convert metadata filters to Qdrant filter format
        filter_conditions = []
        for key, value in metadata_filters.items():
            if isinstance(value, str):
                # String match condition
                filter_conditions.append({
                    "key": key,
                    "match": {"value": value}
                })
            elif isinstance(value, list):
                # Match any of the values in the list
                filter_conditions.append({
                    "key": key,
                    "is_any": {"any": value}
                })
            elif isinstance(value, int):
                # Integer match condition
                filter_conditions.append({
                    "key": key,
                    "match": {"value": value}
                })
            elif isinstance(value, float):
                # Range condition for floats (if needed)
                filter_conditions.append({
                    "key": key,
                    "range": {"gte": value - 0.001, "lte": value + 0.001}  # Small tolerance for floats
                })
            elif isinstance(value, dict):
                # Nested object filters (if needed)
                # For now, just add as a match condition
                filter_conditions.append({
                    "key": key,
                    "match": {"value": str(value)}
                })
            elif isinstance(value, bool):
                # Boolean match condition
                filter_conditions.append({
                    "key": key,
                    "match": {"value": value}
                })

        if filter_conditions:
            return {
                "must": filter_conditions
            }

        return None

    def _convert_to_retrieved_chunk(self, search_result: Any, rank: int) -> RetrievedChunk:
        """Convert a Qdrant search result to a RetrievedChunk object."""
        payload = search_result.payload

        chunk = RetrievedChunk(
            id=search_result.id,
            content=payload.get('content', ''),
            source_url=payload.get('source_url', ''),
            page_title=payload.get('page_title', ''),
            similarity_score=search_result.score,
            chunk_order=payload.get('chunk_order', 0),
            retrieval_rank=rank,
            metadata=payload
        )

        return chunk

    def validate_search_functionality(self) -> bool:
        """Validate that the search functionality is working properly."""
        try:
            # Perform a test search with a generic query
            results = self.search_by_text("test", limit=1)

            if results:
                logger.info("Search functionality validation passed")
                return True
            else:
                logger.warning("Search functionality returned no results for test query")
                return False
        except Exception as e:
            logger.error(f"Search functionality validation failed: {e}")
            return False


def get_similarity_search() -> SimilaritySearch:
    """Get a singleton instance of the similarity search."""
    return SimilaritySearch()