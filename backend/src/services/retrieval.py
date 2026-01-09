from typing import List, Optional
from src.config.settings import settings
from src.models.agent import RetrievedContent
from src.storage.vector_db import get_qdrant_storage
from src.embeddings.generator import get_embedding_generator
import logging

logger = logging.getLogger(__name__)


class RetrievalService:
    """Service for retrieving relevant content from Qdrant vector database."""

    def __init__(self):
        """Initialize the retrieval service with Qdrant and embedding generator."""
        self.storage = get_qdrant_storage()
        self.embedding_generator = get_embedding_generator()

    def retrieve_by_text(self, query_text: str, top_k: int = 5, threshold: float = 0.7) -> List[RetrievedContent]:
        """
        Retrieve relevant chunks from Qdrant based on text query.

        Args:
            query_text: The text query to search for
            top_k: Number of top results to return
            threshold: Minimum similarity threshold for results

        Returns:
            List of RetrievedContent objects containing the relevant content
        """
        try:
            # Generate embedding for the query text using the embedding generator
            vector_embedding = self.embedding_generator.generate_embedding_for_chunk(query_text, "query")

            if vector_embedding is None:
                logger.error(f"Could not generate embedding for query: {query_text[:50]}...")
                return []

            query_embedding = vector_embedding.vector

            # Search in Qdrant using the storage service
            search_results = self.storage.search_similar(query_embedding, limit=top_k)

            # Filter results based on threshold
            filtered_results = [result for result in search_results if result['score'] >= threshold]

            # Convert search results to RetrievedContent objects
            retrieved_contents = []
            for idx, result in enumerate(filtered_results):
                payload = result['payload']
                content = RetrievedContent(
                    id=result['id'],
                    content=payload.get("content_preview", ""),  # Using content_preview since that's what we stored
                    source_url=payload.get("source_url", ""),
                    page_title=payload.get("page_title", ""),
                    similarity_score=result['score'],
                    chunk_order=payload.get("chunk_order", 0),
                    retrieval_rank=idx + 1
                )
                retrieved_contents.append(content)

            logger.info(f"Retrieved {len(retrieved_contents)} chunks for query: {query_text[:50]}...")
            return retrieved_contents

        except Exception as e:
            logger.error(f"Error retrieving chunks for query '{query_text}': {str(e)}", exc_info=True)  # Include full traceback
            return []

    def retrieve_by_embedding(self, query_embedding: List[float], top_k: int = 5, threshold: float = 0.7) -> List[RetrievedContent]:
        """
        Retrieve relevant chunks from Qdrant based on embedding vector.

        Args:
            query_embedding: The embedding vector to search for
            top_k: Number of top results to return
            threshold: Minimum similarity threshold for results

        Returns:
            List of RetrievedContent objects containing the relevant content
        """
        try:
            # Search in Qdrant using the storage service
            search_results = self.storage.search_similar(query_embedding, limit=top_k)

            # Filter results based on threshold
            filtered_results = [result for result in search_results if result['score'] >= threshold]

            # Convert search results to RetrievedContent objects
            retrieved_contents = []
            for idx, result in enumerate(filtered_results):
                payload = result['payload']
                content = RetrievedContent(
                    id=result['id'],
                    content=payload.get("content_preview", ""),  # Using content_preview since that's what we stored
                    source_url=payload.get("source_url", ""),
                    page_title=payload.get("page_title", ""),
                    similarity_score=result['score'],
                    chunk_order=payload.get("chunk_order", 0),
                    retrieval_rank=idx + 1
                )
                retrieved_contents.append(content)

            logger.info(f"Retrieved {len(retrieved_contents)} chunks using embedding")
            return retrieved_contents

        except Exception as e:
            logger.error(f"Error retrieving chunks by embedding: {str(e)}")
            return []

    def validate_connection(self) -> bool:
        """
        Validate the connection to Qdrant and check if collection exists.

        Returns:
            True if connection and collection are valid, False otherwise
        """
        try:
            # Use the storage's collection existence check
            self.storage.ensure_collection_exists()

            logger.info("Qdrant connection validated successfully")
            return True

        except Exception as e:
            logger.error(f"Error validating Qdrant connection: {str(e)}")
            return False


# Global instance of the retrieval service
retrieval_service = RetrievalService()