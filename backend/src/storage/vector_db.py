from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
import logging
from src.config.settings import settings
from src.storage.models import VectorEmbedding, SourceReference

logger = logging.getLogger(__name__)

class QdrantStorage:
    """Qdrant Cloud integration for vector storage."""

    def __init__(self):
        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True  # Use gRPC for better performance if available
        )

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.vector_size = 1024  # Default size for Cohere embeddings
        self.distance = models.Distance.COSINE

    def ensure_collection_exists(self):
        """Ensure the collection exists with the proper schema."""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with proper configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=self.distance
                    )
                )

                # Create payload indexes for efficient querying
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="source_url",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="content_hash",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="ingestion_timestamp",
                    field_schema=models.PayloadSchemaType.DATETIME
                )

                logger.info(f"Created collection '{self.collection_name}' with indexes")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists")

        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            raise

    def store_embeddings(self, embeddings: List[VectorEmbedding], source_references: List[SourceReference]) -> bool:
        """
        Store a list of embeddings in Qdrant with their metadata.

        Args:
            embeddings: List of VectorEmbedding objects to store
            source_references: List of SourceReference objects with metadata

        Returns:
            True if successful, False otherwise
        """
        try:
            # Prepare points for insertion
            points = []
            for i, embedding in enumerate(embeddings):
                # Find the corresponding source reference
                source_ref = None
                for ref in source_references:
                    if ref.chunk_id == embedding.chunk_id:
                        source_ref = ref
                        break

                if source_ref is None:
                    logger.warning(f"No source reference found for chunk {embedding.chunk_id}")
                    continue

                # Create payload with metadata
                payload = {
                    "chunk_id": embedding.chunk_id,
                    "source_url": source_ref.source_url,
                    "page_title": source_ref.page_title,
                    "content_hash": source_ref.chunk_id,  # Using chunk_id as content hash for now
                    "ingestion_timestamp": source_ref.created_at.isoformat() if source_ref.created_at else "",
                    "section_title": source_ref.section_title or "",
                    "content_preview": source_ref.content_preview[:200],  # Limit preview length
                    "model_name": embedding.model_name,
                    "model_version": embedding.model_version
                }

                # Create point for Qdrant
                point = models.PointStruct(
                    id=embedding.id,  # Use the embedding's ID as the point ID
                    vector=embedding.vector,
                    payload=payload
                )

                points.append(point)

            if not points:
                logger.warning("No points to store")
                return True

            # Upsert points into Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Stored {len(points)} embeddings in Qdrant collection '{self.collection_name}'")
            return True

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings using the query embedding.

        Args:
            query_embedding: Embedding vector to search for similarity
            limit: Maximum number of results to return

        Returns:
            List of similar content with metadata
        """
        try:
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True
            )

            results = []
            # query_points returns ScoredPoint objects
            for hit in search_results.points:
                result = {
                    "id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload if hit.payload else {}
                }
                results.append(result)

            logger.info(f"Found {len(results)} similar results")
            return results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {e}")
            return []

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution!).

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection '{self.collection_name}'")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            return False

def get_qdrant_storage() -> QdrantStorage:
    """Get a singleton instance of the Qdrant storage client."""
    return QdrantStorage()