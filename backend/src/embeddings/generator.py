import logging
from typing import List, Optional
from src.embeddings.client import CohereClient, get_cohere_client
from src.storage.models import VectorEmbedding

logger = logging.getLogger(__name__)

class EmbeddingGenerator:
    """Generate vector embeddings from text content."""

    def __init__(self):
        self.client = get_cohere_client()

    def generate_embeddings_for_chunks(self, chunk_texts: List[str], chunk_ids: List[str]) -> List[VectorEmbedding]:
        """
        Generate embeddings for a list of content chunks.

        Args:
            chunk_texts: List of text content to embed
            chunk_ids: List of corresponding chunk IDs

        Returns:
            List of VectorEmbedding objects
        """
        if len(chunk_texts) != len(chunk_ids):
            raise ValueError("chunk_texts and chunk_ids must have the same length")

        # Generate embeddings for all texts at once
        embeddings = self.client.generate_embeddings(chunk_texts)

        # Create VectorEmbedding objects
        vector_embeddings = []
        for i, embedding_vector in enumerate(embeddings):
            vector_embedding = VectorEmbedding(
                chunk_id=chunk_ids[i],
                vector=embedding_vector,
                model_name=self.client.model,
                model_version="",  # This would be populated if the API provides it
                vector_size=len(embedding_vector)
            )

            # Validate the embedding
            validation_errors = vector_embedding.validate()
            if validation_errors:
                logger.error(f"Invalid VectorEmbedding: {validation_errors}")
                continue

            vector_embeddings.append(vector_embedding)

        logger.info(f"Generated {len(vector_embeddings)} vector embeddings")
        return vector_embeddings

    def generate_embedding_for_chunk(self, chunk_text: str, chunk_id: str) -> Optional[VectorEmbedding]:
        """
        Generate embedding for a single content chunk.

        Args:
            chunk_text: Text content to embed
            chunk_id: ID of the chunk

        Returns:
            VectorEmbedding object or None if generation fails
        """
        try:
            embedding_vector = self.client.generate_embedding(chunk_text)

            vector_embedding = VectorEmbedding(
                chunk_id=chunk_id,
                vector=embedding_vector,
                model_name=self.client.model,
                model_version="",  # This would be populated if the API provides it
                vector_size=len(embedding_vector)
            )

            # Validate the embedding
            validation_errors = vector_embedding.validate()
            if validation_errors:
                logger.error(f"Invalid VectorEmbedding: {validation_errors}")
                return None

            return vector_embedding

        except Exception as e:
            logger.error(f"Error generating embedding for chunk {chunk_id}: {e}")
            return None

def get_embedding_generator() -> EmbeddingGenerator:
    """Get a singleton instance of the embedding generator."""
    return EmbeddingGenerator()