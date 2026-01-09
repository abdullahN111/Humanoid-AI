import cohere
from typing import List, Dict, Any
import logging
import time
from src.config.settings import settings

logger = logging.getLogger(__name__)

class CohereClient:
    """Cohere API client for generating embeddings."""

    def __init__(self, max_retries: int = 3, retry_delay: float = 1.0):
        if not settings.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = cohere.Client(settings.COHERE_API_KEY)
        self.model = settings.EMBEDDING_MODEL
        self.max_retries = max_retries
        self.retry_delay = retry_delay

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts with retry logic.

        Args:
            texts: List of texts to embed
            input_type: Type of input (search_document, search_query, classification, etc.)

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        last_exception = None

        for attempt in range(self.max_retries):
            try:
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type=input_type
                )

                logger.info(f"Generated embeddings for {len(texts)} texts using model {self.model}")
                return [embedding for embedding in response.embeddings]

            except (cohere.UnauthorizedError, cohere.BadRequestError, cohere.TooManyRequestsError,
                    cohere.ForbiddenError, cohere.NotFoundError, cohere.InternalServerError,
                    cohere.GatewayTimeoutError, cohere.ServiceUnavailableError) as e:
                logger.warning(f"Cohere API error (attempt {attempt + 1}): {e}")
                last_exception = e
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay * (2 ** attempt))  # Exponential backoff
            except Exception as e:
                logger.error(f"Unexpected error generating embeddings: {e}")
                last_exception = e
                break  # Don't retry on unexpected errors

        logger.error(f"Failed to generate embeddings after {self.max_retries} attempts")
        raise last_exception if last_exception else Exception("Unknown error in embedding generation")

    def generate_embedding(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text with retry logic.

        Args:
            text: Text to embed
            input_type: Type of input (search_document, search_query, classification, etc.)

        Returns:
            Embedding as a list of floats
        """
        embeddings = self.generate_embeddings([text], input_type)
        return embeddings[0] if embeddings else []

def get_cohere_client() -> CohereClient:
    """Get a singleton instance of the Cohere client."""
    return CohereClient()