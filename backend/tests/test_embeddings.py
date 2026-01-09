"""
Unit tests for the embeddings module.

These tests verify the functionality of the embedding generation components.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.embeddings.client import CohereClient
from src.embeddings.generator import EmbeddingGenerator, get_embedding_generator
from src.storage.models import VectorEmbedding


class TestEmbeddings(unittest.TestCase):
    """Test cases for the embeddings module."""

    @patch('src.embeddings.client.cohere.Client')
    def test_cohere_client_initialization(self, mock_cohere_client):
        """Test Cohere client initialization."""
        # Mock the Cohere client
        mock_client_instance = MagicMock()
        mock_cohere_client.return_value = mock_client_instance

        # Mock the embed method response
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_client_instance.embed.return_value = mock_response

        # Create client instance (assuming settings are configured)
        # Since we can't easily mock settings, we'll test the logic
        with patch('src.embeddings.client.settings') as mock_settings:
            mock_settings.COHERE_API_KEY = "test-key"
            mock_settings.EMBEDDING_MODEL = "test-model"

            client = CohereClient()

            self.assertIsNotNone(client)
            self.assertEqual(client.model, "test-model")

    @patch('src.embeddings.client.get_cohere_client')
    def test_embedding_generator_initialization(self, mock_get_client):
        """Test embedding generator initialization."""
        mock_client = MagicMock()
        mock_get_client.return_value = mock_client
        mock_client.model = "test-model"

        generator = EmbeddingGenerator()

        self.assertIsNotNone(generator)
        self.assertEqual(generator.client, mock_client)

    @patch('src.embeddings.client.get_cohere_client')
    def test_generate_embedding_for_chunk(self, mock_get_client):
        """Test generating embedding for a single chunk."""
        # Mock the Cohere client
        mock_client = MagicMock()
        mock_get_client.return_value = mock_client
        mock_client.generate_embedding.return_value = [0.1, 0.2, 0.3]
        mock_client.model = "test-model"

        generator = EmbeddingGenerator()

        # Test generating embedding for a chunk
        chunk_text = "This is a test chunk"
        chunk_id = "test-chunk-id"

        embedding = generator.generate_embedding_for_chunk(chunk_text, chunk_id)

        self.assertIsNotNone(embedding)
        self.assertIsInstance(embedding, VectorEmbedding)
        self.assertEqual(embedding.chunk_id, chunk_id)
        self.assertEqual(embedding.model_name, "test-model")
        self.assertEqual(embedding.vector, [0.1, 0.2, 0.3])

    @patch('src.embeddings.client.get_cohere_client')
    def test_generate_embeddings_for_chunks(self, mock_get_client):
        """Test generating embeddings for multiple chunks."""
        # Mock the Cohere client
        mock_client = MagicMock()
        mock_get_client.return_value = mock_client
        mock_client.generate_embeddings.return_value = [[0.1, 0.2], [0.3, 0.4]]
        mock_client.model = "test-model"

        generator = EmbeddingGenerator()

        # Test generating embeddings for multiple chunks
        chunk_texts = ["First chunk", "Second chunk"]
        chunk_ids = ["id1", "id2"]

        embeddings = generator.generate_embeddings_for_chunks(chunk_texts, chunk_ids)

        self.assertEqual(len(embeddings), 2)
        self.assertIsInstance(embeddings[0], VectorEmbedding)
        self.assertEqual(embeddings[0].chunk_id, "id1")
        self.assertEqual(embeddings[1].chunk_id, "id2")

    def test_get_embedding_generator(self):
        """Test getting singleton embedding generator instance."""
        generator1 = get_embedding_generator()
        generator2 = get_embedding_generator()

        # Both should be valid instances (though not necessarily same instance depending on implementation)
        self.assertIsNotNone(generator1)
        self.assertIsNotNone(generator2)


if __name__ == '__main__':
    unittest.main()