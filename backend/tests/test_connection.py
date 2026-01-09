"""
Unit tests for the connection module.

These tests verify the functionality of the Qdrant connection management components.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.validation.connection import QdrantConnection
from src.config.settings import settings


class TestConnection(unittest.TestCase):
    """Test cases for the connection module."""

    @patch('src.validation.connection.QdrantClient')
    def test_qdrant_connection_initialization(self, mock_qdrant_client):
        """Test Qdrant connection initialization."""
        # Mock the Qdrant client
        mock_client_instance = MagicMock()
        mock_qdrant_client.return_value = mock_client_instance

        # Set up settings with required values
        with patch('src.validation.connection.settings') as mock_settings:
            mock_settings.QDRANT_URL = "https://test.qdrant.tech"
            mock_settings.QDRANT_API_KEY = "test-api-key"

            # Create connection instance
            connection = QdrantConnection()

            # Verify initialization
            self.assertIsNotNone(connection)
            self.assertEqual(connection.get_collection_name(), "book_content_chunks")
            mock_qdrant_client.assert_called_once_with(
                url="https://test.qdrant.tech",
                api_key="test-api-key",
                prefer_grpc=True
            )

    @patch('src.validation.connection.QdrantClient')
    def test_qdrant_connection_missing_config(self, mock_qdrant_client):
        """Test Qdrant connection with missing configuration raises error."""
        # Set up settings with missing values
        with patch('src.validation.connection.settings') as mock_settings:
            mock_settings.QDRANT_URL = ""
            mock_settings.QDRANT_API_KEY = "test-api-key"

            # Expect ValueError for missing configuration
            with self.assertRaises(ValueError):
                QdrantConnection()

    @patch('src.validation.connection.QdrantClient')
    def test_get_client_method(self, mock_qdrant_client):
        """Test getting the Qdrant client instance."""
        # Mock the Qdrant client
        mock_client_instance = MagicMock()
        mock_qdrant_client.return_value = mock_client_instance

        # Set up settings with required values
        with patch('src.validation.connection.settings') as mock_settings:
            mock_settings.QDRANT_URL = "https://test.qdrant.tech"
            mock_settings.QDRANT_API_KEY = "test-api-key"

            # Create connection and get client
            connection = QdrantConnection()
            client = connection.get_client()

            # Verify the client is returned correctly
            self.assertEqual(client, mock_client_instance)

    @patch('src.validation.connection.QdrantClient')
    def test_validate_connection_success(self, mock_qdrant_client):
        """Test successful connection validation."""
        # Mock the Qdrant client and collection info
        mock_client_instance = MagicMock()
        mock_collection_info = MagicMock()
        mock_collection_info.points_count = 100
        mock_client_instance.get_collection.return_value = mock_collection_info
        mock_qdrant_client.return_value = mock_client_instance

        # Set up settings with required values
        with patch('src.validation.connection.settings') as mock_settings:
            mock_settings.QDRANT_URL = "https://test.qdrant.tech"
            mock_settings.QDRANT_API_KEY = "test-api-key"

            # Create connection and validate
            connection = QdrantConnection()
            result = connection.validate_connection()

            # Verify validation succeeded
            self.assertTrue(result)
            mock_client_instance.get_collection.assert_called_once_with("book_content_chunks")

    @patch('src.validation.connection.QdrantClient')
    def test_validate_connection_failure(self, mock_qdrant_client):
        """Test connection validation failure."""
        # Mock the Qdrant client to raise an exception
        mock_client_instance = MagicMock()
        mock_client_instance.get_collection.side_effect = Exception("Connection failed")
        mock_qdrant_client.return_value = mock_client_instance

        # Set up settings with required values
        with patch('src.validation.connection.settings') as mock_settings:
            mock_settings.QDRANT_URL = "https://test.qdrant.tech"
            mock_settings.QDRANT_API_KEY = "test-api-key"

            # Create connection and validate
            connection = QdrantConnection()
            result = connection.validate_connection()

            # Verify validation failed
            self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()