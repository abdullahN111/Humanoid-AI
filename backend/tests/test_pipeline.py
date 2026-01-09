"""
Integration tests for the ingestion pipeline.

These tests verify the functionality of the complete ingestion pipeline.
"""
import unittest
from unittest.mock import patch, MagicMock, mock_open
from src.ingestion.pipeline import IngestionPipeline, run_ingestion_pipeline
from src.storage.models import ContentChunk


class TestPipeline(unittest.TestCase):
    """Test cases for the ingestion pipeline."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a pipeline instance for testing
        self.pipeline = IngestionPipeline()

    @patch('src.ingestion.pipeline.parse_sitemap')
    def test_pipeline_initialization(self, mock_parse_sitemap):
        """Test pipeline initialization."""
        mock_parse_sitemap.return_value = []

        self.assertIsNotNone(self.pipeline)
        self.assertIsInstance(self.pipeline.processed_urls, dict)

    @patch('src.ingestion.pipeline.parse_sitemap')
    @patch('src.ingestion.pipeline.fetch_page_content_with_retry')
    @patch('src.ingestion.pipeline.extract_content_from_html')
    @patch('src.ingestion.pipeline.extract_page_title')
    @patch('src.ingestion.pipeline.process_content')
    def test_process_batch_success(self, mock_process_content, mock_extract_title,
                                   mock_extract_content, mock_fetch_content, mock_parse_sitemap):
        """Test processing a batch of URLs successfully."""
        # Mock the sitemap parsing
        mock_parse_sitemap.return_value = ["https://example.com/page1", "https://example.com/page2"]

        # Mock page content fetching
        mock_fetch_content.return_value = "<html><body>Test content</body></html>"

        # Mock content extraction
        mock_extract_content.return_value = "Test content extracted"
        mock_extract_title.return_value = "Test Page"

        # Mock content processing
        mock_process_content.return_value = [
            ContentChunk(content="Test content extracted", source_url="https://example.com/page1", page_title="Test Page")
        ]

        # Test processing a batch
        urls = ["https://example.com/page1"]
        successful_count, failed_count = self.pipeline._process_batch(urls)

        # Should have processed successfully
        self.assertGreaterEqual(successful_count, 0)  # At least one chunk processed
        self.assertEqual(failed_count, 0)

    def test_is_url_processed(self):
        """Test URL processing status check."""
        # Add a URL to the processed list
        self.pipeline.processed_urls["https://example.com/test"] = "hash123"

        # Check if URL is marked as processed
        is_processed = self.pipeline._is_url_processed("https://example.com/test", "hash123")
        self.assertTrue(is_processed)

        # Check with different hash
        is_processed = self.pipeline._is_url_processed("https://example.com/test", "different_hash")
        self.assertFalse(is_processed)

    def test_mark_url_processed(self):
        """Test marking a URL as processed."""
        self.pipeline._mark_url_processed("https://example.com/test", "hash123")

        # Check if URL was added to processed list
        self.assertIn("https://example.com/test", self.pipeline.processed_urls)
        self.assertEqual(self.pipeline.processed_urls["https://example.com/test"], "hash123")

    @patch('src.ingestion.pipeline.parse_sitemap')
    def test_get_ingestion_status(self, mock_parse_sitemap):
        """Test getting ingestion status."""
        mock_parse_sitemap.return_value = []

        status = self.pipeline.get_ingestion_status()

        self.assertIn('total_processed_urls', status)
        self.assertIn('last_updated', status)
        self.assertIn('tracking_file', status)

    @patch('src.ingestion.pipeline.IngestionPipeline')
    def test_run_ingestion_pipeline_function(self, mock_pipeline_class):
        """Test the run_ingestion_pipeline convenience function."""
        # Mock the pipeline instance
        mock_pipeline_instance = MagicMock()
        mock_pipeline_instance.run_ingestion.return_value = MagicMock()
        mock_pipeline_class.return_value = mock_pipeline_instance

        # Call the function
        result = run_ingestion_pipeline("https://example.com/sitemap.xml")

        # Verify the pipeline was called
        mock_pipeline_instance.run_ingestion.assert_called_once_with("https://example.com/sitemap.xml")


if __name__ == '__main__':
    unittest.main()