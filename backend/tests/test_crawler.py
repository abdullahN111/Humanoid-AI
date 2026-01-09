"""
Unit tests for the crawler module.

These tests verify the functionality of the web crawling and sitemap parsing components.
"""
import unittest
from unittest.mock import patch, MagicMock
import requests
from src.ingestion.crawler import parse_sitemap, fetch_page_content, fetch_page_content_with_retry


class TestCrawler(unittest.TestCase):
    """Test cases for the crawler module."""

    @patch('src.ingestion.crawler.requests.get')
    def test_parse_sitemap_success(self, mock_get):
        """Test successful sitemap parsing."""
        # Mock a simple sitemap response
        mock_response = MagicMock()
        mock_response.content = b'''<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
    <url>
        <loc>https://example.com/page1</loc>
    </url>
    <url>
        <loc>https://example.com/page2</loc>
    </url>
</urlset>'''
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        urls = parse_sitemap("https://example.com/sitemap.xml")

        self.assertEqual(len(urls), 2)
        self.assertIn("https://example.com/page1", urls)
        self.assertIn("https://example.com/page2", urls)

    @patch('src.ingestion.crawler.requests.get')
    def test_fetch_page_content_success(self, mock_get):
        """Test successful page content fetching."""
        # Mock a successful page fetch
        mock_response = MagicMock()
        mock_response.text = "<html><body>Test content</body></html>"
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        content = fetch_page_content("https://example.com/test")

        self.assertEqual(content, "<html><body>Test content</body></html>")

    @patch('src.ingestion.crawler.requests.get')
    def test_fetch_page_content_with_retry_success(self, mock_get):
        """Test successful page content fetching with retry logic."""
        # Mock a successful page fetch on first attempt
        mock_response = MagicMock()
        mock_response.text = "<html><body>Test content</body></html>"
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        content = fetch_page_content_with_retry("https://example.com/test")

        self.assertEqual(content, "<html><body>Test content</body></html>")

    @patch('src.ingestion.crawler.requests.get')
    def test_fetch_page_content_with_retry_multiple_attempts(self, mock_get):
        """Test retry logic when requests fail initially."""
        # Mock multiple failures followed by success
        responses = [
            requests.RequestException("Connection error"),
            requests.RequestException("Timeout error"),
            MagicMock(text="<html><body>Success content</body></html>", status_code=200)
        ]

        def side_effect(*args, **kwargs):
            response = responses.pop(0)
            if isinstance(response, requests.RequestException):
                raise response
            else:
                response.raise_for_status.return_value = None
                return response

        mock_get.side_effect = side_effect

        content = fetch_page_content_with_retry("https://example.com/test", max_retries=3)

        self.assertEqual(content, "<html><body>Success content</body></html>")


if __name__ == '__main__':
    unittest.main()