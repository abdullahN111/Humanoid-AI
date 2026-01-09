import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.config.settings import settings

def test_ingestion_with_debugging():
    print("Testing ingestion with debugging...")
    try:
        # Create pipeline
        pipeline = IngestionPipeline()
        print("Pipeline created successfully")

        # Parse sitemap to see what URLs are available
        from src.ingestion.crawler import parse_sitemap
        urls = parse_sitemap(settings.SITEMAP_URL)
        print(f"Found {len(urls)} URLs in sitemap")

        # Take just the first URL to test
        test_urls = urls[:1]  # Just test with one URL
        print(f"Testing with URL: {test_urls[0]}")

        # Manually process the batch with just one URL to see where it fails
        successful_count = 0
        failed_count = 0

        for url in test_urls:
            try:
                print(f"Processing URL: {url}")

                # Fetch page content
                from src.ingestion.crawler import fetch_page_content_with_retry
                html_content = fetch_page_content_with_retry(url)
                print(f"  Fetched HTML content: {len(html_content)} chars")

                # Extract content and title
                from src.ingestion.extractor import extract_content_from_html, extract_page_title
                content = extract_content_from_html(html_content, url)
                page_title = extract_page_title(html_content)
                print(f"  Extracted content: {len(content)} chars, title: {page_title}")

                # Process content into chunks
                from src.ingestion.processor import process_content
                content_chunks = process_content(content, url, page_title)
                print(f"  Created {len(content_chunks)} content chunks")

                if content_chunks:
                    print(f"  First chunk preview: {content_chunks[0].content[:100]}...")

                # At this point, the content processing is working
                # The embedding generation will fail due to rate limiting, which is expected
                print(f"  Content processing successful for {url}")
                successful_count += len(content_chunks) if content_chunks else 0

            except Exception as e:
                print(f"  Error processing {url}: {e}")
                import traceback
                traceback.print_exc()
                failed_count += 1

        print(f"Ingestion test completed. Successful: {successful_count}, Failed: {failed_count}")

    except Exception as e:
        print(f"Error during ingestion test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ingestion_with_debugging()