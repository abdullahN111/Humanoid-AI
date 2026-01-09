import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.ingestion.crawler import parse_sitemap
from src.config.settings import settings

def test_sitemap_parsing():
    print("Testing sitemap parsing...")
    try:
        sitemap_url = settings.SITEMAP_URL
        print(f"Parsing sitemap: {sitemap_url}")

        urls = parse_sitemap(sitemap_url)
        print(f"Found {len(urls)} URLs in sitemap")

        # Just take the first URL to test
        if urls:
            test_url = urls[0]
            print(f"Test URL: {test_url}")
        else:
            print("No URLs found in sitemap")

    except Exception as e:
        print(f"Error parsing sitemap: {e}")
        import traceback
        traceback.print_exc()

def test_pipeline_creation():
    print("\nTesting pipeline creation...")
    try:
        pipeline = IngestionPipeline()
        print("Pipeline created successfully")

        # Test with a small batch size to avoid processing too much at once
        print(f"Processed URLs loaded: {len(pipeline.processed_urls)}")

    except Exception as e:
        print(f"Error creating pipeline: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_sitemap_parsing()
    test_pipeline_creation()