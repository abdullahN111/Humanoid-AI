import requests
import xml.etree.ElementTree as ET
from typing import List
import logging
from urllib.parse import urljoin, urlparse
from src.config.settings import settings

logger = logging.getLogger(__name__)

def parse_sitemap(sitemap_url: str = None) -> List[str]:
    """
    Parse a sitemap.xml file and return a list of URLs.

    Args:
        sitemap_url: URL of the sitemap.xml file. If None, uses the default from settings.

    Returns:
        List of URLs found in the sitemap
    """
    if sitemap_url is None:
        sitemap_url = settings.SITEMAP_URL

    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Handle both regular sitemaps and sitemap indexes
        urls = []
        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Check if this is a sitemap index (contains sitemap elements)
        sitemap_elements = root.findall('sitemap:sitemap', namespace)

        if sitemap_elements:
            # This is a sitemap index, need to fetch individual sitemaps
            for sitemap_elem in sitemap_elements:
                loc_elem = sitemap_elem.find('sitemap:loc', namespace)
                if loc_elem is not None:
                    individual_sitemap_url = loc_elem.text
                    urls.extend(parse_individual_sitemap(individual_sitemap_url))
        else:
            # This is a regular sitemap with URL elements
            urls = parse_individual_sitemap_from_content(response.content, namespace)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        raise
    except ET.ParseError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        raise

def parse_individual_sitemap(sitemap_url: str) -> List[str]:
    """Parse an individual sitemap (not an index)."""
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        return parse_individual_sitemap_from_content(response.content, namespace)
    except requests.RequestException as e:
        logger.error(f"Error fetching individual sitemap {sitemap_url}: {e}")
        return []

def parse_individual_sitemap_from_content(content, namespace) -> List[str]:
    """Parse sitemap content and extract URLs."""
    root = ET.fromstring(content)
    urls = []

    for url_elem in root.findall('sitemap:url', namespace):
        loc_elem = url_elem.find('sitemap:loc', namespace)
        if loc_elem is not None:
            # Fix placeholder URLs by replacing with actual domain
            original_url = loc_elem.text
            fixed_url = original_url.replace(
                "https://your-docusaurus-site.example.com",
                settings.BOOK_SITE_URL.rstrip('/')
            )
            urls.append(fixed_url)

    return urls

def fetch_page_content(url: str) -> str:
    """
    Fetch content from a given URL.

    Args:
        url: URL to fetch content from

    Returns:
        Content of the page as a string
    """
    try:
        response = requests.get(url, timeout=30)
        response.raise_for_status()
        return response.text
    except requests.RequestException as e:
        logger.error(f"Error fetching page {url}: {e}")
        raise

def fetch_page_content_with_retry(url: str, max_retries: int = 3) -> str:
    """
    Fetch content from a given URL with retry logic.

    Args:
        url: URL to fetch content from
        max_retries: Maximum number of retry attempts

    Returns:
        Content of the page as a string
    """
    last_exception = None

    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            return response.text
        except requests.HTTPError as e:
            logger.warning(f"HTTP error fetching page {url} (attempt {attempt + 1}): {e}")
            if e.response.status_code in [404, 410]:  # Not found or gone
                logger.error(f"Page not found: {url}")
                break  # Don't retry if page doesn't exist
            last_exception = e
        except requests.ConnectionError as e:
            logger.warning(f"Connection error fetching page {url} (attempt {attempt + 1}): {e}")
            last_exception = e
        except requests.Timeout as e:
            logger.warning(f"Timeout fetching page {url} (attempt {attempt + 1}): {e}")
            last_exception = e
        except requests.RequestException as e:
            logger.warning(f"Request error fetching page {url} (attempt {attempt + 1}): {e}")
            last_exception = e

        # Wait before retrying (exponential backoff)
        if attempt < max_retries - 1:
            import time
            wait_time = 2 ** attempt  # Exponential backoff
            logger.info(f"Waiting {wait_time}s before retrying {url}")
            time.sleep(wait_time)

    logger.error(f"Failed to fetch page {url} after {max_retries} attempts")
    raise last_exception if last_exception else requests.RequestException(f"Failed to fetch {url}")