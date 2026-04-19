"""
Crawler module for the ingestion pipeline
"""

import time
import urllib.parse
import urllib.robotparser
from typing import Dict, List, Set
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup

from backend.rag_bot.config import DEFAULT_CONFIG


def normalize_url(url: str) -> str:
    """
    Normalize URL by removing fragments and standardizing format.

    Args:
        url: URL to normalize

    Returns:
        Normalized URL
    """
    parsed = urlparse(url)
    # Remove fragment but keep query parameters
    normalized = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
    if parsed.query:
        normalized += f"?{parsed.query}"
    return normalized


def is_valid_url(url: str) -> bool:
    """
    Validate if the URL has a proper format.

    Args:
        url: URL to validate

    Returns:
        True if valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def check_robots_txt(base_url: str, user_agent: str = "*") -> bool:
    """
    Check if crawling is allowed by robots.txt.

    Args:
        base_url: Base URL to check
        user_agent: User agent string

    Returns:
        True if crawling is allowed, False otherwise
    """
    try:
        rp = urllib.robotparser.RobotFileParser()
        rp.set_url(f"{base_url}/robots.txt")
        rp.read()
        return rp.can_fetch(user_agent, base_url)
    except Exception:
        # If robots.txt check fails, assume it's OK to crawl
        return True


def crawl_docusaurus_site(start_url: str, delay: float = 1.0) -> List[Dict[str, str]]:
    """
    Crawl a Docusaurus site to discover all unique pages.

    Args:
        start_url: The starting URL of the Docusaurus site
        delay: Delay between requests in seconds

    Returns:
        List of dictionaries containing URL, title, and other metadata
    """
    import logging
    logger = logging.getLogger(__name__)

    visited_urls: Set[str] = set()
    urls_to_visit: List[str] = [start_url]
    discovered_pages: List[Dict[str, str]] = []
    base_domain = urlparse(start_url).netloc

    logger.info(f"Starting crawl from: {start_url}")

    while urls_to_visit:
        current_url = urls_to_visit.pop(0)
        normalized_url = normalize_url(current_url)

        if normalized_url in visited_urls:
            continue

        visited_urls.add(normalized_url)

        try:
            # Respect rate limiting
            time.sleep(delay)

            logger.debug(f"Crawling: {normalized_url}")
            response = requests.get(normalized_url, timeout=DEFAULT_CONFIG["timeout"])

            if response.status_code != 200:
                logger.warning(f"Failed to fetch {normalized_url}: Status {response.status_code}")
                continue

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract page title
            title_tag = soup.find('title')
            page_title = title_tag.get_text().strip() if title_tag else ""

            # Store the discovered page
            discovered_pages.append({
                "url": normalized_url,
                "title": page_title,
                "raw_html": response.text
            })

            # Find all links on the page
            links = soup.find_all('a', href=True)

            for link in links:
                href = link['href']

                # Convert relative URLs to absolute
                absolute_url = urljoin(normalized_url, href)

                # Only follow links within the same domain
                if urlparse(absolute_url).netloc == base_domain:
                    # Normalize the URL and check if it hasn't been visited
                    normalized_link = normalize_url(absolute_url)
                    if normalized_link not in visited_urls and normalized_link not in urls_to_visit:
                        # Filter out non-HTML links (like PDFs, images, etc.)
                        if absolute_url.lower().endswith(('.html', '.htm')) or '?' not in absolute_url and '#' not in absolute_url:
                            urls_to_visit.append(absolute_url)

        except requests.RequestException as e:
            logger.error(f"Error crawling {normalized_url}: {str(e)}")
            continue
        except Exception as e:
            logger.error(f"Unexpected error crawling {normalized_url}: {str(e)}")
            continue

    logger.info(f"Crawling completed. Discovered {len(discovered_pages)} pages.")
    return discovered_pages