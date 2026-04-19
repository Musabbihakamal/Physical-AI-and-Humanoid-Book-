#!/usr/bin/env python3
"""
Unified RAG Bot and Ingestion Pipeline

This script combines both the RAG chatbot and the ingestion pipeline in a single application.
- RAG Bot: Answer questions about ingested content
- Ingestion Pipeline: Crawl, extract, chunk, embed, and store Docusaurus content
"""

import argparse
import asyncio
import hashlib
import json
import os
import re
import sys
import time
import urllib.parse
import urllib.robotparser
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Set, Tuple
from urllib.parse import urljoin, urlparse

import cohere
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Import cohere - will be handled in setup if available
try:
    import cohere
except ImportError:
    cohere = None

try:
    import tiktoken
    ENCODER = tiktoken.get_encoding("cl100k_base")
except ImportError:
    ENCODER = None


# ===========================================
# CONFIGURATION AND CONSTANTS
# ===========================================

DEFAULT_CONFIG = {
    "log_level": "INFO",
    "chunk_min_words": 400,
    "chunk_max_words": 700,
    "overlap_percent": 10,
    "batch_size": 96,
    "qdrant_url": os.getenv("QDRANT_URL", "http://localhost:6333"),
    "cohere_model": "embed-english-v3.0",
    "request_delay": 1.0,  # seconds between requests
    "max_retries": 3,
    "timeout": 30,
    "rag_cohere_model": "command-r7b-12-2024",
    "rag_max_context_chunks": 5,
    "rag_similarity_threshold": 0.3,
    "rag_temperature": 0.3,
}


# ===========================================
# LOGGING SETUP
# ===========================================

import logging

def setup_logging(log_level: str = "INFO"):
    """Setup logging configuration."""
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('pipeline.log')
        ]
    )
    return logging.getLogger(__name__)


# ===========================================
# UTILITY FUNCTIONS
# ===========================================

def calculate_word_count(text: str) -> int:
    """Calculate word count using either tiktoken or simple split."""
    if ENCODER:
        # Use token count as approximation for word count
        return len(ENCODER.encode(text))
    else:
        # Fallback to simple word splitting
        return len(text.split())


def generate_content_hash(content: str) -> str:
    """Generate SHA-256 hash of content for idempotency."""
    return hashlib.sha256(content.encode()).hexdigest()


def normalize_url(url: str) -> str:
    """Normalize URL by removing fragments and standardizing format."""
    parsed = urlparse(url)
    # Remove fragment but keep query parameters
    normalized = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
    if parsed.query:
        normalized += f"?{parsed.query}"
    return normalized


def is_valid_url(url: str) -> bool:
    """Validate if the URL has a proper format."""
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def check_robots_txt(base_url: str, user_agent: str = "*") -> bool:
    """Check if crawling is allowed by robots.txt."""
    try:
        rp = urllib.robotparser.RobotFileParser()
        rp.set_url(f"{base_url}/robots.txt")
        rp.read()
        return rp.can_fetch(user_agent, base_url)
    except Exception:
        # If robots.txt check fails, assume it's OK to crawl
        return True


# ===========================================
# CRAWLING FUNCTIONALITY
# ===========================================

def crawl_docusaurus_site(start_url: str, delay: float = 1.0) -> List[Dict[str, str]]:
    """
    Crawl a Docusaurus site to discover all unique pages.

    Args:
        start_url: The starting URL of the Docusaurus site
        delay: Delay between requests in seconds

    Returns:
        List of dictionaries containing URL, title, and other metadata
    """
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


# ===========================================
# CONTENT EXTRACTION FUNCTIONALITY
# ===========================================

def extract_clean_content(html_content: str, url: str = "") -> Tuple[str, str, str]:
    """
    Extract clean text content from HTML, preserving structure.

    Args:
        html_content: Raw HTML content
        url: Source URL for context

    Returns:
        Tuple of (clean_text, page_title, section_title)
    """
    logger = logging.getLogger(__name__)
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove common Docusaurus UI elements that aren't content
    selectors_to_remove = [
        'nav', 'header', 'footer', '.navbar', '.menu', '.sidebar',
        '.theme-edit-this-page', '.theme-last-updated', '.pagination-nav',
        '.toc', '.table-of-contents', '.theme-doc-breadcrumbs'
    ]

    for selector in selectors_to_remove:
        for element in soup.select(selector):
            element.decompose()

    # Extract page title
    title_tag = soup.find('title')
    page_title = title_tag.get_text().strip() if title_tag else ""

    # Look for main heading as section title
    main_heading = soup.find(['h1', 'h2'])
    section_title = main_heading.get_text().strip() if main_heading else page_title

    # Extract main content - look for main content containers
    main_content_selectors = [
        'main', '.main-wrapper', '.theme-doc-markdown',
        '.container.padding-vert--xl', '.doc-content'
    ]

    main_content = None
    for selector in main_content_selectors:
        main_content = soup.select_one(selector)
        if main_content:
            break

    if not main_content:
        # If no specific container found, use body
        main_content = soup.find('body')

    if main_content:
        # Extract text content preserving structure
        clean_text = ""

        # Process different elements to preserve structure
        for element in main_content.descendants:
            if element.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
                clean_text += f"\n\n{element.get_text().strip()}\n"
            elif element.name in ['p', 'div']:
                text = element.get_text().strip()
                if text and not any(parent.name in ['script', 'style'] for parent in element.parents):
                    clean_text += f"{text} "
            elif element.name in ['li']:
                clean_text += f"- {element.get_text().strip()} "
            elif element.name in ['code', 'pre']:
                clean_text += f"\n```\n{element.get_text().strip()}\n```\n"
            elif element.name == 'br':
                clean_text += "\n"
            elif element.name is None and hasattr(element, 'strip'):
                # Text nodes
                text = str(element).strip()
                if text:
                    clean_text += text + " "

        clean_text = re.sub(r'\s+', ' ', clean_text).strip()
    else:
        # Fallback to just getting all text
        clean_text = soup.get_text()
        clean_text = re.sub(r'\s+', ' ', clean_text).strip()

    logger.debug(f"Extracted content from {url}: {len(clean_text)} characters")

    return clean_text, page_title, section_title


# ===========================================
# CHUNKING FUNCTIONALITY
# ===========================================

def chunk_text_with_overlap(
    text: str,
    min_words: int = 400,
    max_words: int = 700,
    overlap_percent: int = 10
) -> List[Dict]:
    """
    Split text into overlapping chunks.

    Args:
        text: Text to chunk
        min_words: Minimum number of words per chunk
        max_words: Maximum number of words per chunk
        overlap_percent: Percentage of overlap between chunks

    Returns:
        List of chunk dictionaries with content and metadata
    """
    logger = logging.getLogger(__name__)

    if not text.strip():
        return []

    words = text.split()
    chunks = []

    # Calculate chunk parameters
    max_chunk_size = max_words
    overlap_size = int(max_chunk_size * overlap_percent / 100)

    start_idx = 0
    while start_idx < len(words):
        # Determine the end index for this chunk
        end_idx = start_idx + max_chunk_size

        # If this is the last chunk and it's too small, include it as is
        if end_idx >= len(words) and (end_idx - start_idx) >= min_words:
            chunk_words = words[start_idx:end_idx]
        elif end_idx >= len(words):
            # Last chunk is too small, extend it to include more words if possible
            chunk_words = words[start_idx:]
        else:
            # Regular chunk
            chunk_words = words[start_idx:end_idx]

        # Ensure the chunk meets minimum size requirements unless it's the very last chunk
        if len(chunk_words) < min_words and end_idx < len(words):
            # This chunk is too small and not the last chunk, skip it and move to next position
            start_idx = end_idx
            continue

        chunk_text = ' '.join(chunk_words)

        # Create chunk with metadata
        chunk_data = {
            "content": chunk_text,
            "word_count": len(chunk_words),
            "source_hash": generate_content_hash(chunk_text),
            "created_at": datetime.utcnow().isoformat()
        }

        chunks.append(chunk_data)

        # Move start index considering overlap
        if end_idx >= len(words):
            # Last chunk, no more chunks
            break

        # Move start index by (chunk_size - overlap_size) to achieve desired overlap
        start_idx = end_idx - overlap_size

        # Ensure we don't go backward in case overlap_size > chunk_size
        if start_idx <= start_idx:
            start_idx = start_idx + max_chunk_size

    logger.info(f"Split content into {len(chunks)} chunks (min: {min_words}, max: {max_words}, overlap: {overlap_percent}%)")
    return chunks


# ===========================================
# EMBEDDING FUNCTIONALITY
# ===========================================

def initialize_cohere_client() -> Optional['cohere.Client']:
    """Initialize Cohere client if API key is available."""
    if not cohere:
        logging.error("Cohere library not installed. Please install with: pip install cohere")
        return None

    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        logging.error("COHERE_API_KEY environment variable not set")
        return None

    try:
        client = cohere.Client(api_key)
        # Test the connection
        client.embed(texts=["test"], model="embed-english-v3.0", input_type="search_document")
        return client
    except Exception as e:
        logging.error(f"Failed to initialize Cohere client: {str(e)}")
        return None


def generate_embeddings_batch(
    texts: List[str],
    cohere_client: 'cohere.Client',
    model: str = "embed-english-v3.0"
) -> Optional[List[List[float]]]:
    """
    Generate embeddings for a batch of texts using Cohere.

    Args:
        texts: List of texts to embed
        cohere_client: Initialized Cohere client
        model: Cohere embedding model to use

    Returns:
        List of embedding vectors or None if failed
    """
    logger = logging.getLogger(__name__)

    try:
        response = cohere_client.embed(
            texts=texts,
            model=model,
            input_type="search_document"  # Optimize for document search
        )
        return response.embeddings
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {str(e)}")
        return None


def generate_embeddings_with_retry(
    chunks: List[Dict],
    cohere_client: 'cohere.Client',
    model: str = "embed-english-v3.0",
    batch_size: int = 96,
    max_retries: int = 3
) -> List[Dict]:
    """
    Generate embeddings for chunks with retry logic and batching.

    Args:
        chunks: List of chunk dictionaries
        cohere_client: Initialized Cohere client
        model: Cohere embedding model to use
        batch_size: Number of texts to process in each batch
        max_retries: Maximum number of retry attempts

    Returns:
        List of chunks with embeddings added
    """
    logger = logging.getLogger(__name__)

    processed_chunks = []

    # Process in batches
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]

        # Extract text content for embedding
        texts = [chunk["content"] for chunk in batch]

        # Attempt to generate embeddings with retry logic
        embeddings = None
        for attempt in range(max_retries):
            try:
                logger.debug(f"Generating embeddings for batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}, attempt {attempt + 1}")
                embeddings = generate_embeddings_batch(texts, cohere_client, model)

                if embeddings is not None and len(embeddings) == len(texts):
                    break
                else:
                    logger.warning(f"Embedding generation failed for attempt {attempt + 1}")

            except Exception as e:
                logger.warning(f"Embedding generation error on attempt {attempt + 1}: {str(e)}")

            if attempt < max_retries - 1:
                # Wait before retry with exponential backoff
                wait_time = (2 ** attempt) + (os.urandom(1)[0] / 255.0)  # Add jitter
                time.sleep(wait_time)

        if embeddings is None or len(embeddings) != len(texts):
            logger.error(f"Failed to generate embeddings for batch after {max_retries} attempts")
            # Still add chunks without embeddings to keep track
            for j, chunk in enumerate(batch):
                updated_chunk = chunk.copy()
                updated_chunk["embedding_vector"] = None
                updated_chunk["embedding_success"] = False
                processed_chunks.append(updated_chunk)
        else:
            # Add embeddings to chunks
            for j, chunk in enumerate(batch):
                updated_chunk = chunk.copy()
                updated_chunk["embedding_vector"] = embeddings[j]
                updated_chunk["embedding_success"] = True
                processed_chunks.append(updated_chunk)

    successful_embeddings = sum(1 for chunk in processed_chunks if chunk.get("embedding_success", False))
    logger.info(f"Successfully generated embeddings for {successful_embeddings}/{len(processed_chunks)} chunks")

    return processed_chunks


# ===========================================
# QDRANT STORAGE FUNCTIONALITY
# ===========================================

def initialize_qdrant_client(qdrant_url: str, api_key: str = None) -> Optional[QdrantClient]:
    """Initialize Qdrant client and create collection if needed."""
    logger = logging.getLogger(__name__)

    try:
        # Get API key from environment if not provided
        if not api_key:
            api_key = os.getenv("QDRANT_API_KEY")

        # Initialize client with or without API key
        if api_key:
            client = QdrantClient(url=qdrant_url, api_key=api_key, timeout=30)
        else:
            client = QdrantClient(url=qdrant_url, timeout=30)

        # Check if collection exists, create if not
        collection_name = "docusaurus_embeddings"

        try:
            collections = client.get_collections()
            collection_exists = any(col.name == collection_name for col in collections.collections)
        except:
            collection_exists = False

        if not collection_exists:
            # Create collection with appropriate vector size for Cohere embeddings
            # Cohere embed-english-v3.0 produces 1024-dimensional vectors
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )

            # Create payload index for faster metadata queries
            client.create_payload_index(
                collection_name=collection_name,
                field_name="source_hash",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            logger.info(f"Created new Qdrant collection: {collection_name}")
        else:
            logger.info(f"Using existing Qdrant collection: {collection_name}")

        return client, collection_name
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {str(e)}")
        return None, None


def store_embeddings_in_qdrant(
    chunks: List[Dict],
    qdrant_client: QdrantClient,
    collection_name: str,
    check_duplicates: bool = True
) -> int:
    """
    Store chunks with embeddings in Qdrant vector database.

    Args:
        chunks: List of chunks with embeddings
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to store in
        check_duplicates: Whether to check for duplicates before storing

    Returns:
        Number of embeddings successfully stored
    """
    logger = logging.getLogger(__name__)

    stored_count = 0
    chunks_to_store = []

    # Prepare points for storage
    for i, chunk in enumerate(chunks):
        if not chunk.get("embedding_success", False) or chunk.get("embedding_vector") is None:
            logger.warning(f"Skipping chunk {i} due to embedding failure")
            continue

        # Prepare payload
        payload = {
            "url": chunk.get("url", ""),
            "page_title": chunk.get("page_title", ""),
            "section_title": chunk.get("section_title", ""),
            "chunk_text": chunk["content"],
            "source_hash": chunk["source_hash"],
            "created_at": chunk["created_at"]
        }

        # Check for duplicates if required
        if check_duplicates:
            # Search for existing records with the same source_hash
            existing_points = qdrant_client.scroll(
                collection_name=collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_hash",
                            match=models.MatchValue(value=chunk["source_hash"])
                        )
                    ]
                ),
                limit=1
            )

            if existing_points[0]:  # If any existing points found
                logger.debug(f"Skipping duplicate chunk with hash: {chunk['source_hash'][:16]}...")
                continue

        # Create point for storage
        # Generate a valid UUID from the source hash and index
        point_id = str(uuid.UUID(chunk['source_hash'][:32]))

        point = models.PointStruct(
            id=point_id,
            vector=chunk["embedding_vector"],
            payload=payload
        )

        chunks_to_store.append(point)

    if not chunks_to_store:
        logger.info("No chunks to store in Qdrant")
        return 0

    try:
        # Upsert the points to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=chunks_to_store
        )

        stored_count = len(chunks_to_store)
        logger.info(f"Successfully stored {stored_count} embeddings in Qdrant")

    except Exception as e:
        logger.error(f"Failed to store embeddings in Qdrant: {str(e)}")
        return 0

    return stored_count


# ===========================================
# RAG BOT IMPLEMENTATION
# ===========================================

class RagBot:
    """RAG Bot implementation for question answering using retrieved context."""

    def __init__(
        self,
        qdrant_url: str = DEFAULT_CONFIG["qdrant_url"],
        cohere_model: str = DEFAULT_CONFIG["rag_cohere_model"],
        collection_name: str = "docusaurus_embeddings",
    ):
        """
        Initialize the RAG Bot with Qdrant and Cohere clients.

        Args:
            qdrant_url: URL of the Qdrant server
            cohere_model: Cohere model to use for generation
            collection_name: Name of the collection containing embeddings
        """
        self.qdrant_url = qdrant_url
        self.cohere_model = cohere_model
        self.collection_name = collection_name

        # Get API key from environment
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Initialize Qdrant client
        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=30)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url, timeout=30)

        # Check if collection exists
        try:
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == collection_name for col in collections.collections)

            if not collection_exists:
                raise ValueError(f"Collection '{collection_name}' does not exist. "
                               f"Run the ingestion pipeline first to populate the database.")
        except Exception as e:
            print(f"Error connecting to Qdrant or collection not found: {e}")
            raise

        # Initialize Cohere client
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable not set")

        try:
            self.cohere_client = cohere.Client(api_key)
            # Test the connection
            self.cohere_client.chat(message="test", model=self.cohere_model)
        except Exception as e:
            raise ValueError(f"Failed to initialize Cohere client: {e}")

        print(f"RAG Bot initialized with collection: {collection_name}")
        print(f"Using Cohere model: {self.cohere_model}")

    def retrieve_relevant_chunks(self, query: str, limit: int = 5, threshold: float = 0.3) -> List[Dict]:
        """
        Retrieve relevant text chunks from the vector database based on the query.

        Args:
            query: The user's question/query
            limit: Maximum number of chunks to retrieve
            threshold: Minimum similarity score threshold

        Returns:
            List of dictionaries containing relevant chunks and metadata
        """
        try:
            # Generate embedding for the query
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",  # Use embedding model to match ingestion
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]

            # Search for similar chunks in Qdrant using query_points
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit * 2,  # Get more results than needed for filtering
                score_threshold=threshold,
                with_payload=True,
                with_vectors=False
            )

            # Filter and format results
            relevant_chunks = []
            # query_points returns a QueryResponse object with .points attribute
            results_list = search_results.points if hasattr(search_results, 'points') else search_results

            for result in results_list:
                if result.score >= threshold:
                    chunk_data = {
                        "score": result.score,
                        "text": result.payload.get("chunk_text", ""),
                        "url": result.payload.get("url", ""),
                        "page_title": result.payload.get("page_title", ""),
                        "section_title": result.payload.get("section_title", ""),
                        "source_hash": result.payload.get("source_hash", "")
                    }
                    relevant_chunks.append(chunk_data)

            # Sort by relevance score (descending)
            relevant_chunks.sort(key=lambda x: x["score"], reverse=True)

            # Limit to desired number
            return relevant_chunks[:limit]

        except Exception as e:
            print(f"Error retrieving relevant chunks: {e}")
            return []

    def generate_response(self, query: str, context_chunks: List[Dict]) -> str:
        """
        Generate a response using the query and retrieved context chunks.

        Args:
            query: The user's question
            context_chunks: Retrieved context chunks

        Returns:
            Generated response string
        """
        try:
            # Format context for the model
            formatted_context = ""
            sources = set()

            for i, chunk in enumerate(context_chunks, 1):
                formatted_context += f"Context Chunk {i}:\n"
                formatted_context += f"Source: {chunk['url']}\n"
                formatted_context += f"Section: {chunk['section_title']}\n"
                formatted_context += f"Content: {chunk['text']}\n\n"

                if chunk['url']:
                    sources.add(chunk['url'])

            # Create the full prompt
            prompt = (
                "You are a helpful assistant that answers questions based on provided documentation.\n"
                "Use only the information in the context sections below to answer the user's question.\n"
                "If the context doesn't contain enough information to answer the question, say so.\n"
                "Be concise but thorough in your response.\n\n"
                f"Context Sections:\n{formatted_context}\n"
                f"User Question: {query}\n"
                "Assistant Response:"
            )

            # Generate response using Cohere
            response = self.cohere_client.chat(
                message=prompt,
                model=self.cohere_model,
                temperature=DEFAULT_CONFIG["rag_temperature"],
                max_tokens=1000,
                preamble=(
                    "You are a helpful documentation assistant. Answer questions based on the "
                    "provided context sections. Always cite sources when possible. "
                    "If the context doesn't contain the information needed to answer, acknowledge this limitation."
                )
            )

            # Get the response text
            answer = response.text

            # Add source citations
            if sources:
                answer += f"\n\nSources:\n" + "\n".join([f"- {source}" for source in list(sources)[:3]])  # Limit to 3 sources

            return answer

        except Exception as e:
            print(f"Error generating response: {e}")
            return "Sorry, I encountered an error while generating a response. Please try again."

    def answer_question(self, query: str) -> str:
        """
        Complete pipeline: retrieve context and generate response.

        Args:
            query: The user's question

        Returns:
            Generated response with sources
        """
        print(f"Processing query: {query}")

        # Retrieve relevant chunks
        context_chunks = self.retrieve_relevant_chunks(
            query,
            limit=DEFAULT_CONFIG["rag_max_context_chunks"],
            threshold=DEFAULT_CONFIG["rag_similarity_threshold"]
        )

        if not context_chunks:
            return "I couldn't find any relevant information to answer your question. The documentation might not contain the information you're looking for."

        print(f"Retrieved {len(context_chunks)} relevant context chunks")

        # Generate response using the context
        response = self.generate_response(query, context_chunks)

        return response

    async def interactive_chat(self):
        """Run an interactive chat session with the user."""
        print("\n" + "="*60)
        print("RAG Bot Interactive Chat")
        print("="*60)
        print("Ask questions about the ingested documentation!")
        print("Type 'quit', 'exit', or 'bye' to end the session")
        print("-" * 60)

        while True:
            try:
                query = input("\nYour question: ").strip()

                if query.lower() in ['quit', 'exit', 'bye', 'stop']:
                    print("Thank you for using the RAG Bot. Goodbye!")
                    break

                if not query:
                    print("Please ask a question.")
                    continue

                # Get response
                response = self.answer_question(query)

                print(f"\nAnswer: {response}")
                print("-" * 60)

            except KeyboardInterrupt:
                print("\n\nChat interrupted. Goodbye!")
                break
            except Exception as e:
                print(f"\nError during chat: {e}")
                print("Please try asking your question again.")


# ===========================================
# INGESTION PIPELINE ORCHESTRATION
# ===========================================

def run_ingestion_pipeline(
    url: str,
    log_level: str = "INFO",
    chunk_min_words: int = 400,
    chunk_max_words: int = 700,
    overlap_percent: int = 10,
    batch_size: int = 96,
    qdrant_url: str = "http://localhost:6333",
    cohere_model: str = "embed-english-v3.0",
    request_delay: float = 1.0
) -> Dict[str, any]:
    """
    Run the complete ingestion pipeline.

    Args:
        url: URL of the Docusaurus site to ingest
        log_level: Logging level
        chunk_min_words: Minimum chunk size in words
        chunk_max_words: Maximum chunk size in words
        overlap_percent: Overlap percentage between chunks
        batch_size: Batch size for embedding generation
        qdrant_url: URL of Qdrant server
        cohere_model: Cohere model to use for embeddings
        request_delay: Delay between requests in seconds

    Returns:
        Dictionary with pipeline results and statistics
    """
    logger = setup_logging(log_level)
    start_time = time.time()

    logger.info(f"Starting ingestion pipeline for: {url}")
    logger.info(f"Configuration: chunk_size={chunk_min_words}-{chunk_max_words}, overlap={overlap_percent}%, batch_size={batch_size}")

    results = {
        "url": url,
        "start_time": start_time,
        "end_time": None,
        "duration": None,
        "pages_processed": 0,
        "content_extracted": 0,
        "chunks_created": 0,
        "embeddings_generated": 0,
        "embeddings_stored": 0,
        "errors": []
    }

    try:
        # Step 1: Crawl the site
        logger.info("Step 1: Crawling the Docusaurus site")
        discovered_pages = crawl_docusaurus_site(url, request_delay)
        results["pages_processed"] = len(discovered_pages)
        logger.info(f"Crawled {len(discovered_pages)} pages")

        if not discovered_pages:
            logger.error("No pages discovered. Check the URL and network connectivity.")
            results["errors"].append("No pages discovered during crawling")
            return results

        # Step 2: Extract content from each page
        logger.info("Step 2: Extracting clean content from pages")
        all_chunks = []

        for page in discovered_pages:
            try:
                clean_text, page_title, section_title = extract_clean_content(
                    page["raw_html"], page["url"]
                )

                if not clean_text.strip():
                    logger.warning(f"No content extracted from {page['url']}")
                    continue

                # Add page metadata to the text for context in chunks
                enhanced_text = f"PAGE: {page_title}\nSECTION: {section_title}\nCONTENT: {clean_text}"

                # Step 3: Chunk the content
                page_chunks = chunk_text_with_overlap(
                    enhanced_text,
                    min_words=chunk_min_words,
                    max_words=chunk_max_words,
                    overlap_percent=overlap_percent
                )

                # Add page-specific metadata to each chunk
                for chunk in page_chunks:
                    chunk["url"] = page["url"]
                    chunk["page_title"] = page_title
                    chunk["section_title"] = section_title

                all_chunks.extend(page_chunks)

            except Exception as e:
                logger.error(f"Error processing page {page['url']}: {str(e)}")
                results["errors"].append(f"Error processing page {page['url']}: {str(e)}")
                continue

        results["content_extracted"] = len([p for p in discovered_pages if p.get("raw_html")])
        results["chunks_created"] = len(all_chunks)
        logger.info(f"Extracted content from {results['content_extracted']} pages, creating {len(all_chunks)} chunks")

        if not all_chunks:
            logger.error("No content chunks created. Pipeline stopped.")
            results["errors"].append("No content chunks created")
            return results

        # Step 4: Initialize Cohere client and generate embeddings
        logger.info("Step 3: Initializing Cohere client and generating embeddings")
        cohere_client = initialize_cohere_client()

        if not cohere_client:
            logger.error("Could not initialize Cohere client. Pipeline stopped.")
            results["errors"].append("Could not initialize Cohere client")
            return results

        logger.info(f"Generating embeddings using model: {cohere_model}")
        chunks_with_embeddings = generate_embeddings_with_retry(
            all_chunks,
            cohere_client,
            model=cohere_model,
            batch_size=batch_size
        )

        # Count successful embeddings
        results["embeddings_generated"] = sum(
            1 for chunk in chunks_with_embeddings if chunk.get("embedding_success", False)
        )
        logger.info(f"Generated {results['embeddings_generated']} embeddings successfully")

        if results["embeddings_generated"] == 0:
            logger.error("No embeddings were generated successfully. Pipeline stopped.")
            results["errors"].append("No embeddings generated successfully")
            return results

        # Step 5: Initialize Qdrant and store embeddings
        logger.info("Step 4: Initializing Qdrant client and storing embeddings")
        qdrant_client, collection_name = initialize_qdrant_client(qdrant_url)

        if not qdrant_client:
            logger.error("Could not initialize Qdrant client. Pipeline stopped.")
            results["errors"].append("Could not initialize Qdrant client")
            return results

        logger.info(f"Storing embeddings in Qdrant collection: {collection_name}")
        stored_count = store_embeddings_in_qdrant(
            chunks_with_embeddings,
            qdrant_client,
            collection_name
        )

        results["embeddings_stored"] = stored_count
        logger.info(f"Stored {stored_count} embeddings in Qdrant")

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        results["errors"].append("Pipeline interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error in pipeline: {str(e)}")
        results["errors"].append(f"Unexpected error: {str(e)}")

    finally:
        end_time = time.time()
        duration = end_time - start_time
        results["end_time"] = end_time
        results["duration"] = duration

        logger.info(f"Pipeline completed in {duration:.2f} seconds")
        logger.info(f"Results - Pages: {results['pages_processed']}, Chunks: {results['chunks_created']}, "
                   f"Embeddings: {results['embeddings_generated']}/{results['embeddings_stored']}")

        if results["errors"]:
            logger.error(f"Pipeline completed with {len(results['errors'])} errors")

    return results


# ===========================================
# PDF INGESTION PIPELINE
# ===========================================

def run_pdf_ingestion(
    pdf_path: str,
    log_level: str = "INFO",
    chunk_min_words: int = 400,
    chunk_max_words: int = 700,
    overlap_percent: int = 10,
    batch_size: int = 96,
    qdrant_url: str = "http://localhost:6333",
    cohere_model: str = "embed-english-v3.0"
) -> Dict[str, any]:
    """
    Run PDF ingestion pipeline.

    Args:
        pdf_path: Path to the PDF file
        log_level: Logging level
        chunk_min_words: Minimum chunk size in words
        chunk_max_words: Maximum chunk size in words
        overlap_percent: Overlap percentage between chunks
        batch_size: Batch size for embedding generation
        qdrant_url: URL of Qdrant server
        cohere_model: Cohere model to use for embeddings

    Returns:
        Dictionary with pipeline results and statistics
    """
    logger = setup_logging(log_level)
    start_time = time.time()

    logger.info(f"Starting PDF ingestion pipeline for: {pdf_path}")

    results = {
        "pdf_path": pdf_path,
        "start_time": start_time,
        "end_time": None,
        "duration": None,
        "pages_processed": 0,
        "chunks_created": 0,
        "embeddings_generated": 0,
        "embeddings_stored": 0,
        "errors": []
    }

    try:
        # Import PDF extractor
        from rag_bot.pdf_extractor import extract_text_from_pdf

        # Step 1: Extract text from PDF
        logger.info("Step 1: Extracting text from PDF")
        pages = extract_text_from_pdf(pdf_path)
        results["pages_processed"] = len(pages)
        logger.info(f"Extracted text from {len(pages)} pages")

        if not pages:
            logger.error("No pages extracted from PDF")
            results["errors"].append("No pages extracted from PDF")
            return results

        # Step 2: Chunk the content
        logger.info("Step 2: Chunking PDF content")
        all_chunks = []

        for page in pages:
            try:
                # Chunk the page content
                page_chunks = chunk_text_with_overlap(
                    page['content'],
                    min_words=chunk_min_words,
                    max_words=chunk_max_words,
                    overlap_percent=overlap_percent
                )

                # Add page-specific metadata to each chunk
                for chunk in page_chunks:
                    chunk['url'] = f"PDF Page {page['page_number']}"
                    chunk['page_title'] = f"{page['source_file']} - Page {page['page_number']}"
                    chunk['section_title'] = page['title']

                all_chunks.extend(page_chunks)

            except Exception as e:
                logger.error(f"Error processing page {page['page_number']}: {str(e)}")
                results["errors"].append(f"Error processing page {page['page_number']}: {str(e)}")
                continue

        results["chunks_created"] = len(all_chunks)
        logger.info(f"Created {len(all_chunks)} chunks from PDF")

        if not all_chunks:
            logger.error("No content chunks created from PDF")
            results["errors"].append("No content chunks created")
            return results

        # Step 3: Initialize Cohere client and generate embeddings
        logger.info("Step 3: Generating embeddings")
        cohere_client = initialize_cohere_client()

        if not cohere_client:
            logger.error("Could not initialize Cohere client")
            results["errors"].append("Could not initialize Cohere client")
            return results

        chunks_with_embeddings = generate_embeddings_with_retry(
            all_chunks,
            cohere_client,
            model=cohere_model,
            batch_size=batch_size
        )

        results["embeddings_generated"] = sum(
            1 for chunk in chunks_with_embeddings if chunk.get("embedding_success", False)
        )
        logger.info(f"Generated {results['embeddings_generated']} embeddings")

        if results["embeddings_generated"] == 0:
            logger.error("No embeddings generated successfully")
            results["errors"].append("No embeddings generated successfully")
            return results

        # Step 4: Store in Qdrant
        logger.info("Step 4: Storing embeddings in Qdrant")
        qdrant_client, collection_name = initialize_qdrant_client(qdrant_url)

        if not qdrant_client:
            logger.error("Could not initialize Qdrant client")
            results["errors"].append("Could not initialize Qdrant client")
            return results

        stored_count = store_embeddings_in_qdrant(
            chunks_with_embeddings,
            qdrant_client,
            collection_name
        )

        results["embeddings_stored"] = stored_count
        logger.info(f"Stored {stored_count} embeddings in Qdrant")

    except Exception as e:
        logger.error(f"Unexpected error in PDF ingestion pipeline: {str(e)}")
        results["errors"].append(f"Unexpected error: {str(e)}")

    finally:
        end_time = time.time()
        duration = end_time - start_time
        results["end_time"] = end_time
        results["duration"] = duration

        logger.info(f"PDF ingestion completed in {duration:.2f} seconds")
        logger.info(f"Results - Pages: {results['pages_processed']}, Chunks: {results['chunks_created']}, "
                   f"Embeddings: {results['embeddings_generated']}/{results['embeddings_stored']}")

        if results["errors"]:
            logger.error(f"Pipeline completed with {len(results['errors'])} errors")

    return results


# ===========================================
# CLI ARGUMENT PARSING
# ===========================================

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Unified RAG Bot and Ingestion Pipeline: Crawl, extract, chunk, embed, store, and answer questions"
    )

    # Mode selection
    parser.add_argument(
        "--mode",
        type=str,
        choices=["ingest", "pdf", "chat", "ask"],
        default="chat",
        help="Operation mode: ingest (web crawl), pdf (PDF file), chat (interactive RAG bot), ask (single question)"
    )

    # Ingestion arguments
    parser.add_argument(
        "--url",
        type=str,
        help="URL of the Docusaurus site to ingest (required for ingest mode)"
    )

    parser.add_argument(
        "--pdf-path",
        type=str,
        help="Path to PDF file to ingest (required for pdf mode)"
    )

    parser.add_argument(
        "--log-level",
        type=str,
        default=DEFAULT_CONFIG["log_level"],
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)"
    )

    parser.add_argument(
        "--chunk-min-words",
        type=int,
        default=DEFAULT_CONFIG["chunk_min_words"],
        help="Minimum chunk size in words (default: 400)"
    )

    parser.add_argument(
        "--chunk-max-words",
        type=int,
        default=DEFAULT_CONFIG["chunk_max_words"],
        help="Maximum chunk size in words (default: 700)"
    )

    parser.add_argument(
        "--overlap-percent",
        type=int,
        default=DEFAULT_CONFIG["overlap_percent"],
        help="Overlap percentage between chunks (default: 10)"
    )

    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_CONFIG["batch_size"],
        help="Batch size for embedding generation (default: 96)"
    )

    parser.add_argument(
        "--qdrant-url",
        type=str,
        default=DEFAULT_CONFIG["qdrant_url"],
        help="Qdrant server URL (default: http://localhost:6333)"
    )

    parser.add_argument(
        "--cohere-model",
        type=str,
        default=DEFAULT_CONFIG["cohere_model"],
        help="Cohere model to use for embeddings (default: embed-english-v3.0)"
    )

    parser.add_argument(
        "--request-delay",
        type=float,
        default=DEFAULT_CONFIG["request_delay"],
        help="Delay between requests in seconds (default: 1.0)"
    )

    # RAG bot arguments
    parser.add_argument(
        "--collection",
        type=str,
        default="docusaurus_embeddings",
        help="Qdrant collection name containing embeddings (default: docusaurus_embeddings)"
    )

    parser.add_argument(
        "--rag-model",
        type=str,
        default=DEFAULT_CONFIG["rag_cohere_model"],
        help="Cohere model to use for RAG generation (default: command-r7b-12-2024)"
    )

    parser.add_argument(
        "--max-chunks",
        type=int,
        default=DEFAULT_CONFIG["rag_max_context_chunks"],
        help="Maximum number of context chunks to retrieve (default: 5)"
    )

    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_CONFIG["rag_similarity_threshold"],
        help="Similarity threshold for chunk retrieval (default: 0.3)"
    )

    parser.add_argument(
        "--query",
        type=str,
        help="Single query to process (for ask mode)"
    )

    return parser.parse_args()


# ===========================================
# MAIN EXECUTION
# ===========================================

def main():
    """Main entry point for the unified application."""
    args = parse_arguments()

    if args.mode == "ingest":
        if not args.url:
            print("Error: --url is required for ingestion mode")
            sys.exit(1)

        # Validate URL
        if not is_valid_url(args.url):
            print(f"Error: Invalid URL format: {args.url}")
            sys.exit(1)

        # Check robots.txt compliance
        if not check_robots_txt(args.url):
            print(f"Warning: robots.txt for {args.url} may restrict crawling")
            response = input("Do you want to continue? (y/N): ")
            if response.lower() not in ['y', 'yes']:
                print("Pipeline cancelled by user.")
                sys.exit(0)

        # Run the ingestion pipeline
        results = run_ingestion_pipeline(
            url=args.url,
            log_level=args.log_level,
            chunk_min_words=args.chunk_min_words,
            chunk_max_words=args.chunk_max_words,
            overlap_percent=args.overlap_percent,
            batch_size=args.batch_size,
            qdrant_url=args.qdrant_url,
            cohere_model=args.cohere_model,
            request_delay=args.request_delay
        )

        # Print summary
        print("\n" + "="*60)
        print("INGESTION PIPELINE SUMMARY")
        print("="*60)
        print(f"Target URL: {results['url']}")
        print(f"Pages processed: {results['pages_processed']}")
        print(f"Content extracted: {results['content_extracted']}")
        print(f"Chunks created: {results['chunks_created']}")
        print(f"Embeddings generated: {results['embeddings_generated']}")
        print(f"Embeddings stored: {results['embeddings_stored']}")
        print(f"Duration: {results['duration']:.2f} seconds")

        if results['errors']:
            print(f"\nErrors occurred: {len(results['errors'])}")
            for i, error in enumerate(results['errors'], 1):
                print(f"  {i}. {error}")

        print("="*60)

    elif args.mode == "pdf":
        if not args.pdf_path:
            print("Error: --pdf-path is required for pdf mode")
            sys.exit(1)

        # Check if file exists
        if not os.path.exists(args.pdf_path):
            print(f"Error: PDF file not found: {args.pdf_path}")
            sys.exit(1)

        # Run the PDF ingestion pipeline
        results = run_pdf_ingestion(
            pdf_path=args.pdf_path,
            log_level=args.log_level,
            chunk_min_words=args.chunk_min_words,
            chunk_max_words=args.chunk_max_words,
            overlap_percent=args.overlap_percent,
            batch_size=args.batch_size,
            qdrant_url=args.qdrant_url,
            cohere_model=args.cohere_model
        )

        # Print summary
        print("\n" + "="*60)
        print("PDF INGESTION PIPELINE SUMMARY")
        print("="*60)
        print(f"PDF File: {results['pdf_path']}")
        print(f"Pages processed: {results['pages_processed']}")
        print(f"Chunks created: {results['chunks_created']}")
        print(f"Embeddings generated: {results['embeddings_generated']}")
        print(f"Embeddings stored: {results['embeddings_stored']}")
        print(f"Duration: {results['duration']:.2f} seconds")

        if results['errors']:
            print(f"\nErrors occurred: {len(results['errors'])}")
            for i, error in enumerate(results['errors'], 1):
                print(f"  {i}. {error}")

        print("="*60)

    elif args.mode == "chat" or args.mode == "ask":
        try:
            # Update default config with command line arguments
            DEFAULT_CONFIG["rag_max_context_chunks"] = args.max_chunks
            DEFAULT_CONFIG["rag_similarity_threshold"] = args.threshold

            # Initialize the RAG Bot
            print("Initializing RAG Bot...")
            rag_bot = RagBot(
                qdrant_url=args.qdrant_url,
                cohere_model=args.rag_model,
                collection_name=args.collection
            )

            if args.mode == "ask":
                if not args.query:
                    print("Error: --query is required for ask mode")
                    sys.exit(1)

                # Process single query in non-interactive mode
                response = rag_bot.answer_question(args.query)
                print(f"\nQuestion: {args.query}")
                print(f"Answer: {response}")
            else:
                # Run interactive chat session
                asyncio.run(rag_bot.interactive_chat())

        except ValueError as e:
            print(f"Configuration Error: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error running RAG Bot: {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()