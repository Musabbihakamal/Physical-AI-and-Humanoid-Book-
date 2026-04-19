"""
Embedder module for the ingestion pipeline
"""

import os
import time
from typing import List, Optional

import cohere

from backend.rag_bot.config import DEFAULT_CONFIG


def initialize_cohere_client() -> Optional['cohere.Client']:
    """
    Initialize Cohere client if API key is available.

    Returns:
        Cohere client instance or None if initialization fails
    """
    import logging
    logger = logging.getLogger(__name__)

    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        logger.error("COHERE_API_KEY environment variable not set")
        return None

    try:
        client = cohere.Client(api_key)
        # Test the connection
        client.embed(texts=["test"], model="embed-english-v3.0")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {str(e)}")
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
    import logging
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
    import logging
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