"""
Storage module for the ingestion pipeline
"""

from typing import Dict, List, Optional, Tuple

from qdrant_client import QdrantClient
from qdrant_client.http import models

from backend.rag_bot.config import DEFAULT_CONFIG


def initialize_qdrant_client(qdrant_url: str) -> Optional[Tuple[QdrantClient, str]]:
    """
    Initialize Qdrant client and create collection if needed.

    Args:
        qdrant_url: URL of the Qdrant server

    Returns:
        Tuple of (client, collection_name) or (None, None) if initialization fails
    """
    import logging
    logger = logging.getLogger(__name__)

    try:
        # Initialize client
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
    import logging
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
        point = models.PointStruct(
            id=f"{chunk['source_hash'][:16]}_{i}",  # Use hash prefix and index for ID
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