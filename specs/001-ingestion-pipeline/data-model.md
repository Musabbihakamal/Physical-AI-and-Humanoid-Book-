# Data Model: Single-File Ingestion Pipeline

## PageData Entity
**Description**: Represents a single page from the Docusaurus site during processing

**Fields**:
- `url` (string): The source URL of the page
- `raw_html` (string): The raw HTML content from the page
- `clean_text` (string): Extracted clean text content
- `page_title` (string): The title of the page
- `section_title` (string): The section header (if applicable)
- `discovered_at` (datetime): Timestamp when the page was discovered

**Relationships**:
- One-to-many with ChunkData (one page can produce multiple chunks)

## ChunkData Entity
**Description**: Represents a text chunk created from page content for embedding

**Fields**:
- `id` (string): Unique identifier for the chunk
- `content` (string): The text content of the chunk (400-700 words)
- `url` (string): Source URL of the original page
- `page_title` (string): Title of the source page
- `section_title` (string): Section title of the content
- `word_count` (int): Number of words in the chunk
- `embedding_vector` (list[float]): The vector representation of the content
- `source_hash` (string): Hash of the original content for idempotency
- `created_at` (datetime): Timestamp when chunk was created

**Relationships**:
- Many-to-one with PageData (many chunks come from one page)

## ProcessingState Entity
**Description**: Tracks the current state of the ingestion process

**Fields**:
- `visited_urls` (set[string]): URLs that have been processed
- `processed_pages` (int): Count of pages processed
- `generated_chunks` (int): Count of chunks created
- `stored_embeddings` (int): Count of embeddings stored
- `start_time` (datetime): When the process started
- `current_stage` (string): Current stage of the pipeline (crawling, extracting, etc.)
- `errors` (list[dict]): Any errors encountered during processing

## EmbeddingRequest Entity
**Description**: Represents a batch of chunks to be sent to the Cohere API

**Fields**:
- `chunks` (list[ChunkData]): List of chunks to embed
- `model` (string): The embedding model to use
- `batch_size` (int): Number of chunks in the batch
- `retry_count` (int): Number of times this batch has been retried

## QdrantPayload Entity
**Description**: The payload structure for storing in Qdrant vector database

**Fields**:
- `vector` (list[float]): The embedding vector
- `payload` (dict): Metadata dictionary containing:
  - `url` (string): Source URL
  - `page_title` (string): Page title
  - `section_title` (string): Section title
  - `chunk_text` (string): The chunk content
  - `source_hash` (string): Hash for idempotency
  - `created_at` (datetime): Creation timestamp
- `id` (string): Unique identifier for the record