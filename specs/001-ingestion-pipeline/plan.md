# Implementation Plan: Ingestion Pipeline for Docusaurus Site

## 1. Overall Pipeline Architecture and Data Flow

The ingestion pipeline follows a sequential processing model with the following flow:

```
Input URL → Crawler → Extractor → Chunker → Embedder → Qdrant Store → Output
```

Each component performs a specific role in the pipeline:
- **Input**: URL of the Docusaurus site to crawl
- **Crawler**: Discovers all unique pages on the site
- **Extractor**: Cleans HTML and extracts meaningful text content
- **Chunker**: Splits content into manageable chunks with overlap
- **Embedder**: Generates vector embeddings for each chunk
- **Qdrant Store**: Persists embeddings with metadata to vector database
- **Output**: Confirmation of successful ingestion with statistics

The pipeline is designed to be idempotent - running it multiple times on the same URL should not create duplicate entries.

## 2. Module Responsibilities

### crawler.py
- Discovers all unique pages on the Docusaurus site starting from the provided URL
- Implements breadth-first or depth-first crawling strategy
- Tracks visited URLs to prevent infinite loops and duplicates
- Handles relative and absolute URL resolution
- Filters out UI elements, navigation, and other noise from crawl scope
- Respects robots.txt and implements rate limiting
- Returns a list of URLs with their associated metadata (page titles, hierarchy)

### extractor.py
- Extracts clean text content from HTML pages
- Preserves document structure: headings (h1-h6), lists (ul/ol), code blocks, paragraphs
- Removes navigation elements, footers, sidebars, and other UI noise
- Maintains text readability and context
- Extracts page title and section information
- Handles various HTML structures commonly found in Docusaurus sites
- Returns structured content objects with clean text and metadata

### chunker.py
- Splits extracted text into chunks of 400-700 words
- Implements 10-15% overlap between adjacent chunks to maintain context
- Attaches metadata to each chunk: URL, page title, section title
- Preserves semantic coherence when creating chunks (doesn't break in the middle of code blocks or lists)
- Ensures chunks don't exceed maximum word count while maintaining readability
- Returns list of chunk objects with content and metadata

### embedder.py
- Generates embeddings using Cohere's embedding model
- Implements batching for efficient API usage
- Handles API rate limits and quota management
- Implements retry logic with exponential backoff for failed requests
- Manages API credentials securely
- Returns embedding vectors for each chunk

### qdrant_store.py
- Connects to Qdrant vector database
- Defines collection schema for storing embeddings and metadata
- Implements idempotent storage (checks for existing entries before storing)
- Handles connection pooling and error recovery
- Implements efficient bulk insertion for performance
- Stores embeddings with associated metadata (URL, title, section, etc.)

### main.py
- Orchestrates the entire pipeline execution
- Parses command-line arguments (--url parameter)
- Validates input URL format
- Initializes each component module
- Manages pipeline execution flow and error handling
- Provides progress logging and statistics
- Handles graceful shutdown on interruption

## 3. Crawling Strategy

- **Discovery Method**: Breadth-first search starting from the provided URL
- **URL Tracking**: Maintain a set of visited URLs to prevent duplicates
- **URL Filtering**: Only follow links within the same domain/subdomain as the base URL
- **Noise Reduction**: Identify and exclude common UI elements by analyzing CSS classes and HTML structure typical of Docusaurus sites
- **Rate Limiting**: Implement configurable delay between requests to avoid overwhelming the server
- **Robots.txt Compliance**: Check robots.txt before crawling and respect crawl delays
- **Error Handling**: Continue crawling even if individual pages fail to load
- **URL Normalization**: Normalize URLs to handle different representations of the same page

## 4. Content Extraction Approach

- **HTML Parsing**: Use BeautifulSoup or similar library to parse HTML
- **Heading Preservation**: Extract and preserve heading hierarchy (h1-h6) with proper nesting
- **List Handling**: Preserve ordered and unordered lists with proper formatting
- **Code Block Extraction**: Extract code blocks separately to maintain syntax highlighting context
- **Text Cleaning**: Remove HTML tags while preserving text content and structure
- **Metadata Extraction**: Extract page title, meta descriptions, and other relevant metadata
- **Noise Filtering**: Remove navigation elements, footers, headers, and other UI components by targeting common Docusaurus CSS classes
- **Content Prioritization**: Focus on main content areas (typically within `<main>` or elements with content-specific classes)

## 5. Chunking Strategy

- **Size Range**: 400-700 words per chunk (configurable)
- **Overlap**: 10-15% overlap between adjacent chunks to maintain context
- **Boundary Respect**: Avoid breaking chunks in the middle of code blocks, lists, or other semantic units
- **Metadata Attachment**: Attach URL, page_title, section_title to each chunk
- **Coherence Maintenance**: Ensure chunks maintain semantic coherence
- **Minimum Size**: Ensure chunks don't fall below a minimum threshold (e.g., 100 words) unless the source content is very short
- **Structure Preservation**: Keep related content together (e.g., a heading with its following paragraph)

## 6. Embedding Plan

- **Model**: Cohere's embed-english-v3.0 or latest recommended model
- **Batching**: Process chunks in batches of 96 (optimal for Cohere API)
- **Retries**: Implement exponential backoff with jitter for API failures
- **Rate Limits**: Handle 429 (rate limit) responses gracefully with appropriate delays
- **Authentication**: Securely manage API keys using environment variables
- **Error Handling**: Log failed embeddings and continue processing remaining chunks
- **Timeouts**: Set appropriate timeouts for API requests (e.g., 30 seconds)

## 7. Qdrant Schema and Metadata Payload

- **Collection Configuration**:
  - Vector size: Match Cohere embedding dimensions (typically 1024)
  - Distance metric: Cosine similarity
  - Additional payload indexes for metadata fields

- **Payload Structure**:
  ```json
  {
    "url": "source URL",
    "page_title": "extracted page title",
    "section_title": "section header",
    "chunk_text": "the actual chunk content",
    "source_hash": "hash of original content for idempotency",
    "created_at": "timestamp"
  }
  ```

- **Indexing**: Create indexes on metadata fields for efficient querying
- **Idempotency**: Use content hash to prevent duplicate storage

## 8. Idempotency Implementation

- **Content Hashing**: Generate hash of source content before embedding
- **Duplicate Check**: Query Qdrant for existing content with same hash before storing
- **Progress Tracking**: Optionally maintain a local record of processed URLs
- **Safe Re-runs**: Running the pipeline multiple times should not create duplicates
- **Partial Recovery**: Ability to resume from point of failure without reprocessing everything

## 9. Logging Plan

- **Progress Logging**: Log progress at each stage (crawling, extraction, chunking, embedding, storage)
- **Statistics**: Track pages processed, chunks created, embeddings generated, storage operations
- **Error Logging**: Log all errors with sufficient context for debugging
- **Performance Metrics**: Log timing information for each major operation
- **Log Levels**: Support different log levels (INFO, DEBUG, WARNING, ERROR)
- **Structured Logging**: Use structured format for easier parsing and analysis

## 10. CLI Execution

Command format: `python main.py --url <book_url>`

- **Required Parameter**: `--url` accepts the Docusaurus site URL
- **Optional Parameters**:
  - `--log-level`: Set logging level (default: INFO)
  - `--batch-size`: Set embedding batch size (default: 96)
  - `--chunk-size-min`: Minimum chunk size in words (default: 400)
  - `--chunk-size-max`: Maximum chunk size in words (default: 700)
  - `--overlap-percent`: Overlap percentage between chunks (default: 10)
  - `--qdrant-url`: Qdrant server URL (default: http://localhost:6333)
  - `--cohere-model`: Cohere model name (default: embed-english-v3.0)

- **Validation**: Validate URL format and accessibility before starting pipeline
- **Help Text**: Provide clear help text with all available options
- **Exit Codes**: Return appropriate exit codes (0 for success, non-zero for errors)
- **Progress Indication**: Show progress indicators during long-running operations