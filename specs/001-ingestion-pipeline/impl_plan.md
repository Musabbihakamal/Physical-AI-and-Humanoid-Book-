# Implementation Plan: Single-File Ingestion Pipeline for Docusaurus Site

## Technical Context

- **Project Structure**: Create `backend/` folder with only `main.py` containing all functionality
- **Pipeline Flow**: URL → Crawl → Extract → Chunk → Embed → Store in Qdrant (all in main.py)
- **Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, argparse, logging, urllib.robotparser, lxml (for better HTML parsing), tiktoken (for accurate token counting)
- **Architecture Constraint**: All functionality must be contained within a single main.py file
- **CLI Entry Point**: `python main.py --url <book_url>`
- **Memory Management**: Process large sites efficiently without exceeding memory limits, supporting sites up to 1000 pages

## Constitution Check

- **Security**: API keys and credentials must be handled securely using environment variables
- **Performance**: Process large sites efficiently with appropriate batching and memory management
- **Reliability**: Implement proper error handling and retry mechanisms
- **Maintainability**: Despite single-file constraint, organize code with clear functions and documentation
- **Scalability**: Design for sites of varying sizes with configurable parameters
- **Safety**: Ensure crawling respects robots.txt and doesn't overwhelm servers
- **Global Content Standards**: Follow proper documentation standards and code examples

## Gates

- All architectural decisions must be justified regarding the single-file constraint
- Security requirements for API key handling must be met
- Performance considerations for large sites must be addressed
- Compliance with website terms of service and robots.txt must be ensured

## Phase 0: Research

### Research Tasks

1. **Single-File Architecture Patterns**: Research best practices for organizing complex functionality within a single Python file while maintaining readability and maintainability.

2. **Dependency Identification**: Identify all required packages for crawling, text extraction, chunking, embeddings, and Qdrant storage.

3. **Memory Management**: Investigate techniques for processing large sites without exceeding memory limits when all functionality is in one file.

4. **Large Site Processing**: Determine optimal approaches for handling sites of varying sizes in a single-file architecture.

## Phase 1: Design Elements

### Data Model
- **PageData**: Contains URL, raw HTML, extracted text, metadata
- **ChunkData**: Contains text content, metadata (URL, page_title, section_title), embedding vector
- **ProcessingState**: Tracks progress, visited URLs, statistics

### API Contracts
- Command-line interface: `python main.py --url <url> [options]`
- Options include: log level, chunk size parameters, API settings, etc.

## Phase 2: Implementation Plan

### 1. Overall Pipeline Data Flow

The pipeline will follow a linear flow within main.py:

```
CLI Args → URL Discovery → Content Extraction → Chunking → Embedding → Qdrant Storage → Completion Report
```

Each step is implemented as a separate function within main.py, called sequentially from the main() function.

### 2. Logical Organization Inside main.py

The main.py file will be organized into these logical sections:

- **Imports and Configuration**: All imports and configuration constants at the top
- **Utility Functions**: Helper functions for URL manipulation, text processing, etc.
- **Crawling Functions**: Functions for discovering and collecting URLs from the site
- **Extraction Functions**: Functions for cleaning HTML and extracting meaningful content
- **Chunking Functions**: Functions for splitting content into appropriately sized chunks
- **Embedding Functions**: Functions for generating and managing embeddings via Cohere
- **Storage Functions**: Functions for connecting to and storing in Qdrant
- **Logging Setup**: Configuration for application logging
- **CLI Argument Parsing**: Definition and parsing of command-line arguments
- **Main Function**: Orchestrates the entire pipeline flow

### 3. Crawling Strategy

- **Implementation**: Within main.py, implement a breadth-first search crawler
- **URL Discovery**: Parse HTML content to find all internal links
- **Uniqueness**: Maintain a set of visited URLs within the processing function
- **Noise Filtering**: Use CSS selectors to identify and exclude common Docusaurus UI elements (navigation, footers, etc.)
- **Domain Restriction**: Only follow links within the same domain as the base URL
- **Rate Limiting**: Implement configurable delays between requests to avoid overwhelming the server

### 4. Content Extraction Approach

- **HTML Parsing**: Use BeautifulSoup to parse HTML content
- **Structure Preservation**: Extract and maintain heading hierarchy (h1-h6), lists, code blocks
- **Noise Removal**: Identify and remove navigation elements, footers, and sidebars using common Docusaurus CSS classes
- **Text Cleaning**: Remove HTML tags while preserving semantic meaning and readability
- **Metadata Extraction**: Extract page title and section information for each page

### 5. Chunking Strategy

- **Size Range**: Implement chunking to create segments of 400-700 words
- **Overlap Logic**: Apply 10-15% overlap between adjacent chunks to maintain context
- **Boundary Respect**: Avoid breaking chunks in the middle of code blocks, lists, or other semantic units
- **Metadata Attachment**: Attach URL, page_title, and section_title to each chunk
- **Coherence Maintenance**: Ensure chunks maintain semantic coherence and readability

### 6. Embedding Plan

- **Model Selection**: Use Cohere's embed-english-v3.0 or latest recommended model
- **Batching Implementation**: Group chunks into batches for efficient API processing
- **Retry Logic**: Implement exponential backoff with jitter for failed API requests
- **Rate Limit Handling**: Manage API quotas and handle rate limiting responses
- **Error Management**: Log failed embeddings and continue processing remaining chunks

### 7. Qdrant Schema and Metadata Payload

- **Collection Setup**: Define vector collection with appropriate dimensions for Cohere embeddings
- **Payload Structure**: Store metadata alongside embeddings including URL, titles, and content
- **Indexing**: Configure appropriate indexing for efficient retrieval
- **Idempotency Support**: Include content hash in payload to support duplicate detection

### 8. Idempotency Implementation

- **Content Hashing**: Generate hash of source content before embedding
- **Duplicate Check**: Query Qdrant for existing content with same hash before storing
- **Safe Re-runs**: Implement logic to skip already processed content during subsequent runs
- **Progress Tracking**: Optionally maintain processing state to support resumption

### 9. Logging Plan

- **Progress Logging**: Log progress at each major pipeline stage
- **Statistics Tracking**: Monitor and log metrics (pages processed, chunks created, etc.)
- **Error Reporting**: Log errors with sufficient context for debugging
- **Performance Metrics**: Track timing for major operations
- **Configurable Levels**: Support different log levels (DEBUG, INFO, WARNING, ERROR)

### 10. CLI Execution

- **Argument Definition**: Define --url as required parameter
- **Optional Parameters**:
  - `--log-level`: Set logging verbosity
  - `--chunk-min-words`: Minimum chunk size (default: 400)
  - `--chunk-max-words`: Maximum chunk size (default: 700)
  - `--overlap-percent`: Overlap percentage (default: 10)
  - `--batch-size`: Embedding batch size (default: 96)
  - `--qdrant-url`: Qdrant server URL (default: http://localhost:6333)
  - `--cohere-model`: Cohere model name (default: embed-english-v3.0)
- **Validation**: Validate URL format and accessibility before processing
- **Help Text**: Provide comprehensive help with all available options