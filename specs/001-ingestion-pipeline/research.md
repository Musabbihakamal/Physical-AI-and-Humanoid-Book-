# Research: Single-File Ingestion Pipeline for Docusaurus Site

## Decision: Required Dependencies
**Rationale**: The ingestion pipeline requires several Python packages to function properly. Based on the functionality needed (crawling, HTML parsing, embeddings, vector storage), the following packages are essential:
- requests: for HTTP requests during crawling
- beautifulsoup4: for parsing and extracting content from HTML
- cohere: for generating text embeddings
- qdrant-client: for interacting with Qdrant vector database
- python-dotenv: for loading environment variables securely
- argparse: for parsing command-line arguments
- logging: for application logging (built-in)
- urllib.robotparser: for checking robots.txt compliance
- lxml: as an alternative HTML parser (optional but recommended for performance)
- tiktoken: for accurate token counting (alternative to word counting)

**Alternatives considered**:
- Using urllib instead of requests (requests is more feature-rich and easier to use)
- Using html.parser instead of lxml (lxml is faster and more robust)

## Decision: Maximum Site Size Handling
**Rationale**: The pipeline should be able to handle medium-sized Docusaurus sites (up to 1000 pages) efficiently. For memory management in a single-file architecture, the approach will be to process content incrementally rather than loading everything into memory at once. This includes:
- Process pages in batches
- Clear variables after processing each page/chunk
- Use generators where possible to avoid loading large datasets into memory
- Implement proper cleanup of BeautifulSoup objects after parsing

**Alternatives considered**:
- Loading all content at once (would cause memory issues for large sites)
- Streaming processing (more complex but not necessary for the expected site sizes)

## Decision: Single-File Architecture Organization
**Rationale**: The architecture constraint requires all functionality in a single main.py file. To maintain code quality, the file will be organized into logical sections with clear separation of concerns using functions and comments.
**Alternatives considered**:
- Multiple files (not allowed per constraints)
- Classes within single file (adds complexity without benefit for this use case)
- Monolithic function approach (would be unmaintainable)

## Decision: Memory Management for Large Sites
**Rationale**: Process content incrementally rather than loading everything into memory. Process pages/chunks in batches and clear variables when no longer needed to prevent memory accumulation.
**Alternatives considered**:
- Load all content at once (would cause memory issues for large sites)
- Stream processing (more complex but not necessary for this scale)

## Best Practice: Code Organization in Single File
**Pattern**: Organize main.py with the following structure:
1. Imports and constants
2. Utility functions
3. Core processing functions (crawling, extraction, chunking, embedding, storage)
4. CLI setup and main function
5. Guard clause for direct execution

## Best Practice: Error Handling in Monolithic Code
**Pattern**: Implement comprehensive error handling at each processing stage with specific exception types and appropriate fallbacks. Use context managers for resource handling.

## Best Practice: Configuration Management
**Pattern**: Use environment variables for sensitive data (API keys) and command-line arguments for operational parameters. Centralize configuration in a single function/object.

## Decision: Logging Configuration
**Rationale**: Implement structured logging with different levels to track progress and diagnose issues. Configure logging early in the application lifecycle.
**Alternatives considered**: Simple print statements (not suitable for production), external logging service (overkill for this use case)

## Decision: Testing Approach
**Rationale**: Since all code is in one file, implement comprehensive unit tests that can mock individual functions to test each component separately.
**Alternatives considered**: Integration tests only (would make debugging harder), no tests (not acceptable)

## Decision: Rate Limiting and Robots.txt Compliance
**Rationale**: Implement proper rate limiting to avoid overwhelming target servers and check robots.txt to comply with website terms of service. This includes:
- Configurable delay between requests (default 1 second)
- Checking robots.txt before crawling
- Respecting crawl-delay directives
- Handling 429 responses appropriately

**Alternatives considered**: No rate limiting (could get IP banned), ignoring robots.txt (violates terms of service)