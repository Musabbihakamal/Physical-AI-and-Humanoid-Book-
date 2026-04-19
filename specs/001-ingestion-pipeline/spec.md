# Feature Specification: Ingestion Pipeline for Docusaurus Site

## Overview
Create an ingestion pipeline that crawls a Docusaurus site, extracts clean text, chunks the content, generates Cohere embeddings, and stores them in Qdrant vector database. The pipeline should accept a URL as input and process the entire site in a reliable, idempotent manner.

## User Scenarios & Testing

### Primary User Scenario
1. User runs the ingestion pipeline with a Docusaurus site URL: `python main.py --url <docusaurus_site_url>`
2. System discovers all unique pages on the site through crawling
3. System extracts clean text content from each page, preserving important structural elements
4. System chunks the content according to specified parameters with metadata
5. System generates embeddings using Cohere API
6. System stores the embeddings in Qdrant with associated metadata
7. User receives confirmation of successful ingestion with statistics

### Secondary User Scenarios
- User runs the pipeline again on the same URL without creating duplicate entries
- User monitors progress through detailed logging
- System gracefully handles network failures during crawling or API calls

## Functional Requirements

### FR1: Project Setup
- The system shall create a `backend/` directory structure
- The system shall initialize a Python project using `uv` package manager
- The system shall define required packages for crawling, text processing, embeddings, and vector storage

### FR2: Crawling Module
- The system shall discover all unique pages on the Docusaurus site starting from the provided URL
- The system shall handle relative and absolute URLs appropriately
- The system shall ignore UI noise elements (navigation bars, footers, sidebars)
- The system shall avoid infinite loops by tracking visited URLs
- The system shall respect robots.txt and rate limiting

### FR3: Content Extraction Module
- The system shall extract clean text content from HTML pages
- The system shall preserve document structure including headings, lists, and code blocks
- The system shall remove navigation elements, headers, footers, and other UI noise
- The system shall maintain text readability and context

### FR4: Chunking Module
- The system shall split extracted text into chunks of 400-700 words
- The system shall apply 10-15% overlap between adjacent chunks
- The system shall attach metadata to each chunk: URL, page title, and section title
- The system shall preserve semantic coherence when creating chunks

### FR5: Embedding Module
- The system shall generate embeddings using Cohere's embedding model
- The system shall batch requests for efficiency
- The system shall implement retry logic for API failures
- The system shall handle rate limiting from the Cohere API

### FR6: Storage Module
- The system shall store embeddings in Qdrant vector database
- The system shall create appropriate schema for storing text chunks and metadata
- The system shall support efficient similarity search operations
- The system shall handle connection failures gracefully

### FR7: Idempotency
- The system shall support safe re-runs without creating duplicate entries
- The system shall identify and skip already processed content
- The system shall handle partial failures and resume operations

### FR8: Logging
- The system shall provide detailed progress logging during the ingestion process
- The system shall log errors and warnings appropriately
- The system shall provide summary statistics upon completion

### FR9: CLI Interface
- The system shall accept a URL parameter via `--url` command line argument
- The system shall validate the URL format before starting processing
- The system shall provide clear error messages for invalid inputs

## Success Criteria

- 95% of pages on a typical Docusaurus site are successfully crawled and processed
- Processing completes within 30 minutes for sites with up to 1000 pages
- Text extraction preserves document structure (headings, lists, code blocks) with 95% accuracy
- Embeddings are stored in Qdrant with all required metadata attached
- Re-running the pipeline on the same URL completes without duplicating entries
- At least 99% of API calls to Cohere succeed with proper retry handling
- Users can initiate the process with a single command and receive progress updates

## Key Entities

### Page Content
- URL: The source URL of the page
- Raw HTML: Original HTML content from the page
- Clean Text: Extracted text content after removing noise
- Metadata: Associated metadata (title, headings, etc.)

### Text Chunk
- Content: The text content of the chunk (400-700 words)
- Metadata: URL, page title, section title, and other contextual information
- Embedding: Vector representation of the content
- ID: Unique identifier for the chunk

### Processing State
- Visited URLs: Collection of URLs already processed
- Progress: Current state of the ingestion pipeline
- Statistics: Metrics about the processing (pages processed, chunks created, etc.)

## Constraints

- The system must handle various Docusaurus themes and configurations
- The system must comply with website terms of service and robots.txt
- The system must not overload the target server with requests
- The system must handle network timeouts and transient failures gracefully

## Assumptions

- The target site follows standard Docusaurus structure and conventions
- User has valid API credentials for Cohere and Qdrant services
- The target site is publicly accessible without authentication
- Network connectivity is available during the ingestion process