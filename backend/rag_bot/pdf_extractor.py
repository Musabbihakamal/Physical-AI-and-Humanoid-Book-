
"""
PDF content extractor module for the ingestion pipeline
"""

import logging
from typing import List, Dict
from pathlib import Path

try:
    import PyPDF2
except ImportError:
    PyPDF2 = None


def extract_text_from_pdf(pdf_path: str) -> List[Dict[str, str]]:
    """
    Extract text content from a PDF file, page by page.

    Args:
        pdf_path: Path to the PDF file

    Returns:
        List of dictionaries containing page content and metadata
    """
    logger = logging.getLogger(__name__)

    if not PyPDF2:
        raise ImportError("PyPDF2 is not installed. Install with: pip install PyPDF2")

    pdf_file = Path(pdf_path)
    if not pdf_file.exists():
        raise FileNotFoundError(f"PDF file not found: {pdf_path}")

    logger.info(f"Extracting text from PDF: {pdf_path}")

    pages_content = []

    try:
        with open(pdf_path, 'rb') as file:
            pdf_reader = PyPDF2.PdfReader(file)
            total_pages = len(pdf_reader.pages)

            logger.info(f"PDF has {total_pages} pages")

            for page_num in range(total_pages):
                try:
                    page = pdf_reader.pages[page_num]
                    text = page.extract_text()

                    if text.strip():
                        page_data = {
                            "page_number": page_num + 1,
                            "content": text.strip(),
                            "source_file": pdf_file.name,
                            "source_path": str(pdf_file.absolute()),
                            "title": f"Page {page_num + 1}"
                        }
                        pages_content.append(page_data)
                        logger.debug(f"Extracted {len(text)} characters from page {page_num + 1}")
                    else:
                        logger.warning(f"Page {page_num + 1} has no extractable text")

                except Exception as e:
                    logger.error(f"Error extracting text from page {page_num + 1}: {str(e)}")
                    continue

            logger.info(f"Successfully extracted text from {len(pages_content)}/{total_pages} pages")

    except Exception as e:
        logger.error(f"Error reading PDF file: {str(e)}")
        raise

    return pages_content


def extract_pdf_metadata(pdf_path: str) -> Dict[str, str]:
    """
    Extract metadata from a PDF file.

    Args:
        pdf_path: Path to the PDF file

    Returns:
        Dictionary containing PDF metadata
    """
    logger = logging.getLogger(__name__)

    if not PyPDF2:
        raise ImportError("PyPDF2 is not installed. Install with: pip install PyPDF2")

    metadata = {
        "title": "",
        "author": "",
        "subject": "",
        "creator": "",
        "producer": "",
        "num_pages": 0
    }

    try:
        with open(pdf_path, 'rb') as file:
            pdf_reader = PyPDF2.PdfReader(file)

            # Get page count
            metadata["num_pages"] = len(pdf_reader.pages)

            # Get metadata if available
            if pdf_reader.metadata:
                metadata["title"] = pdf_reader.metadata.get('/Title', '') or ''
                metadata["author"] = pdf_reader.metadata.get('/Author', '') or ''
                metadata["subject"] = pdf_reader.metadata.get('/Subject', '') or ''
                metadata["creator"] = pdf_reader.metadata.get('/Creator', '') or ''
                metadata["producer"] = pdf_reader.metadata.get('/Producer', '') or ''

            logger.info(f"Extracted metadata from PDF: {metadata}")

    except Exception as e:
        logger.error(f"Error extracting PDF metadata: {str(e)}")

    return metadata
