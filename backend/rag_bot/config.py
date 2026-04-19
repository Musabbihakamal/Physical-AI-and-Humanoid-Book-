"""
Configuration module for the RAG Bot
"""

import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Default configuration
DEFAULT_CONFIG = {
    "qdrant_url": os.getenv("QDRANT_URL", "https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io"),
    "cohere_model": os.getenv("COHERE_MODEL", "command-r-plus"),  # Recommended for RAG applications
    "max_context_chunks": int(os.getenv("MAX_CONTEXT_CHUNKS", "5")),
    "similarity_threshold": float(os.getenv("SIMILARITY_THRESHOLD", "0.3")),
    "temperature": float(os.getenv("TEMPERATURE", "0.3")),
    "log_level": os.getenv("LOG_LEVEL", "INFO"),
    "chunk_min_words": int(os.getenv("CHUNK_MIN_WORDS", "400")),
    "chunk_max_words": int(os.getenv("CHUNK_MAX_WORDS", "700")),
    "overlap_percent": int(os.getenv("OVERLAP_PERCENT", "10")),
    "batch_size": int(os.getenv("BATCH_SIZE", "96")),
    "request_delay": float(os.getenv("REQUEST_DELAY", "1.0")),
    "max_retries": int(os.getenv("MAX_RETRIES", "3")),
    "timeout": int(os.getenv("TIMEOUT", "30")),
}

def get_config(key: str, default=None):
    """
    Get configuration value by key

    Args:
        key: Configuration key
        default: Default value if key not found

    Returns:
        Configuration value
    """
    return DEFAULT_CONFIG.get(key, default)

def update_config_from_env():
    """
    Update configuration from environment variables
    """
    for key in DEFAULT_CONFIG.keys():
        env_value = os.getenv(key.upper())
        if env_value is not None:
            # Type conversion based on original type
            original_value = DEFAULT_CONFIG[key]
            if isinstance(original_value, int):
                DEFAULT_CONFIG[key] = int(env_value)
            elif isinstance(original_value, float):
                DEFAULT_CONFIG[key] = float(env_value)
            elif isinstance(original_value, bool):
                DEFAULT_CONFIG[key] = env_value.lower() in ('true', '1', 'yes', 'on')
            else:
                DEFAULT_CONFIG[key] = env_value

# Update config from environment on import
update_config_from_env()