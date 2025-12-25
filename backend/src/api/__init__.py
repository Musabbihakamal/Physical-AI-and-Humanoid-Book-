"""
API module.
"""
from .main import app
from . import rag_routes
from . import content_routes

__all__ = ["app"]