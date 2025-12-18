"""
API module for the multi-agent book generation system.
"""
from .main import app
from . import agent_routes
from . import rag_routes
from . import content_routes

__all__ = ["app"]