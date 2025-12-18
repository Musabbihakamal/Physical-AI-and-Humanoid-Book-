from setuptools import setup, find_packages

setup(
    name="book-agent-system",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        "fastapi>=0.100.0",
        "uvicorn>=0.22.0",
        "sqlalchemy>=2.0.0",
        "psycopg2-binary>=2.9.0",
        "pydantic>=2.0.0",
        "python-dotenv>=1.0.0",
        "pytest>=7.0.0",
        "pytest-asyncio>=0.21.0",
    ],
    author="AI System Developer",
    description="Book + RAG Bot + Multi-Agent System",
    python_requires=">=3.11",
)