# Project Initialization Guide

This guide provides comprehensive instructions for initializing the Book + RAG Bot + Multi-Agent System project.

## Prerequisites

Before initializing the project, ensure you have the following installed:

- **Python 3.11+** (with pip)
- **Node.js 18+** (with npm)
- **Git** (for version control)
- **Access to OpenAI API** (for embeddings and agent capabilities)
- **Qdrant Cloud account** (or local Qdrant instance) - optional
- **PostgreSQL** (or compatible database) - optional

## Quick Start

The fastest way to initialize the project is using one of the initialization scripts:

### On Unix/Linux/macOS:
```bash
# Make the script executable
chmod +x initialize.sh

# Run the initialization script
./initialize.sh
```

### On Windows:
```cmd
# Run the initialization batch file
initialize.bat
```

## Manual Initialization

If you prefer to set up the project manually, follow these steps:

### 1. Backend Setup

```bash
# Navigate to the backend directory
cd backend

# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Unix/Linux/macOS:
source venv/bin/activate
# On Windows:
venv\Scripts\activate

# Upgrade pip
python -m pip install --upgrade pip

# Install dependencies
pip install -r requirements.txt
```

### 2. Frontend Setup

```bash
# Navigate to the frontend directory
cd frontend

# Install dependencies
npm install
```

### 3. Environment Configuration

Create and configure environment files:

```bash
# In backend directory
cp .env.example .env
# Edit .env with your API keys and database URLs

# In frontend directory
cp .env.example .env
# Edit .env with your backend API URL and other configuration
```

### 4. Database Initialization

```bash
# From the backend directory with virtual environment activated
python -m scripts.init_db
```

## Project Structure

```
project-root/
├── backend/                 # FastAPI backend services
│   ├── src/                 # Source code
│   │   ├── agents/          # AI agent implementations
│   │   ├── models/          # Database models
│   │   ├── services/        # Business logic services
│   │   ├── api/             # API routes
│   │   └── database/        # Database configuration
│   ├── requirements.txt     # Python dependencies
│   └── tests/               # Backend tests
├── frontend/                # Docusaurus frontend
│   ├── src/                 # Frontend source
│   ├── docs/                # Documentation
│   ├── package.json         # Node.js dependencies
│   └── docusaurus.config.js # Docusaurus configuration
├── shared/                  # Shared utilities and prompts
├── specs/                   # Specification documents
├── history/                 # Prompt History Records
├── initialize.sh            # Unix initialization script
├── initialize.bat           # Windows initialization script
├── start_backend.py         # Backend startup script
├── README.md               # Project overview
└── package.json            # Root package configuration
```

## Available Scripts

After initialization, you can use the following npm scripts:

### Development
```bash
# Initialize the project (Unix)
npm run init

# Initialize the project (Windows)
npm run init:windows

# Install all dependencies
npm run setup

# Start both backend and frontend in development mode
npm run dev

# Start backend only
npm run backend

# Start frontend only
npm run frontend
```

### Testing & Validation
```bash
# Run backend tests
npm run test

# Build documentation
npm run docs

# Validate all subagents
npm run validate
```

## Environment Variables

### Backend (.env)
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_url
CLAUDE_API_KEY=your_claude_api_key
```

### Frontend (.env)
```
REACT_APP_BACKEND_URL=http://localhost:8000
REACT_APP_RAG_ENABLED=true
```

## Next Steps

1. Update environment variables in `backend/.env` and `frontend/.env`
2. Start the backend: `npm run backend` or `cd backend && python -m src.main`
3. Start the frontend: `npm run frontend` or `cd frontend && npm start`
4. Access the application at http://localhost:3000
5. API documentation available at http://localhost:8000/docs

## Troubleshooting

### Common Issues

1. **Python version mismatch**: Ensure you're using Python 3.11+
2. **Node.js version mismatch**: Ensure you're using Node.js 18+
3. **Dependency installation fails**: Check your internet connection and package managers
4. **Port already in use**: Change the port in the configuration files
5. **API key issues**: Verify your API keys in the environment files

### Getting Help

- Check logs in `backend/logs/` for backend issues
- Use browser developer tools for frontend issues
- Review API documentation at `/docs` endpoint
- Consult the specification documents in `specs/` directory

## Development Workflow

1. The project follows a specification-driven development approach
2. Tasks are organized by user stories in the `specs/` directory
3. All changes should be validated with the subagent validation script
4. Use the Claude Code tools for AI-assisted development