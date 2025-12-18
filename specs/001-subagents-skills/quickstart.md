# Quickstart: Subagents & Reusable Intelligence System

## Overview
This guide will help you set up and start using the multi-agent system for book generation and interactive learning. The system includes four specialized subagents:
- **Glossary Maker**: Auto-generate glossaries from chapter content with term linking
- **Code Explainer**: Parse and explain code with ROS 2 and Isaac Sim command highlighting
- **Quiz Creator**: Generate MCQs, short answer, and coding exercises with configurable difficulty
- **Chapter Generator**: Auto-generate structured chapters based on module focus

The system follows a modular architecture with clear separation between backend services, frontend components, and shared utilities.

## Prerequisites
- Python 3.11+
- Node.js 18+
- Access to OpenAI API (for embeddings and agent capabilities)
- Qdrant Cloud account (or local Qdrant instance)
- Neon Postgres account (or local Postgres instance)

## System Architecture

### Module Structure
The system is organized into 4 main functional modules:

1. **Glossary Maker Module** (Priority P1)
   - Auto-generates glossaries from chapter content
   - Links terms to their occurrences in text
   - Provides Docusaurus-compatible output

2. **Code Explainer Module** (Priority P1)
   - Parses code snippets and generates detailed explanations
   - Highlights ROS 2 and Isaac Sim specific commands
   - Supports multiple programming languages

3. **Quiz Creator Module** (Priority P2)
   - Generates multiple question types (MCQ, short answer, coding exercises)
   - Configures difficulty based on user profile
   - Creates adaptive assessments

4. **Chapter Generator Module** (Priority P2)
   - Generates structured chapters based on module focus
   - Includes headings, code blocks, diagrams, and exercises
   - Produces Docusaurus-compatible Markdown

### Technical Architecture
- **Backend**: FastAPI services with PostgreSQL/SQLite database
- **Frontend**: Docusaurus-based documentation site with React components
- **AI Services**: OpenAI API integration for agent capabilities
- **Storage**: Vector database (Qdrant) for RAG functionality

## Installation

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up backend services
```bash
cd backend
pip install -r requirements.txt
cp .env.example .env
# Edit .env with your API keys and database URLs
```

### 3. Set up frontend/Docusaurus
```bash
cd frontend
npm install
cp .env.example .env
# Edit .env with your backend API URL and other configuration
```

### 4. Set up shared utilities
```bash
cd shared
npm install  # if there are shared npm packages
```

## Configuration

### 1. Environment Variables
Set up the following environment variables in your `.env` files:

**Backend (.env):**
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_url
CLAUDE_API_KEY=your_claude_api_key  # if using Claude Code
```

**Frontend (.env):**
```
REACT_APP_BACKEND_URL=http://localhost:8000  # or your deployed backend URL
REACT_APP_RAG_ENABLED=true
```

### 2. Initialize the database
```bash
cd backend
python -m scripts.init_db
```

### 3. Initialize the vector store
```bash
cd backend
python -m scripts.init_vector_store
```

## Running the System

### 1. Start backend services
```bash
cd backend
uvicorn main:app --reload --port 8000
```

### 2. Start frontend
```bash
cd frontend
npm start
```

## Using the Agents

### 1. Research Agent
To use the Research Agent for gathering information:
```bash
curl -X POST http://localhost:8000/api/agents/research \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Research ROS 2 navigation stack",
    "context": "Chapter on robot navigation"
  }'
```

### 2. Writer Agent
To use the Writer Agent for generating content:
```bash
curl -X POST http://localhost:8000/api/agents/writer \
  -H "Content-Type: application/json" \
  -d '{
    "outline": ["Introduction", "Key Concepts", "Implementation"],
    "context": "Module on AI planning algorithms",
    "user_profile": {"experience_level": "INTERMEDIATE"}
  }'
```

### 3. Editor Agent
To use the Editor Agent for content improvement:
```bash
curl -X POST http://localhost:8000/api/agents/editor \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Raw content to be edited...",
    "editing_focus": ["grammar", "structure", "clarity"]
  }'
```

### 4. RAG Engineer Agent
To use the RAG Engineer Agent for RAG system tasks:
```bash
curl -X POST http://localhost:8000/api/agents/rag-engineer \
  -H "Content-Type: application/json" \
  -d '{
    "task": "create_embedding_pipeline",
    "parameters": {
      "chunk_size": 512,
      "overlap": 64
    }
  }'
```

### 5. Using the RAG Chatbot
The RAG chatbot is accessible through the frontend interface. You can also use it via the API:

```bash
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain how ROS 2 nodes communicate?",
    "session_id": "optional-session-id"
  }'
```

## Development Workflow

### 1. Creating a new book chapter
1. Use the Research Agent to gather information
2. Use the Writer Agent to generate the initial draft
3. Use the Editor Agent to refine the content
4. Review and approve the generated content
5. Add the content to the documentation system

### 2. Adding new agent capabilities
1. Define the new agent in the `agents/` directory
2. Create appropriate API endpoints
3. Update the data models if needed
4. Add tests for the new functionality

## Troubleshooting

### Common Issues

1. **API Keys Not Working**
   - Verify your API keys in the environment variables
   - Check that your OpenAI/Qdrant/Postgres accounts are active

2. **Database Connection Issues**
   - Verify your database URL is correct
   - Check that your database service is running
   - Ensure your database credentials are correct

3. **Vector Store Issues**
   - Verify your Qdrant connection details
   - Check that your Qdrant service is accessible

### Getting Help
- Check the logs in the backend service: `tail -f backend/logs/app.log`
- For frontend issues, check browser developer tools
- Review the API documentation at `/docs` endpoint

## Next Steps

1. Review the complete API documentation
2. Set up user authentication for personalized experiences
3. Configure the RAG system with your book content
4. Customize agent prompts for your specific needs
5. Set up monitoring and analytics