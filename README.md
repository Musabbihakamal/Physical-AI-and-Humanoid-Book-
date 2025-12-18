# Book + RAG Bot + Multi-Agent System

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Node.js](https://img.shields.io/badge/node.js-18+-green.svg)](https://nodejs.org/)
[![FastAPI](https://img.shields.io/badge/fastapi-0.104.1-red.svg)](https://fastapi.tiangolo.com/)
[![Docusaurus](https://img.shields.io/badge/docusaurus-3.9.2-blue.svg)](https://docusaurus.io/)

</div>

A comprehensive AI system that includes specialized subagents for book generation, a RAG-based chatbot trained on book content and external knowledge, and a complete book generation system using AI subagents. The system supports writing, managing, and interacting with a complete technical book using AI capabilities.

## 🚀 Features

### Multi-Agent Architecture
- **Research Agent**: Gathers accurate, verifiable information and provides citations
- **Writer Agent**: Expands outlines into full chapters with clarity and flow
- **Editor Agent**: Performs structural editing and ensures grammar and clarity
- **RAG Engineer Agent**: Builds retrieval pipelines and handles embeddings
- **Developer Agent**: Writes clean, production-ready code
- **Documentation Agent**: Creates polished, structured documentation
- **Project Planner Agent**: Breaks large tasks into steps and defines milestones

### Educational Features
- **Glossary Maker**: Automatically generates glossaries from chapter content with links to occurrences
- **Code Explainer**: Explains complex code examples with ROS 2 and Isaac Sim command highlighting
- **Quiz Creator**: Generates MCQs, short answer questions, and coding exercises with configurable difficulty
- **Chapter Generator**: Creates well-structured educational content following pedagogical best practices

### RAG System
- Vector database for efficient content retrieval
- Full-book, section, and paragraph-level retrieval
- Source citation capabilities
- Integration with Docusaurus-based documentation website

## 🏗️ Architecture

The system is built with a multi-service architecture:

```
├── backend/                    # FastAPI backend services
│   ├── src/
│   │   ├── agents/            # AI agent implementations
│   │   ├── models/            # Database models
│   │   ├── services/          # Business logic services
│   │   ├── api/               # API routes
│   │   └── database/          # Database configuration
│   ├── requirements.txt       # Python dependencies
│   └── tests/                 # Backend tests
├── frontend/                   # Docusaurus frontend
│   ├── src/
│   ├── docs/
│   ├── package.json
│   └── docusaurus.config.js
├── shared/                     # Shared utilities and prompts
├── docs/                       # Documentation files
├── scripts/                    # Initialization and utility scripts
├── data/                       # Data files and database
├── specs/                      # Specification documents
└── history/                    # Prompt History Records
```

## 🛠️ Prerequisites

- **Python** 3.11+
- **Node.js** 18+
- **Access to OpenAI API** (for embeddings and agent capabilities)
- **Qdrant Cloud account** (or local Qdrant instance)
- **PostgreSQL** (or compatible database)

## 📦 Setup

### Quick Initialization

The easiest way to set up the project is using the initialization scripts:

**On Unix/Linux/macOS:**
```bash
npm run init
# or
bash scripts/initialize.sh
```

**On Windows:**
```cmd
npm run init:windows
# or
scripts\initialize.bat
```

**Using Python (cross-platform):**
```bash
python scripts/initialize.py
```

### Manual Installation

If you prefer to set up manually:

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies:
   ```bash
   npm install
   # This will install both backend and frontend dependencies
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and database URLs
   ```

4. Set up backend services:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

5. Set up frontend/Docusaurus:
   ```bash
   cd frontend
   npm install
   ```

6. Initialize database (if needed):
   ```bash
   cd backend
   python -m scripts.init_db
   ```

## 🚀 Usage

### Running the System

**Development Mode (Recommended):**
```bash
npm run dev
# This starts both backend and frontend in development mode
```

**Separate Services:**
```bash
# Start backend only:
npm run backend
# or
cd backend && python -m src.main

# Start frontend only:
npm run frontend
# or
cd frontend && npm start
```

**Production Build:**
```bash
npm run build
npm run deploy
```

### API Documentation

Once the backend is running, API documentation is available at:
- http://localhost:8000/docs
- http://localhost:8000/redoc

### Using the Agents

The system provides API endpoints for each agent type. See the backend documentation for specific usage examples.

## 🧪 Testing

Run backend tests:
```bash
npm run test
# or
cd backend && python -m pytest tests/
```

Validate all subagents:
```bash
npm run validate
```

## 📚 Documentation

- **Initialization Guide**: `docs/INITIALIZATION.md`
- **Deployment Guide**: `docs/DEPLOYMENT.md`
- **Project Summary**: `docs/PROJECT_SUMMARY.md`
- **API Documentation**: Available at http://localhost:8000/docs when running

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Run tests (`npm test`)
6. Commit your changes (`git commit -m 'Add some amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

If you encounter any issues or have questions:
- Check the [Issues](https://github.com/your-username/your-repo/issues) page
- Review the documentation in the `docs/` directory
- Create a new issue with detailed information about your problem

## 🙏 Acknowledgments

- Thanks to the FastAPI and Docusaurus communities for excellent frameworks
- Special thanks to the OpenAI team for their powerful APIs
- Appreciation to all contributors who have helped improve this project