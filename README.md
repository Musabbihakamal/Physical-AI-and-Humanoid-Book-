# Physical AI and Humanoid Robotics - Educational System

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Node.js](https://img.shields.io/badge/node.js-18+-green.svg)](https://nodejs.org/)
[![FastAPI](https://img.shields.io/badge/fastapi-0.104.1-red.svg)](https://fastapi.tiangolo.com/)
[![Docusaurus](https://img.shields.io/badge/docusaurus-3.1.0-blue.svg)](https://docusaurus.io/)

</div>

Comprehensive educational platform for Physical AI & Humanoid Robotics with RAG-powered chatbot, multi-agent architecture, and Urdu translation capabilities.

## 🎯 Overview

This project provides a complete learning ecosystem for robotics and AI, featuring:

- **6 Educational Modules** covering ROS 2, Digital Twins, NVIDIA Isaac, and VLA systems
- **RAG Chatbot** for intelligent Q&A with source citations
- **Ingestion Pipeline** for automated content processing and embedding
- **Multi-Agent System** with specialized AI agents for content generation
- **Urdu Translation** with technical term preservation
- **Interactive Documentation** built with Docusaurus

## 🚀 Quick Start

### Prerequisites

- Python 3.11+
- Node.js 18+
- Cohere API key (for RAG and embeddings)
- Qdrant instance (vector database)

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install dependencies
npm install

# Configure environment
cp .env.example .env
# Edit .env with your API keys
```

### Running the System

```bash
# Start both backend and frontend
npm run dev

# Or run separately:
npm run backend  # Backend API on http://localhost:8000
npm run frontend # Frontend on http://localhost:3000
```

### RAG Pipeline Usage

```bash
# Ingest content from Docusaurus site
python backend/main.py --mode ingest --url http://localhost:3000

# Interactive chat mode
python backend/main.py --mode chat

# Single question
python backend/main.py --mode ask --query "What is ROS 2?"
```

## 📁 Project Structure

```
├── backend/                    # FastAPI backend services
│   ├── main.py                # Unified RAG bot and ingestion pipeline
│   ├── test_pipeline.py       # Pipeline tests
│   ├── rag_bot/               # Modular RAG components
│   │   ├── crawler.py         # Web crawler
│   │   ├── extractor.py       # Text extraction
│   │   ├── chunker.py         # Content chunking
│   │   ├── embedder.py        # Embedding generation
│   │   └── storage.py         # Vector storage
│   └── src/                   # FastAPI application
│       ├── api/               # API routes
│       ├── models/            # Database models
│       └── config/            # Configuration
├── frontend/                   # Docusaurus documentation site
│   ├── docs/                  # Educational content (6 modules)
│   ├── src/
│   │   ├── components/        # React components
│   │   │   ├── RAGChatWidget/ # Chat interface
│   │   │   └── TranslateButton/ # Urdu translation
│   │   └── contexts/          # React contexts
│   └── docusaurus.config.js
├── docs/                       # Project documentation
│   ├── README.md              # Documentation index
│   ├── DEPLOYMENT.md          # Deployment guide
│   ├── PROJECT_SUMMARY.md     # Project overview
│   └── HACKATHON_COMPLETION_REPORT.md
├── specs/                      # Feature specifications
├── history/                    # Prompt history records
├── scripts/                    # Utility scripts
└── .specify/                   # SpecKit Plus templates
```

## 🎓 Educational Content

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 architecture and fundamentals
- Nodes, topics, publishers & subscribers
- Robot control with rclpy
- URDF modeling for humanoids

### Module 2: Digital Twin Simulation
- Gazebo simulation environment
- URDF and SDF formats
- Physics and sensor simulation
- Unity robotics visualization

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Isaac Sim foundations
- Perception pipelines
- VSLAM and navigation
- AI model training in simulation

### Module 4: Vision-Language-Action & Capstone
- VLA system architecture
- Voice-to-action with Whisper
- Multimodal fusion
- Humanoid AI companion project

### Modules 5-6: Advanced Topics (Bonus)
- Advanced control & locomotion
- Safety, ethics & human-robot interaction

## 🤖 RAG System Architecture

```
User Query
    ↓
RAG Chat Widget (Frontend)
    ↓
FastAPI Backend (/api/rag/query)
    ↓
RagBot Class
    ↓
┌─────────────────┬──────────────────┐
│                 │                  │
Qdrant Vector DB  Cohere API
(Semantic Search) (Answer Generation)
│                 │
└─────────────────┴──────────────────┘
    ↓
Response + Source Citations
```

## 🔧 Key Features

### RAG Chatbot
- Semantic search with Qdrant vector database
- Context-aware responses using Cohere
- Source citations with URLs
- Configurable similarity threshold
- Interactive and single-query modes

### Ingestion Pipeline
- Docusaurus site crawler with robots.txt compliance
- Clean text extraction preserving structure
- Smart chunking (400-700 words, 10-15% overlap)
- Cohere embeddings with batching
- Idempotent storage via content hashing

### Translation System
- In-browser Urdu translation
- Preserves code blocks, diagrams, and technical terms
- Google Translate API integration
- Toggle between English and Urdu

### Authentication
- Email/password authentication
- OAuth support (Google, GitHub)
- User profile management
- Experience level tracking

## 📚 Documentation

- **[Documentation Index](docs/README.md)** - Complete documentation overview
- **[Deployment Guide](docs/DEPLOYMENT.md)** - Production deployment
- **[Project Summary](docs/PROJECT_SUMMARY.md)** - Detailed project overview
- **[Hackathon Report](docs/HACKATHON_COMPLETION_REPORT.md)** - Completion analysis
- **[API Documentation](http://localhost:8000/docs)** - Interactive API docs (when running)

## 🧪 Testing

```bash
# Run backend tests
cd backend
python test_pipeline.py

# Test ingestion pipeline
python main.py --mode ingest --url http://localhost:3000

# Test RAG chatbot
python main.py --mode ask --query "Explain ROS 2 nodes"
```

## 🚢 Deployment

### Docker Compose (Recommended)

```bash
# Start all services
docker-compose up -d

# Check status
docker-compose ps

# View logs
docker-compose logs -f
```

### GitHub Pages (Frontend Only)

The frontend is configured for GitHub Pages deployment via GitHub Actions. Push to `main` branch to trigger automatic deployment.

## 🛠️ Technology Stack

### Backend
- **FastAPI** - Modern Python web framework
- **Cohere** - Embeddings and text generation
- **Qdrant** - Vector database for semantic search
- **SQLAlchemy** - Database ORM
- **Pydantic** - Data validation

### Frontend
- **Docusaurus 3.1.0** - Documentation framework
- **React 18.2.0** - UI library
- **Axios** - HTTP client
- **CSS Modules** - Component styling

### Infrastructure
- **Docker** - Containerization
- **GitHub Actions** - CI/CD
- **Nginx** - Reverse proxy (production)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- FastAPI and Docusaurus communities
- Cohere for powerful AI APIs
- NVIDIA for Isaac Sim and robotics tools
- Open Robotics for ROS 2

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-username/your-repo/issues)
- **Documentation**: [docs/](docs/)
- **API Docs**: http://localhost:8000/docs (when running)

---

**Built with ❤️ for robotics education and AI research**
