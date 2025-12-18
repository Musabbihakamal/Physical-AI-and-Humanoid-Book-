# Physical AI & Humanoid Robotics Book - Deployment Guide

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Prerequisites](#prerequisites)
4. [Quick Start](#quick-start)
5. [Configuration](#configuration)
6. [Production Deployment](#production-deployment)
7. [Monitoring](#monitoring)
8. [Troubleshooting](#troubleshooting)

## Overview

This repository contains a complete educational system for Physical AI & Humanoid Robotics, featuring:
- Interactive documentation website built with Docusaurus
- AI-powered content generation agents
- RAG (Retrieval Augmented Generation) chatbot
- Multi-agent architecture for book generation
- Integration with robotics frameworks (ROS 2, Isaac Sim, etc.)

## Architecture

The system consists of:

- **Frontend**: Docusaurus-based documentation website with embedded agent widgets
- **Backend**: FastAPI services for agents and RAG functionality
- **Database**: PostgreSQL for structured data, Qdrant for vector storage
- **Shared**: Common prompts, utilities, and types

## Prerequisites

- **Docker** and **Docker Compose** (recommended for deployment)
- **Node.js** 18+ (for local development)
- **Python** 3.11+ (for local development)
- **Access to OpenAI API** (for AI capabilities)
- **Qdrant Cloud account** or local Qdrant instance
- **PostgreSQL database** (Neon or local instance)

## Quick Start

### Using Docker Compose (Recommended)

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Create environment file:
```bash
cp .env.example .env
```

3. Edit `.env` with your API keys and database URLs:
```bash
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
POSTGRES_PASSWORD=your_secure_password
SECRET_KEY=your_secret_key
```

4. Start the system:
```bash
docker-compose up -d
```

5. Access the system:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000

### Local Development

1. Set up backend:
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

2. Set up frontend:
```bash
cd frontend
npm install
```

3. Start both services:
```bash
# Terminal 1 - Backend
cd backend
python -m src.main

# Terminal 2 - Frontend
cd frontend
npm start
```

## Configuration

### Environment Variables

**Backend (.env)**:
- `OPENAI_API_KEY`: OpenAI API key for AI capabilities
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `DATABASE_URL`: PostgreSQL database connection string
- `SECRET_KEY`: Secret key for security
- `DEBUG`: Enable/disable debug mode (True/False)
- `REDIS_URL`: Redis connection string (optional)

**Frontend (.env)**:
- `REACT_APP_BACKEND_URL`: URL of the backend API
- `REACT_APP_OPENAI_API_KEY`: OpenAI API key (if needed on frontend)

### Database Setup

The system uses PostgreSQL for structured data and Qdrant for vector storage:

1. Initialize the database:
```bash
cd backend
python -m scripts.init_db
```

2. Run migrations:
```bash
cd backend
python -m alembic upgrade head
```

### Agent Configuration

The system includes multiple specialized agents:
- Research Agent: Gathers accurate, verifiable information
- Writer Agent: Expands outlines into full chapters
- Editor Agent: Performs structural editing
- RAG Engineer Agent: Builds retrieval pipelines
- Developer Agent: Writes production-ready code
- Documentation Agent: Creates polished documentation
- Project Planner Agent: Breaks tasks into steps

## Production Deployment

### Docker Compose Production

For production deployment with Docker Compose:

1. Create a production `.env` file with secure values
2. Use the production compose file:
```bash
docker-compose -f docker-compose.yml up -d
```

3. Set up a reverse proxy (nginx/Apache) with SSL

### Cloud Deployment Options

#### AWS Deployment

1. **ECS with Fargate**:
   - Create ECS cluster
   - Deploy services using the provided Docker images
   - Set up Application Load Balancer

2. **Elastic Beanstalk**:
   - Package the application
   - Deploy to Elastic Beanstalk
   - Configure environment variables

#### Google Cloud Platform

1. **Cloud Run**:
   - Build container images
   - Deploy to Cloud Run
   - Set up Cloud SQL for database

2. **GKE**:
   - Create GKE cluster
   - Deploy using Kubernetes manifests

#### Azure

1. **Azure Container Instances**:
   - Deploy containerized application
   - Use Azure Database for PostgreSQL

2. **Azure App Service**:
   - Deploy web application
   - Configure custom deployment

### Kubernetes Deployment

For Kubernetes deployment, create the following manifests:

1. **Deployments** for each service
2. **Services** for internal communication
3. **Ingress** for external access
4. **PersistentVolumes** for data storage
5. **Secrets** for environment variables

## Monitoring

### Health Checks

The backend provides health check endpoints:
- `GET /health`: Overall system health
- `GET /health/db`: Database connectivity
- `GET /health/vector`: Vector database connectivity
- `GET /health/agents`: Agent availability

### Logging

- Backend logs are available through Docker logs
- Frontend logs are available in browser console
- Structured logging with JSON format for analysis

### Performance Metrics

The system includes:
- API response time monitoring
- Database query performance
- Vector search performance
- Memory and CPU usage

## Security

### API Security

- Use HTTPS in production
- Implement rate limiting
- Validate all inputs
- Use environment variables for secrets
- Implement proper authentication

### Data Security

- Encrypt sensitive data in transit
- Use secure database connections
- Implement proper access controls
- Regular security audits

### Container Security

- Use minimal base images
- Run containers as non-root users
- Implement resource limits
- Scan images for vulnerabilities

## Backup and Recovery

### Database Backup

Regular database backups:
```bash
pg_dump -h hostname -U username -d database_name > backup.sql
```

### File Backup

Backup important files:
- Environment files
- Database dumps
- Configuration files
- SSL certificates

## Scaling

### Horizontal Scaling

The system supports horizontal scaling:
- Multiple backend instances behind load balancer
- Database read replicas
- CDN for static assets
- Auto-scaling in cloud environments

### Database Scaling

- Connection pooling
- Read replicas
- Sharding for large datasets
- Proper indexing strategies

## Troubleshooting

### Common Issues

1. **Port Conflicts**:
   - Check if ports 8000 (backend) and 3000 (frontend) are available
   - Use `lsof -ti:8000` or `lsof -ti:3000` to find processes

2. **Environment Variables**:
   - Ensure all required environment variables are set
   - Check for typos in variable names

3. **Database Connection**:
   - Verify database URL is correct
   - Check network connectivity
   - Ensure database is running

4. **API Limits**:
   - Check OpenAI API usage limits
   - Implement retry logic for API calls

### Debugging

Enable debug mode by setting `DEBUG=True` in environment.

Check Docker logs:
```bash
docker-compose logs -f
```

Use browser development tools for frontend issues.

### Health Checks

Monitor service health:
```bash
curl http://localhost:8000/health
```

## Updates and Maintenance

### Updating Dependencies

Regularly update dependencies:

Backend:
```bash
cd backend
pip install --upgrade -r requirements.txt
```

Frontend:
```bash
cd frontend
npm update
```

### System Maintenance

- Regular security updates
- Database optimization
- Log rotation
- Performance monitoring
- Backup verification

## Support

For support, please:
1. Check the troubleshooting section
2. Review the logs for error messages
3. Create an issue in the repository
4. Contact the development team

## Contributing

See the contributing guidelines in the `.specify/` directory for detailed information about the development process.

---

This deployment guide provides comprehensive instructions for setting up, configuring, and maintaining the Physical AI & Humanoid Robotics educational system in various environments.