---
sidebar_position: 10
title: "Deployment Guide"
---

# Deployment Guide for Physical AI & Humanoid Robotics Book

## Overview

This guide provides instructions for deploying the complete Physical AI & Humanoid Robotics educational system, including the Docusaurus documentation website, backend services, and integrated AI agents.

## Prerequisites

Before deploying the system, ensure you have:

- **Node.js** (v18 or higher)
- **Python** (v3.11 or higher)
- **Git**
- **Docker** (optional, for containerized deployment)
- **Access to OpenAI API** (for AI agent capabilities)
- **Qdrant Cloud account** (or local Qdrant instance) for vector storage
- **PostgreSQL database** (Neon or local instance)

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Backend Services

Navigate to the backend directory and create a virtual environment:

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt
```

Create and configure the environment file:

```bash
cp .env.example .env
```

Edit the `.env` file with your API keys and database URLs:

```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
SECRET_KEY=your_secret_key
```

### 3. Set Up Frontend

Navigate to the frontend directory:

```bash
cd frontend
npm install
```

Create and configure the frontend environment file:

```bash
cp .env.example .env
```

Edit the `.env` file with your backend API URL:

```env
REACT_APP_BACKEND_URL=http://localhost:8000
REACT_APP_OPENAI_API_KEY=your_openai_api_key
```

### 4. Initialize Database

Run the database initialization script:

```bash
cd backend
python -m scripts.init_db
```

## Running the System

### Development Mode

To run the system in development mode with both frontend and backend:

```bash
# From the project root
npm run dev
```

This will start both the backend server and the frontend development server.

### Production Build

To build the system for production:

```bash
# Build the frontend
cd frontend
npm run build

# The backend can be run with uvicorn in production
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## Docker Deployment

### Building Docker Images

Create a `Dockerfile` for the backend:

```Dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ ./backend/
COPY shared/ ./shared/

WORKDIR /app/backend

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Create a `Dockerfile` for the frontend:

```Dockerfile
FROM node:18-alpine

WORKDIR /app

COPY frontend/package*.json ./
RUN npm install

COPY frontend/ ./

RUN npm run build

EXPOSE 3000

CMD ["npm", "start"]
```

### Docker Compose

Create a `docker-compose.yml` file:

```yaml
version: '3.8'

services:
  backend:
    build:
      context: .
      dockerfile: Dockerfile.backend
    ports:
      - "8000:8000"
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - DATABASE_URL=${DATABASE_URL}
    depends_on:
      - postgres
      - qdrant

  frontend:
    build:
      context: .
      dockerfile: Dockerfile.frontend
    ports:
      - "3000:3000"
    environment:
      - REACT_APP_BACKEND_URL=http://localhost:8000
    depends_on:
      - backend

  postgres:
    image: postgres:15
    environment:
      POSTGRES_DB: book_agent_db
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: password
    volumes:
      - postgres_data:/var/lib/postgresql/data
    ports:
      - "5432:5432"

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - qdrant_data:/qdrant/storage

volumes:
  postgres_data:
  qdrant_data:
```

To run with Docker Compose:

```bash
docker-compose up -d
```

## Cloud Deployment

### Deploying to AWS

1. **EC2 Instance Setup**:
   - Launch an EC2 instance with Ubuntu
   - Install Docker and Docker Compose
   - Clone the repository
   - Set up environment variables
   - Run with Docker Compose

2. **Elastic Beanstalk**:
   - Package the application
   - Deploy to Elastic Beanstalk
   - Configure environment variables

### Deploying to Google Cloud Platform

1. **Compute Engine**:
   - Create a VM instance
   - Install required dependencies
   - Deploy the application

2. **Google App Engine**:
   - Configure app.yaml
   - Deploy using gcloud CLI

### Deploying to Azure

1. **Azure App Service**:
   - Create a web app
   - Deploy using Azure CLI or portal

2. **Azure Container Instances**:
   - Deploy containerized application

## Configuration

### Environment Variables

The system requires several environment variables:

**Backend (.env)**:
- `OPENAI_API_KEY`: API key for OpenAI services
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `DATABASE_URL`: PostgreSQL database connection string
- `SECRET_KEY`: Secret key for security
- `DEBUG`: Set to "True" for development

**Frontend (.env)**:
- `REACT_APP_BACKEND_URL`: URL of the backend API
- `REACT_APP_OPENAI_API_KEY`: OpenAI API key (if needed on frontend)

### Database Migrations

To run database migrations:

```bash
cd backend
python -m alembic upgrade head
```

To create new migrations:

```bash
cd backend
python -m alembic revision --autogenerate -m "migration message"
python -m alembic upgrade head
```

## Monitoring and Logging

### Application Logs

Backend logs are available at:
- Console output when running locally
- Docker logs when running in containers

Frontend logs:
- Browser console
- Build logs during compilation

### Health Checks

The backend provides health check endpoints:
- `GET /health`: Overall system health
- `GET /health/db`: Database connectivity
- `GET /health/vector`: Vector database connectivity

### Performance Monitoring

The system includes performance monitoring for:
- API response times
- Database query performance
- Vector search performance
- Memory and CPU usage

## Security Considerations

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

## Scaling

### Horizontal Scaling

The system supports horizontal scaling:
- Multiple backend instances behind a load balancer
- Database read replicas
- CDN for static assets
- Auto-scaling groups in cloud environments

### Database Scaling

- Connection pooling
- Read replicas
- Sharding for large datasets
- Proper indexing strategies

## Troubleshooting

### Common Issues

1. **Port Conflicts**:
   - Check if ports 8000 (backend) and 3000 (frontend) are available
   - Kill processes using `lsof -ti:8000` or `lsof -ti:3000`

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

Enable debug mode by setting `DEBUG=True` in the backend environment.

Check logs for error messages and stack traces.

Use the development tools in your browser for frontend issues.

## Backup and Recovery

### Database Backup

Regular database backups should be scheduled:

```bash
pg_dump -h hostname -U username -d database_name > backup.sql
```

### File Backup

Backup important files and configurations regularly:
- Environment files
- Database dumps
- Configuration files

## Updates and Maintenance

### Updating Dependencies

Regularly update dependencies:

```bash
# Backend
cd backend
pip install --upgrade -r requirements.txt

# Frontend
cd frontend
npm update
```

### System Maintenance

- Regular security updates
- Database optimization
- Log rotation
- Performance monitoring