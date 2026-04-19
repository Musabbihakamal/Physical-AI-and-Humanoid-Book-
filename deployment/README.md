# Deployment Files

This directory contains deployment configurations for various platforms.

## Docker Deployment

### Files
- `Dockerfile` - Main Docker configuration
- `Dockerfile.backend` - Backend-specific Docker configuration
- `Dockerfile.frontend` - Frontend-specific Docker configuration
- `docker-compose.yml` - Development Docker Compose setup
- `docker-compose.prod.yml` - Production Docker Compose setup
- `nginx.conf` - Nginx reverse proxy configuration

### Usage

**Development:**
```bash
docker-compose up -d
```

**Production:**
```bash
docker-compose -f docker-compose.prod.yml up -d
```

## Platform-Specific Deployments

### Heroku
- `Procfile` - Heroku process configuration

### Linux Systemd
- `book-agent-system.service` - Systemd service file for Linux servers

## Notes

These files are for production deployment. For local development, use the batch scripts in the `backend/` directory:
- `backend/setup.bat` - Setup
- `backend/start.bat` - Start server
- `backend/ingest.bat` - Ingest content

See `docs/guides/DEPLOY_RAG_TO_PRODUCTION.md` for detailed deployment instructions.
