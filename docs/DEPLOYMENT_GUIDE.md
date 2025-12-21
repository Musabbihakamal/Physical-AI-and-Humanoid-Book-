# Deployment Guide

This guide provides instructions for deploying the Book + RAG Bot + Multi-Agent System in different environments.

## 🚀 Quick Deployment

### Using Docker Compose (Recommended)

For a quick local deployment with all services:

```bash
# Build and start all services
docker-compose up -d

# Check service status
docker-compose ps

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### Production Deployment

For production deployment:

```bash
# Use the production docker-compose file
docker-compose -f docker-compose.prod.yml up -d
```

## 🏗️ Environment Configuration

### Required Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
# Database configuration
POSTGRES_PASSWORD=your_secure_password
DATABASE_URL=postgresql://postgres:your_secure_password@localhost/book_agent_db

# API keys
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
SECRET_KEY=your_very_long_secret_key

# Optional settings
DEBUG=False
LOG_LEVEL=INFO
WORKERS=4
```

## ☁️ Cloud Deployment Options

### AWS Deployment

1. Create an RDS PostgreSQL instance
2. Set up an EC2 instance with Docker
3. Configure environment variables in AWS Systems Manager Parameter Store
4. Deploy using the production docker-compose file

### Google Cloud Platform

1. Create a Cloud SQL PostgreSQL instance
2. Deploy to Google Cloud Run or Compute Engine
3. Configure environment variables in Secret Manager
4. Use the production docker-compose file

### Azure Deployment

1. Create an Azure Database for PostgreSQL
2. Deploy to Azure Container Instances or App Service
3. Configure environment variables in Azure Key Vault
4. Use the production docker-compose file

## 🌐 Production Configuration

### Nginx Configuration

The production setup includes an Nginx reverse proxy. Ensure your `nginx.conf` file is properly configured:

```nginx
upstream backend {
    server backend:8000;
}

upstream frontend {
    server frontend:3000;
}

server {
    listen 80;
    server_name your-domain.com;

    location /api/ {
        proxy_pass http://backend;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    location / {
        proxy_pass http://frontend;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

### SSL/HTTPS Setup

For production deployment, configure SSL certificates:

1. Place SSL certificates in the `ssl/` directory
2. Update the nginx configuration to use HTTPS
3. Update the docker-compose files to expose port 443

## 🗄️ Database Migration

Before deploying to production, run database migrations:

```bash
# Run migrations in the backend container
docker-compose exec backend alembic upgrade head
```

## 🧪 Health Checks

The application includes health check endpoints:

- Backend: `http://your-domain.com/health`
- Frontend: `http://your-domain.com/` (should return the main page)

## 🔒 Security Considerations

1. **Secret Management**: Never commit secrets to version control
2. **HTTPS**: Always use HTTPS in production
3. **Database Security**: Use strong passwords and restrict database access
4. **API Key Security**: Rotate API keys regularly
5. **Container Security**: Keep base images updated

## 🚢 CI/CD Pipeline

### GitHub Actions Example

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to Production

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3

    - name: Deploy to server
      uses: appleboy/ssh-action@v0.1.5
      with:
        host: ${{ secrets.HOST }}
        username: ${{ secrets.USERNAME }}
        key: ${{ secrets.KEY }}
        script: |
          cd /path/to/your/app
          git pull origin main
          docker-compose -f docker-compose.prod.yml up -d --build
```

## 📊 Monitoring and Logging

### Log Management

Logs are available in the containers:

```bash
# View backend logs
docker-compose logs backend

# View frontend logs
docker-compose logs frontend

# View all logs in real-time
docker-compose logs -f
```

### Health Monitoring

Set up monitoring for:
- Container health status
- Database connectivity
- API response times
- Error rates

## 🔄 Scaling

### Horizontal Scaling

The application can be scaled horizontally:

```bash
# Scale backend services
docker-compose up -d --scale backend=3

# Scale frontend services
docker-compose up -d --scale frontend=2
```

### Database Scaling

Consider these options for database scaling:
- Read replicas for PostgreSQL
- Connection pooling
- Database sharding for large datasets

## 🧹 Maintenance

### Regular Maintenance Tasks

1. **Database Backup**: Regularly backup your database
2. **Log Rotation**: Set up log rotation to prevent disk space issues
3. **Security Updates**: Keep all dependencies updated
4. **Performance Monitoring**: Monitor application performance

### Backup and Restore

```bash
# Backup database
docker-compose exec postgres pg_dump -U postgres book_agent_db > backup.sql

# Restore database
cat backup.sql | docker-compose exec -T postgres psql -U postgres book_agent_db
```

## 🆘 Troubleshooting

### Common Issues

1. **Port Conflicts**: Check if required ports are already in use
2. **Database Connection**: Verify database URL and credentials
3. **API Keys**: Ensure all required API keys are set
4. **Container Dependencies**: Check service startup order

### Debugging Production Issues

```bash
# Check container status
docker-compose ps

# View detailed logs
docker-compose logs --tail=100 service_name

# Access container shell
docker-compose exec service_name bash
```

## 📋 Deployment Checklist

- [ ] Environment variables configured securely
- [ ] SSL certificates in place
- [ ] Database backup strategy implemented
- [ ] Monitoring and alerting configured
- [ ] Security settings reviewed
- [ ] Load testing completed
- [ ] Rollback plan prepared
- [ ] Documentation updated