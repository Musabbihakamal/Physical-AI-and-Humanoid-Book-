# GitHub Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics Book application to GitHub.

## Prerequisites

1. A GitHub account
2. Repository access permissions
3. API keys for external services (OpenAI, Qdrant, etc.)

## Step 1: Create GitHub Repository

1. Go to GitHub.com and create a new repository
2. Name it `Physical-AI-and-Humanoid-Book` (or your preferred name)
3. Initialize with a README if desired
4. Clone the repository to your local machine:

```bash
git clone https://github.com/your-username/Physical-AI-and-Humanoid-Book.git
cd Physical-AI-and-Humanoid-Book
```

## Step 2: Push Your Code

1. Add all files to the repository:

```bash
git add .
git commit -m "Initial commit: Physical AI & Humanoid Robotics Book application"
git push origin main
```

## Step 3: Configure GitHub Pages for Frontend

1. Go to your repository on GitHub
2. Navigate to Settings > Pages
3. Under "Source", select "GitHub Actions"
4. This will use the workflow in `.github/workflows/deploy-frontend.yml`

## Step 4: Set Up GitHub Secrets

For the backend to work properly, you need to configure secrets in your GitHub repository:

1. Go to your repository on GitHub
2. Navigate to Settings > Secrets and variables > Actions
3. Add the following secrets:
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `QDRANT_URL` - Your Qdrant vector database URL
   - `QDRANT_API_KEY` - Your Qdrant API key (if required)
   - `DATABASE_URL` - Your PostgreSQL database URL
   - `SECRET_KEY` - Your application secret key
   - `POSTGRES_PASSWORD` - Your PostgreSQL password

## Step 5: Configure Environment Variables

Create a `.env` file in the root of your repository with the following structure (but don't commit it to the repository):

```bash
# Backend configuration
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
SECRET_KEY=your_secret_key
POSTGRES_PASSWORD=your_secure_password
DEBUG=False

# Frontend configuration
REACT_APP_BACKEND_URL=https://your-username.github.io/Physical-AI-and-Humanoid-Book
```

## Step 6: Deploy the Application

### Frontend Deployment (GitHub Pages)

The frontend will automatically deploy when you push to the main branch. The GitHub Actions workflow will:

1. Install dependencies
2. Build the Docusaurus application
3. Deploy to GitHub Pages

### Backend Deployment Options

Since GitHub doesn't host backend applications directly, you have several options:

#### Option A: Deploy to Heroku
1. Create a Heroku account
2. Install Heroku CLI
3. Create a new Heroku app
4. Set the same environment variables in Heroku settings
5. Deploy using the Heroku CLI

#### Option B: Deploy to Railway
1. Create a Railway account
2. Connect your GitHub repository
3. Set environment variables in Railway
4. Deploy automatically on pushes to main branch

#### Option C: Deploy to Render
1. Create a Render account
2. Connect your GitHub repository
3. Configure the web service with the Dockerfile
4. Set environment variables

## Step 7: Update Configuration Files

Before deployment, make sure to update the following files with your actual repository information:

1. `frontend/docusaurus.config.js` - Update `url`, `baseUrl`, `organizationName`, and `projectName`
2. Update the GitHub links in the navigation bar
3. Make sure the `editUrl` points to your repository

## Step 8: Verification

After deployment:

1. Frontend: Visit `https://your-username.github.io/Physical-AI-and-Humanoid-Book/`
2. Backend: Visit your backend URL (e.g., `https://your-app.herokuapp.com/health`)

## Troubleshooting

### GitHub Pages Not Deploying
- Check the Actions tab for workflow errors
- Ensure your `baseUrl` in `docusaurus.config.js` matches your repository name
- Verify that the build workflow completed successfully

### Backend Issues
- Check that all required environment variables are set
- Verify that your API keys are valid
- Check the health endpoint to ensure the backend is running

### Build Failures
- Ensure all dependencies are properly specified
- Check that the Node.js and Python versions match your local setup
- Verify that there are no syntax errors in configuration files

## Maintenance

### Updating the Application
1. Make changes to your local repository
2. Test locally using `npm run dev`
3. Commit and push changes to the main branch
4. GitHub Actions will automatically deploy frontend changes
5. For backend, redeploy to your hosting platform

### Monitoring
- Monitor GitHub Actions for deployment status
- Check your backend health endpoints regularly
- Monitor API usage for external services

## Security Best Practices

1. Never commit API keys or secrets to the repository
2. Use GitHub Secrets for sensitive information
3. Regularly rotate API keys
4. Enable two-factor authentication on your GitHub account
5. Review access permissions regularly

## Support

For deployment issues, please:
1. Check the GitHub Actions logs
2. Verify all configuration files
3. Ensure all required services are running
4. Create an issue in the repository with detailed information