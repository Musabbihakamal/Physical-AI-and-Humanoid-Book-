# Vercel and GitHub Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics Book application to Vercel and GitHub.

## Prerequisites

1. A GitHub account
2. Repository access permissions
3. API keys for external services (OpenAI, Qdrant, etc.)
4. For Vercel deployment: A Vercel account

## Step 1: Create GitHub Repository

1. Go to GitHub.com and create a new repository
2. Name it `Physical-AI-and-Humanoid-Book` (or your preferred name)
3. Initialize with a README if desired
4. Clone the repository to your local machine:

```bash
git clone https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book.git
cd Physical-AI-and-Humanoid-Book
```

## Step 2: Push Your Code

1. Add all files to the repository:

```bash
git add .
git commit -m "Initial commit: Physical AI & Humanoid Robotics Book application"
git push origin main
```

## Step 3: Deploy to Vercel (Frontend)

### Option A: Deploy via Vercel CLI

1. Install the Vercel CLI:
```bash
npm install -g vercel
```

2. Navigate to the frontend directory:
```bash
cd frontend
```

3. Deploy to Vercel:
```bash
vercel --prod
```

4. Follow the prompts to link your GitHub account and configure the project

### Option B: Deploy via Vercel Dashboard

1. Go to [vercel.com](https://vercel.com) and sign in
2. Click "New Project" and import your GitHub repository
3. Configure the project settings:
   - Framework: Docusaurus
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Root Directory: `frontend`
4. Add environment variables in the Vercel dashboard (see Step 4)

### Option C: Deploy via Git Integration

1. Go to [vercel.com](https://vercel.com) and sign in
2. Import your GitHub repository
3. Vercel will automatically detect the Docusaurus configuration from `vercel.json`
4. Configure the build settings to point to the `frontend` directory

## Step 4: Set Up Environment Variables

### For Vercel Deployment:

1. Go to your Vercel dashboard
2. Select your project
3. Navigate to Settings > Environment Variables
4. Add the following variables:
   - `NODE_ENV` - Set to `production`
   - `REACT_APP_BACKEND_URL` - Your backend API URL (e.g., `https://your-backend-app.herokuapp.com`)

### For GitHub Deployment (Backend):

1. Go to your repository on GitHub
2. Navigate to Settings > Secrets and variables > Actions
3. Add the following secrets:
   - `OPENAI_API_KEY` - Your OpenAI API key
   - `QDRANT_URL` - Your Qdrant vector database URL
   - `QDRANT_API_KEY` - Your Qdrant API key (if required)
   - `DATABASE_URL` - Your PostgreSQL database URL
   - `SECRET_KEY` - Your application secret key
   - `POSTGRES_PASSWORD` - Your PostgreSQL password

## Step 5: Configure Vercel Settings

### Update vercel.json for Frontend:

The project already includes a `vercel.json` file in the root directory with the following configuration:

```json
{
  "version": 2,
  "framework": "docusaurus",
  "builds": [
    {
      "src": "frontend/package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ],
  "github": {
    "enabled": true,
    "autoJobCancelation": true
  }
}
```

### Update Docusaurus Configuration:

In `frontend/docusaurus.config.js`, make sure the URL and base URL are properly configured for Vercel deployment:

```js
// For Vercel deployment, update these values:
url: 'https://your-project-name.vercel.app',  // Replace with your actual Vercel URL
baseUrl: '/',  // For Vercel, typically use root path
```

## Step 6: Configure Environment Variables

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
REACT_APP_BACKEND_URL=https://your-backend-app.herokuapp.com  # Replace with your backend URL
```

## Step 7: Deploy the Application

### Frontend Deployment (Vercel)

When you deploy to Vercel, the platform will:

1. Install dependencies in the frontend directory
2. Run `npm run build` to build the Docusaurus application
3. Serve the built files from the `build` directory
4. Automatically deploy on pushes to main branch (if Git integration is used)

### Backend Deployment Options

For the backend, you still need to deploy separately:

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

## Step 8: Verification

After deployment:

1. Frontend on Vercel: Visit `https://your-project-name.vercel.app`
2. Backend: Visit your backend URL (e.g., `https://your-app.herokuapp.com/health`)

## Troubleshooting

### Vercel Deployment Issues
- Check the Vercel dashboard for build logs and error messages
- Ensure your `vercel.json` is properly configured
- Verify that the build command runs successfully in the frontend directory
- Make sure all required environment variables are set in Vercel

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
4. Vercel will automatically deploy frontend changes (if Git integration is used)
5. For backend, redeploy to your hosting platform

### Monitoring
- Monitor Vercel dashboard for deployment status
- Check your backend health endpoints regularly
- Monitor API usage for external services

## Security Best Practices

1. Never commit API keys or secrets to the repository
2. Use Vercel Environment Variables for frontend secrets
3. Use GitHub Secrets for backend secrets
4. Regularly rotate API keys
5. Enable two-factor authentication on your GitHub and Vercel accounts
6. Review access permissions regularly

## Support

For deployment issues, please:
1. Check the Vercel dashboard logs
2. Check the GitHub Actions logs
3. Verify all configuration files
4. Ensure all required services are running
5. Create an issue in the repository with detailed information