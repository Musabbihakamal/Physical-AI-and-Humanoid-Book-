# Backend Translation Service Setup

## Overview
This document explains the changes made to move the Urdu translation functionality from the frontend to the backend, fixing the issue where the "Initializing..." state would hang indefinitely.

## Problem
The original implementation attempted to run the translation agent directly in the browser, which is impossible since AI models require backend processing. The frontend-only agent would try to initialize but never complete, causing the UI to remain in the "Initializing..." state forever.

## Solution
1. Created a backend translation service in `backend/src/services/translation_service.py`
2. Added translation API endpoints in `backend/src/api/translation_routes.py`
3. Updated the frontend to call the backend API instead of running the agent in the browser
4. Maintained all existing features (code block preservation, constitutional compliance, etc.)

## Backend Translation Service Features
- Supports both OpenAI and Claude APIs
- Constitutional compliance filtering
- Context-aware translation (legal, educational, religious, technical)
- Batch translation capabilities
- Language detection
- Rate limiting and security validation

## API Endpoints
- `POST /api/translate` - Translate text
- `POST /api/translate-batch` - Batch translate multiple texts
- `POST /api/detect-language` - Detect language of text

## Environment Variables
Set these in your `.env` file:
- `OPENAI_API_KEY` - For OpenAI translation
- `CLAUDE_API_KEY` - For Claude translation (optional)

## Frontend Changes
- Removed frontend agent initialization logic
- Updated to call backend API with proper error handling
- Added environment-specific API URL configuration
- Maintained all content preservation features (code blocks, technical terms)
- Added comprehensive error handling and state management

## Running the System
1. Start the backend: `npm run backend` (or `cd backend && python -m src.main`)
2. Start the frontend: `npm run frontend` (or `cd frontend && npm start`)
3. The translation feature will now work properly

## Static Hosting Compatibility
For GitHub Pages or other static hosting:
1. Deploy the backend separately (e.g., on a VPS, Heroku, or other cloud platform)
2. Set `REACT_APP_API_URL` environment variable in the frontend build to point to your backend URL
3. The frontend will automatically use the configured backend URL

Example:
```bash
REACT_APP_API_URL=https://your-backend-domain.com npm run build
```