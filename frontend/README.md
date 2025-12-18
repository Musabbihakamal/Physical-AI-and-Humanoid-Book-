# Book + RAG Bot + Multi-Agent System - Frontend

This is the frontend Docusaurus-based documentation website for the multi-agent book generation system, featuring embedded agent widgets for glossary generation, code explanation, quiz creation, and chapter generation.

## Features

- Docusaurus-based documentation website
- Embedded agent widgets for interactive learning
- Glossary maker for automatic term definition
- Code explainer with ROS 2 and Isaac Sim highlighting
- Quiz creator with configurable difficulty
- Chapter generator for educational content

## Tech Stack

- Docusaurus v3
- React
- Node.js 18+
- npm or yarn

## Setup

1. Install dependencies:
   ```bash
   npm install
   ```

2. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your backend API URL and other configuration
   ```

3. Run the development server:
   ```bash
   npm start
   ```

## Building

To build the website for production:

```bash
npm run build
```

The built website will be in the `build` directory and can be served using any static hosting service.

## Environment Variables

- `REACT_APP_BACKEND_URL` - URL of the backend API server
- `REACT_APP_RAG_ENABLED` - Whether to enable RAG features (true/false)

## Project Structure

- `src/` - Source files for custom components
- `docs/` - Documentation content
- `static/` - Static assets
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration