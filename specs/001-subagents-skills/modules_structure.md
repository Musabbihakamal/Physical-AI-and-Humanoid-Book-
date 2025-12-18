# Modules Structure: Subagents & Reusable Intelligence

## Overview
This document outlines the modular structure of the Subagents & Reusable Intelligence system, organized by functional modules and implementation phases.

## Module 1: Glossary Maker (Priority P1 - Week 1-2)
**Purpose**: Auto-generate glossary from chapter content and link terms to their occurrences

### Components:
- **Backend**:
  - `backend/src/agents/glossary_maker/` - Core agent logic
  - `backend/src/services/glossary_service.py` - Term extraction and definition generation
  - `backend/src/services/content_link_service.py` - Linking functionality
  - `backend/src/api/agent_routes.py` - API endpoints
- **Frontend**:
  - `frontend/src/components/agent-widgets/glossary-widget.js` - Widget component
  - `frontend/src/components/book-content/` - Integration with book content

### Features:
- Term identification from chapter content
- Definition generation for technical terms
- Linking terms to their occurrences in text
- Docusaurus-compatible output format

## Module 2: Code Explainer (Priority P1 - Week 2-3)
**Purpose**: Parse code snippets and generate explanations with ROS 2 and Isaac Sim command highlighting

### Components:
- **Backend**:
  - `backend/src/agents/code_explainer/` - Core agent logic
  - `backend/src/services/code_explainer_service.py` - Code parsing and highlighting
  - `backend/src/api/agent_routes.py` - API endpoints
- **Frontend**:
  - `frontend/src/components/agent-widgets/code-explainer-widget.js` - Widget component
  - `frontend/src/components/book-content/` - Integration with book content

### Features:
- Code snippet parsing and analysis
- Detailed explanation generation
- ROS 2 and Isaac Sim command detection and highlighting
- Multi-language code support

## Module 3: Quiz Creator (Priority P2 - Week 3-4)
**Purpose**: Generate MCQs, short answer, and coding exercises with configurable difficulty

### Components:
- **Backend**:
  - `backend/src/agents/quiz_creator/` - Core agent logic
  - `backend/src/services/quiz_service.py` - Question generation logic
  - `backend/src/models/user_profile.py` - User profile integration
  - `backend/src/api/agent_routes.py` - API endpoints
- **Frontend**:
  - `frontend/src/components/agent-widgets/quiz-widget.js` - Widget component
  - `frontend/src/components/book-content/` - Integration with book content

### Features:
- Multiple question type generation (MCQ, short answer, coding exercises)
- Difficulty configuration based on user profile
- Adaptive question complexity
- Interactive quiz interface

## Module 4: Chapter Generator (Priority P2 - Week 4-5)
**Purpose**: Auto-generate chapter sections based on module focus with proper structure

### Components:
- **Backend**:
  - `backend/src/agents/chapter_generator/` - Core agent logic
  - `backend/src/services/chapter_service.py` - Chapter generation logic
  - `backend/src/models/book_chapter.py` - Chapter data model
  - `backend/src/api/agent_routes.py` - API endpoints
- **Frontend**:
  - `frontend/src/components/agent-widgets/chapter-generator-widget.js` - Widget component

### Features:
- Chapter structure generation with headings
- Code block and diagram inclusion
- Exercise generation
- Docusaurus-compatible Markdown output

## Module 5: Core Infrastructure (Week 1)
**Purpose**: Shared foundation for all subagents

### Components:
- **Backend**:
  - `backend/src/models/` - Data models (AgentRequest, GeneratedContent, etc.)
  - `backend/src/database/` - Database setup
  - `backend/src/utils/` - Security, validation, caching
  - `backend/src/api/` - API routing and middleware
- **Shared**:
  - `shared/prompts/` - System prompts
  - `shared/utils/` - Content validation utilities

### Features:
- Database schema and models
- Security and validation
- Content caching
- Error handling and logging

## Module 6: Frontend Integration (Week 3-4)
**Purpose**: Docusaurus-based interface for all subagents

### Components:
- **Frontend**:
  - `frontend/src/components/agent-widgets/` - All agent widgets
  - `frontend/src/components/book-content/` - Book content integration
  - `frontend/src/services/api.js` - API service
  - `frontend/src/utils/cache.js` - Frontend caching
  - `frontend/src/services/analytics.js` - Usage tracking

### Features:
- Interactive agent widgets
- Book content integration
- Real-time API communication
- Usage analytics and monitoring

## Dependencies Between Modules:
- Core Infrastructure (Module 5) is required by all other modules
- User Profile functionality (Module 3) supports difficulty configuration in other modules
- Content Linking (Module 1) supports linking in other modules
- Frontend Integration (Module 6) integrates all agent modules