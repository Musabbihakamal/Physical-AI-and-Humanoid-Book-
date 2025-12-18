---
id: 001
title: Book-RAG-Multi-Agent-Plan
stage: plan
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: 001-subagents-skills
branch: 001-subagents-skills
user: Claude Code User
command: /sp.plan
labels: [multi-agent,rag,book-generation,planning]
links:
  spec: ../specs/001-subagents-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - ../specs/001-subagents-skills/plan.md
  - ../specs/001-subagents-skills/research.md
  - ../specs/001-subagents-skills/data-model.md
  - ../specs/001-subagents-skills/quickstart.md
  - ../specs/001-subagents-skills/contracts/agent-api.yaml
tests:
  - plan-validation
---

## Prompt

. Project Overview

This project aims to create a professional AI system that supports:

✔ Writing and managing a complete book (non-fiction or technical)
✔ A RAG-based chatbot trained on book content + external knowledge
✔ A multi-agent architecture with intelligent subagents
✔ A full documentation website built with Docusaurus
✔ A scalable framework to maintain and extend the project

The project uses subagents for research, writing, coding, editing, and retrieval tasks.
The final output includes:

A complete manuscript

A functioning RAG chatbot

A fully-structured multi-agent system

A documentation website for developers & end-users

2. Project Goals

Write and edit a complete book using AI subagents.

Build a high-quality RAG (Retrieval-Augmented Generation) pipeline.

Use a modular multi-agent system for different responsibilities.

Create tool-aware subagents (some with restricted tools).

Build a production-ready documentation site (Docusaurus).

Ensure that the system can scale to future volumes and future agents.

3. Project Deliverables
Book Writing Deliverables

Full book outline

All book chapters

Editor-reviewed versions

Style-consistent final manuscript

Export-ready formats (PDF, EPUB, Markdown)

RAG Bot Deliverables

Data ingestion pipeline

Embedding + vector storage

Query-handling pipeline

Context relevance scoring

Evaluation tests

Multi-Agent System Deliverables

System prompts for all subagents

Delegation rules

Tool access permissions

Inter-agent communication logic

Documentation Website Deliverables

Docusaurus installation

Technical docs

API reference

Tutorials for building RAG

Subagent documentation

Deployment instructions

4. Subagents Needed in This Project (With Roles)
A. Research Agent

Finds accurate information

Summarizes sources

Performs literature review

Provides citations

Supports book chapters

B. Writer Agent

Writes book content based on outlines

Expands bullet points into chapters

Maintains consistent tone/style

C. Editor Agent

Performs structural editing

Rewrites unclear sections

Removes redundancy

Ensures grammar + clarity

D. RAG Engineer Agent

Builds RAG pipeline

Handles embeddings & chunking strategy

Designs vector database schema

Writes retrieval logic

E. Developer Agent

Implements code needed for the project

Builds scripts, tools, utilities

Integrates RAG models

Maintains backend pipeline

F. Documentation Agent

Writes Docusaurus pages

Maintains folder structure

Creates tutorials, guides, diagrams

G. Project Planner Agent

Breaks tasks into steps

Generates timelines

Tracks progress

Determines next actions

5. Subagent System Prompts (Professional Level)

You will paste these individually when creating subagents.

🔹 Subagent Prompt: Research Agent

"You are the Research Agent. Your responsibility is to gather accurate, verifiable information.
You never fabricate data. You summarize, compare, analyze, and cite sources.
You avoid opinion unless asked, and never hallucinate.
Use tools responsibly for searching when needed."

🔹 Subagent Prompt: Writer Agent

"You are the Writer Agent. You expand outlines into full chapters with clarity and flow.
Maintain consistent voice and tone.
Do not invent facts — use provided research.
If research is missing, request it."

🔹 Subagent Prompt: Editor Agent

"You are the Editor Agent.
Improve structure, clarity, grammar, and readability.
Do NOT change meaning.
Focus on quality and coherence.
Highlight what was improved."

🔹 Subagent Prompt: RAG Engineer Agent

"You are the RAG Engineering Agent.
You build retrieval pipelines, embeddings, chunking, and vector database logic.
You design the most efficient and scalable RAG architecture.
You produce diagrams, flowcharts, and clearly explained code."

🔹 Subagent Prompt: Developer Agent

"You are the Developer Agent.
You write clean, production-ready code.
You maintain scripts, APIs, RAG utilities, and deployment logic.
You follow best practices and do not hallucinate library names."

🔹 Subagent Prompt: Documentation Agent

"You are the Documentation Agent.
You write polished, structured documentation for Docusaurus.
You create tutorials, examples, API references, and architecture guides.
Use clear formatting and consistent voice."

🔹 Subagent Prompt: Project Planner Agent

"You are the Project Planner Agent.
Break large tasks into steps.
Define milestones and dependencies.
Always produce actionable next steps."

6. Work Breakdown Structure (WBS)
Phase 1 — Book Planning

Generate full outline

Chapter-by-chapter research

Writer Agent produces drafts

Editor Agent reviews drafts

Phase 2 — RAG System Development

Gather book chapters into dataset

Chunking & embedding strategy

Vector DB setup

Retrieval & ranking logic

Evaluation

Phase 3 — Multi-Agent System Setup

Define subagent system prompts

Configure tool access

Main agent delegation logic

Testing cycles

Phase 4 — Documentation Website

Initialize Docusaurus

Add pages for:

Book workflow

RAG architecture

Agent roles

Code Tutorials

Final polish

Phase 5 — Final Assembly

Manuscript finalization

RAG bot deployment

Publish documentation

7. Expected Output From SP Engine

A full timeline

Detailed step-by-step tasks

Dependencies

Task priority

Module-based execution

Next action recommendations

8. Style & Quality Rules

Professional

Structured

Clear

No hallucinations

Maintain consistency

## Response snapshot

# Implementation Plan: Book + RAG Bot + Multi-Agent System

**Branch**: `001-subagents-skills` | **Date**: 2025-12-10 | **Spec**: [specs/001-subagents-skills/spec.md](../001-subagents-skills/spec.md)
**Input**: Feature specification from `/specs/[001-subagents-skills]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Development of a comprehensive AI system that includes: 1) A multi-agent architecture with specialized subagents (Research Agent, Writer Agent, Editor Agent, RAG Engineer Agent, Developer Agent, Documentation Agent, Project Planner Agent), 2) A RAG-based chatbot trained on book content and external knowledge, 3) A complete book generation system using AI subagents, and 4) A Docusaurus-based documentation website. The system will support writing, managing, and interacting with a complete technical book using AI capabilities.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend integration
**Primary Dependencies**: Claude Code SDK, Docusaurus, OpenAI API, FastAPI, Qdrant, Neon Postgres, Node.js
**Storage**: Vector DB (Qdrant Cloud), SQL DB (Neon Postgres), File storage for book content
**Testing**: pytest for backend logic, Jest for frontend components, integration tests for RAG pipeline
**Target Platform**: Linux server, cross-platform compatibility for documentation website
**Project Type**: Web application with backend services for RAG and multi-agent system
**Performance Goals**: Subagent responses within 10 seconds, 95% accuracy for code explanations, RAG responses within 2 seconds
**Constraints**: <200ms p95 for glossary generation, maintain technical accuracy >95%, integrate with Docusaurus Markdown, support 500 concurrent users
**Scale/Scope**: Support 500 concurrent subagent requests, handle entire book content for RAG, support multiple user profiles

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status: PASSED** - All constitution requirements have been addressed in the design

- Content Standards: All generated content will be accurate and grounded in official documentation through quality scoring and source verification
- Formatting: All agents will produce Docusaurus-compatible Markdown format as specified
- AI Behavior: Agents will follow strict compliance with constitution rules and avoid hallucination through verification mechanisms
- Safety: All content generation will include safety checks and validation for minor-appropriate content
- RAG Requirements: Chatbot will support proper retrieval levels (full-book, section, paragraph) with source citation requirements
- Deployment: All content will be deterministic, reproducible, and version-controlled through Git and structured data models

## Project Structure

### Documentation (this feature)

```text
specs/[001-subagents-skills]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend services
backend/
├── src/
│   ├── agents/
│   │   ├── research_agent/
│   │   ├── writer_agent/
│   │   ├── editor_agent/
│   │   ├── rag_engineer_agent/
│   │   ├── developer_agent/
│   │   ├── documentation_agent/
│   │   └── planner_agent/
│   ├── models/
│   │   ├── agent_request.py
│   │   ├── generated_content.py
│   │   └── user_profile.py
│   ├── services/
│   │   ├── agent_service.py
│   │   ├── rag_service.py
│   │   ├── content_service.py
│   │   └── user_service.py
│   ├── api/
│   │   ├── agent_routes.py
│   │   ├── rag_routes.py
│   │   └── content_routes.py
│   └── main.py
└── tests/

# Frontend/Docusaurus documentation
frontend/
├── src/
│   ├── components/
│   │   ├── agent-widgets/
│   │   ├── rag-chatbot/
│   │   └── book-content/
│   ├── pages/
│   └── services/
├── docs/
│   ├── book-content/
│   ├── agent-guides/
│   ├── rag-setup/
│   └── api-reference/
├── docusaurus.config.js
└── package.json

# Shared utilities
shared/
├── prompts/
├── utils/
└── types/
```

**Structure Decision**: Multi-service architecture with dedicated backend for agents and RAG services, and frontend Docusaurus site for documentation and book content with embedded chatbot functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for multi-agent book generation system with RAG capabilities
- 🧪 Tests: plan-validation
- 📁 Files: specs/001-subagents-skills/plan.md, specs/001-subagents-skills/research.md, specs/001-subagents-skills/data-model.md, specs/001-subagents-skills/quickstart.md, specs/001-subagents-skills/contracts/agent-api.yaml
- 🔁 Next prompts: /sp.tasks to generate actionable tasks
- 🧠 Reflection: The plan successfully integrates all required components including specialized agents, RAG system, and documentation website

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks from this plan using /sp.tasks