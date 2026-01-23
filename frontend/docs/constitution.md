---
sidebar_position: 2
title: "Project Constitution"
---

# Physical AI Book Constitution

## Core Principles

### Global Content Standards
All content must be technically accurate, grounded in official documentation, reproducible by students, and safe for minors. The tone must be clear, structured, friendly, and pedagogical. Every chapter must contain Overview Summary, Learning Objectives, Detailed Theory, Practical Implementation Steps, Python & ROS 2 code examples, Diagrams/Flowcharts, Hands-on Exercises, and Assessment Questions. Code samples must follow ROS 2 Rolling/Humble conventions, Python PEP8 style, Docusaurus Markdown syntax, and platform-agnostic instructions (Linux, Jetson, Cloud). No chapter may contradict another; all updates must maintain global consistency.

### Formatting Standards
All agents MUST follow Docusaurus-compatible Markdown: # for main title, ## for subheadings, fenced code blocks for Python, XML, URDF, Bash, YAML, tables for comparisons, admonitions for important notes, and internal anchors for RAG linking. Example code block rule: import rclpy from rclpy.node import Node.

### AI Agent Behavior Rules
All Claude agents and subagents must follow: strict compliance with this constitution, consistent writing style across all chapters, and reusable intelligence using Subagents, Agent Skills, and Predefined toolchains. Agents must automatically generate a glossary per chapter, generate quizzes and flashcards, generate diagrams using Mermaid, and validate all code before output. No hallucination is allowed. If unsure, the agent must say: "Source required — ambiguity detected."

### RAG Chatbot Requirements
The integrated RAG chatbot must support: Full-book retrieval, Section-level retrieval, Paragraph-level retrieval, and User-selected text Q&A (highlight → ask → answer). Answering Rules: Always cite source references, Never hallucinate, Prioritize retrieved context, Decline to answer questions outside book scope, Provide step-by-step robotics explanations when relevant. Technical Requirements: Backend: FastAPI, Vector DB: Qdrant Cloud, SQL DB: Neon Postgres, Embeddings: OpenAI text-embedding-3-large, Frontend: Book-embedded widget.

### Personalization Engine Rules
When user logs in via better-auth: Collect user's background including Software experience, Hardware experience, Language preference, Robotics familiarity, and Access to Jetson / GPU / Cloud. Personalization behavior: Adapt complexity (Beginner / Intermediate / Expert), Suggest personalized exercises, Tailor examples based on user's hardware, Simplify or expand sections as required, Never omit core technical content. All personalized content must be generated on-demand.

### Safety Rules
Because the users may include minors: No dangerous robotics instructions, No actionable harmful procedures, No unsafe electrical/mechanical instructions, No bypassing of motor safety or torque limits, No violent simulation content. All chapters must emphasize safe practice in robotics labs.

## Deployment Rules
All generated content must be: deterministic (same input → same output), reproducible, version-controlled in GitHub, compatible with GitHub Pages or Vercel, structured automatically using Spec-Kit Plus pipelines. CI/CD should: validate Markdown, check broken links, lint code, ensure spec conformance, rebuild vector database for RAG. Subagents MUST support: Reusable Intelligence, Content Builder Agent, Diagram Generator Agent, ROS 2 Code Writer Agent, Glossary & Quiz Generator Agent, Personalization Agent, Urdu Translation Agent. Advanced Features: Dynamic chapter extension, Auto-metadata creation, Auto-linked RAG anchors, Student progress tracking.

## Final Product Requirements
The final submission must include: Public GitHub repository, Live Docusaurus website on GitHub Pages or Vercel, Embedded RAG chatbot, Full book content auto-generated through constitution + agent.

## Governance
This constitution governs the creation, maintenance, and evolution of the Physical AI & Humanoid Robotics book. All Claude Code agents, subagents, and skills MUST follow these rules when generating content. The book will comprehensively cover: Physical AI & Embodied Intelligence, ROS 2 Essentials & Robotics Middleware, Gazebo Physics Simulation, Unity Visualization & Digital Twin, NVIDIA Isaac Sim, Isaac ROS, and RL pipelines, Vision-Language-Action (VLA) systems, Conversational Robotics with GPT-based agents, Capstone: Autonomous Humanoid Robot (Simulated + Edge Deployment). The core goal is to create a world-class, industry-standard reference for robotics + AI.

**Version**: 2.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10