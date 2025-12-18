# Data Model: Book + RAG Bot + Multi-Agent System

## Entities

### AgentRequest
Represents a user's request to execute a specific agent with associated parameters and context.

**Fields:**
- id: string (UUID)
- agent_type: enum (RESEARCH_AGENT, WRITER_AGENT, EDITOR_AGENT, RAG_ENGINEER_AGENT, DEVELOPER_AGENT, DOCUMENTATION_AGENT, PROJECT_PLANNER_AGENT, GLOSSARY_MAKER, CHAPTER_GENERATOR, CODE_EXPLAINER, QUIZ_CREATOR)
- parameters: object (agent-specific parameters)
- context: object (chapter content, user profile, book outline, etc.)
- created_at: timestamp
- status: enum (PENDING, PROCESSING, COMPLETED, FAILED)
- user_id: string (optional, for personalization)

**Relationships:**
- One-to-many with GeneratedContent (one request can generate multiple content pieces)

**Validation rules:**
- agent_type must be one of the defined enum values
- parameters must match the schema for the specific agent type
- context must contain required fields for the agent

### GeneratedContent
The output produced by agents, including chapters, glossaries, code explanations, quizzes, documentation, and other educational materials.

**Fields:**
- id: string (UUID)
- request_id: string (foreign key to AgentRequest)
- content_type: enum (CHAPTER, GLOSSARY, CODE_EXPLANATION, QUIZ, DOCUMENTATION, RESEARCH_SUMMARY, EDITED_CONTENT, RAG_COMPONENT, DEPLOYMENT_SCRIPT)
- content: string (the actual generated content in appropriate format)
- metadata: object (generation details, sources, quality metrics, etc.)
- created_at: timestamp
- updated_at: timestamp
- quality_score: number (0-100, quality assessment)

**Relationships:**
- Many-to-one with AgentRequest
- One-to-many with ContentLink (links from generated content to other content)

**Validation rules:**
- content_type must match the agent_type from the request
- content must be valid and appropriate for the content_type
- metadata must contain required generation information
- quality_score must be between 0 and 100

### UserProfile
Information from user profiles that influences the behavior of agents, particularly for personalization.

**Fields:**
- user_id: string (UUID)
- experience_level: enum (BEGINNER, INTERMEDIATE, EXPERT)
- technical_background: string (programming, robotics, etc.)
- preferred_difficulty: enum (EASY, MEDIUM, HARD)
- learning_goals: array of strings
- hardware_access: array of strings (e.g., "Jetson", "GPU", "Cloud")
- language_preference: string
- created_at: timestamp
- updated_at: timestamp

**Relationships:**
- One-to-many with AgentRequest (optional, for personalized requests)
- One-to-many with RAGSession (for personalized interactions)

**Validation rules:**
- experience_level must be one of the defined enum values
- preferred_difficulty must be one of the defined enum values
- technical_background should be non-empty for personalized results

### ContentLink
Represents links between generated content and other content in the book, particularly for glossary terms linking to their occurrences.

**Fields:**
- id: string (UUID)
- source_content_id: string (ID of the content containing the link)
- target_content_id: string (ID of the content being linked to)
- link_type: enum (GLOSSARY_TERM, CROSS_REFERENCE, RELATED_CONTENT, SEE_ALSO)
- created_at: timestamp

**Relationships:**
- Many-to-one with GeneratedContent (source)
- Many-to-one with GeneratedContent (target)

**Validation rules:**
- Both source and target content IDs must exist
- link_type must be one of the defined enum values

### RAGSession
Represents a user's interaction session with the RAG chatbot, including query history and context.

**Fields:**
- id: string (UUID)
- user_id: string (optional, for logged-in users)
- session_token: string (for anonymous users)
- created_at: timestamp
- updated_at: timestamp
- query_history: array of objects (containing query, response, timestamp)
- active_context: string (current context for conversation)

**Relationships:**
- One-to-many with RAGQuery (queries within this session)

**Validation rules:**
- Either user_id or session_token must be provided
- query_history items must contain required fields

### RAGQuery
Represents a single query to the RAG system and its response.

**Fields:**
- id: string (UUID)
- session_id: string (foreign key to RAGSession)
- query: string (the user's question)
- response: string (the RAG system's answer)
- sources: array of strings (document IDs used to generate response)
- confidence_score: number (0-100, confidence in response accuracy)
- created_at: timestamp

**Relationships:**
- Many-to-one with RAGSession
- Many-to-many with GeneratedContent (sources used)

**Validation rules:**
- confidence_score must be between 0 and 100
- sources must reference existing content IDs
- query and response must not be empty

### BookChapter
Represents a chapter in the book, which may be generated or edited by agents.

**Fields:**
- id: string (UUID)
- title: string
- content: string (Docusaurus-compatible Markdown)
- module_focus: string (the module or topic the chapter covers)
- word_count: number
- difficulty_level: enum (BEGINNER, INTERMEDIATE, EXPERT)
- created_at: timestamp
- updated_at: timestamp
- generated_by: string (ID of the agent request that created this chapter)

**Relationships:**
- One-to-many with ContentLink (as target content)
- Many-to-one with AgentRequest (as generated content)

**Validation rules:**
- difficulty_level must be one of the defined enum values
- content must be valid Docusaurus-compatible Markdown
- word_count must be non-negative

## State Transitions

### AgentRequest Status Transitions
- PENDING → PROCESSING (when agent starts processing)
- PROCESSING → COMPLETED (when agent finishes successfully)
- PROCESSING → FAILED (when agent encounters an error)
- PENDING → FAILED (when request validation fails)

### RAGQuery Confidence Levels
- HIGH (80-100): Response is highly confident and should be provided
- MEDIUM (50-79): Response has moderate confidence, may need verification
- LOW (0-49): Response has low confidence, should indicate uncertainty

## Constraints

1. All generated content must comply with the book's constitution requirements (technical accuracy, safety, formatting standards)
2. Agent requests must not exceed rate limits to prevent system overload
3. Generated content must be linked back to source material for verification
4. User profile data must be used appropriately for personalization without compromising privacy
5. RAG responses must always cite sources and never hallucinate information
6. Content quality must meet minimum quality_score thresholds before being accepted