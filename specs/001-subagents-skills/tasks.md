---
description: "Task list for Subagents & Reusable Intelligence feature implementation"
---

# Tasks: Subagents & Reusable Intelligence

**Input**: Design documents from `/specs/[001-subagents-skills]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend services**: `backend/src/`
- **Frontend/Docusaurus**: `frontend/`
- **Shared utilities**: `shared/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in backend/, frontend/, shared/
- [x] T002 Initialize Python project with dependencies in backend/requirements.txt
- [x] T003 [P] Configure linting and formatting tools for Python and JavaScript
- [x] T004 Initialize Docusaurus project in frontend/ with proper configuration
- [x] T005 [P] Set up shared prompts and utilities in shared/ directory

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Setup database schema and migrations framework in backend/src/database/
- [x] T007 [P] Implement user profile model and service in backend/src/models/user_profile.py and backend/src/services/user_service.py
- [x] T008 [P] Create agent request model in backend/src/models/agent_request.py
- [x] T009 Create generated content model in backend/src/models/generated_content.py
- [x] T010 Create content link model in backend/src/models/content_link.py
- [x] T011 Create RAG session models in backend/src/models/rag_session.py and backend/src/models/rag_query.py
- [x] T012 Create book chapter model in backend/src/models/book_chapter.py
- [x] T013 Setup API routing and middleware structure in backend/src/api/
- [x] T014 Configure error handling and logging infrastructure in backend/src/utils/
- [x] T015 Setup environment configuration management in backend/src/config/

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Glossary Maker (Priority: P1) 🎯 MVP

**Goal**: Provide essential learning support by automatically generating glossaries from chapter content with links to their occurrences

**Independent Test**: Can be fully tested by running the Glossary Maker on a chapter and verifying that relevant terms are identified, defined, and linked back to their occurrences in the text.

### Implementation for User Story 1

- [x] T016 [P] [US1] Create Glossary Maker subagent system prompt in shared/prompts/glossary_maker_system_prompt.txt
- [x] T017 [P] [US1] Implement Glossary Maker core logic in backend/src/agents/glossary_maker/
- [x] T018 [US1] Create glossary generation service in backend/src/services/glossary_service.py
- [x] T019 [US1] Implement term extraction functionality in backend/src/services/glossary_service.py
- [x] T020 [US1] Implement term definition generation in backend/src/services/glossary_service.py
- [x] T021 [US1] Create content linking functionality for glossary terms in backend/src/services/content_link_service.py
- [x] T022 [US1] Add Glossary Maker endpoint in backend/src/api/agent_routes.py
- [x] T023 [US1] Create frontend component for glossary display in frontend/src/components/agent-widgets/glossary-widget.js
- [x] T024 [US1] Integrate glossary linking with book content in frontend/src/components/book-content/
- [x] T025 [US1] Add validation and error handling for glossary generation
- [x] T026 [US1] Add logging for glossary generation operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Code Explainer (Priority: P1)

**Goal**: Enable students to understand complex code examples by automatically generating detailed explanations with specific highlighting of ROS 2 and Isaac Sim commands

**Independent Test**: Can be fully tested by running the Code Explainer on various code snippets and verifying that explanations are accurate and ROS 2/Isaac Sim specific commands are properly highlighted.

### Implementation for User Story 2

- [x] T027 [P] [US2] Create Code Explainer subagent system prompt in shared/prompts/code_explainer_system_prompt.txt
- [x] T028 [P] [US2] Implement Code Explainer core logic in backend/src/agents/code_explainer/
- [x] T029 [US2] Create code parsing service in backend/src/services/code_explainer_service.py
- [x] T030 [US2] Implement ROS 2 command detection and highlighting in backend/src/services/code_explainer_service.py
- [x] T031 [US2] Implement Isaac Sim command detection and highlighting in backend/src/services/code_explainer_service.py
- [x] T032 [US2] Add Code Explainer endpoint in backend/src/api/agent_routes.py
- [x] T033 [US2] Create frontend component for code explanation display in frontend/src/components/agent-widgets/code-explainer-widget.js
- [x] T034 [US2] Integrate code explanation with book content in frontend/src/components/book-content/
- [x] T035 [US2] Add validation and error handling for code explanation
- [x] T036 [US2] Add logging for code explanation operations

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Quiz Creator (Priority: P2)

**Goal**: Enable students to test their understanding of chapter content by generating MCQs, short answer questions, and coding exercises with configurable difficulty based on user profile

**Independent Test**: Can be fully tested by generating quizzes for different chapters and verifying that question types and difficulty levels match the student's profile and chapter content.

### Implementation for User Story 3

- [x] T037 [P] [US3] Create Quiz Creator subagent system prompt in shared/prompts/quiz_creator_system_prompt.txt
- [x] T038 [P] [US3] Implement Quiz Creator core logic in backend/src/agents/quiz_creator/
- [x] T039 [US3] Create quiz generation service in backend/src/services/quiz_service.py
- [x] T040 [US3] Implement MCQ generation functionality in backend/src/services/quiz_service.py
- [x] T041 [US3] Implement short answer question generation in backend/src/services/quiz_service.py
- [x] T042 [US3] Implement coding exercise generation in backend/src/services/quiz_service.py
- [x] T043 [US3] Add user profile integration for difficulty configuration in backend/src/services/quiz_service.py
- [x] T044 [US3] Add Quiz Creator endpoint in backend/src/api/agent_routes.py
- [x] T045 [US3] Create frontend component for quiz generation and display in frontend/src/components/agent-widgets/quiz-widget.js
- [x] T046 [US3] Integrate quiz generation with book content in frontend/src/components/book-content/
- [x] T047 [US3] Add validation and error handling for quiz generation
- [x] T048 [US3] Add logging for quiz generation operations

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Chapter Generator (Priority: P2)

**Goal**: Enhance content creation efficiency by automatically generating well-structured educational content that follows pedagogical best practices based on module focus

**Independent Test**: Can be fully tested by providing a module focus to the Chapter Generator and verifying that the resulting content is educationally valuable, technically accurate, and properly structured.

### Implementation for User Story 4

- [x] T049 [P] [US4] Create Chapter Generator subagent system prompt in shared/prompts/chapter_generator_system_prompt.txt
- [x] T050 [P] [US4] Implement Chapter Generator core logic in backend/src/agents/chapter_generator/
- [x] T051 [US4] Create chapter generation service in backend/src/services/chapter_service.py
- [x] T052 [US4] Implement chapter structure generation with headings in backend/src/services/chapter_service.py
- [x] T053 [US4] Implement code block generation functionality in backend/src/services/chapter_service.py
- [x] T054 [US4] Implement diagram generation functionality in backend/src/services/chapter_service.py
- [x] T055 [US4] Implement exercise generation functionality in backend/src/services/chapter_service.py
- [x] T056 [US4] Add Chapter Generator endpoint in backend/src/api/agent_routes.py
- [x] T057 [US4] Create frontend component for chapter generation in frontend/src/components/agent-widgets/chapter-generator-widget.js
- [x] T058 [US4] Add validation and error handling for chapter generation
- [x] T059 [US4] Add logging for chapter generation operations

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T060 [P] Add comprehensive documentation for all subagents in frontend/docs/agent-guides/
- [x] T061 Code cleanup and refactoring across all agent implementations
- [x] T062 Performance optimization for subagent response times
- [x] T063 [P] Add unit tests for all subagent services in backend/tests/
- [x] T064 Security hardening for agent request validation
- [x] T065 Run quickstart.md validation to ensure all subagents work as expected
- [x] T066 Integrate all subagents with the Docusaurus-based book website
- [x] T067 Add quality scoring and validation for all generated content
- [x] T068 Implement content caching for improved performance
- [x] T069 Add analytics and monitoring for subagent usage

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core agent logic before services
- Services before endpoints
- Core implementation before frontend integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Stories 1-2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Glossary Maker)
4. Complete Phase 4: User Story 2 (Code Explainer)
5. **STOP and VALIDATE**: Test User Stories 1-2 independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Glossary Maker)
   - Developer B: User Story 2 (Code Explainer)
   - Developer C: User Story 3 (Quiz Creator)
   - Developer D: User Story 4 (Chapter Generator)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence