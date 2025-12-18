<!--
Sync Impact Report:
Version Change: N/A → 1.0.0
Reason: Initial constitution ratification for AI-Driven Book with Embedded RAG Chatbot project
Modified Principles: N/A (initial creation)
Added Sections:
  - Core Principles (5 principles)
  - Technical Stack & Constraints
  - Development Workflow
  - Governance
Removed Sections: N/A
Templates Requiring Updates:
  ✅ spec-template.md - Reviewed (already aligned with spec-driven principles)
  ✅ plan-template.md - Reviewed (already includes constitution check)
  ✅ tasks-template.md - Reviewed (already aligned with testable tasks principle)
Follow-up TODOs: None
-->

# AI-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven, AI-First Development

All development starts with clear specifications authored through Claude Code using Spec-Kit Plus workflows. Features must be specified (`/sp.specify`), planned (`/sp.plan`), and broken into tasks (`/sp.tasks`) before implementation. No feature work begins without a completed spec that defines user stories, requirements, and success criteria. AI tools (Claude Code) are first-class development partners, not optional enhancements.

**Rationale**: Prevents scope creep, ensures alignment with user intent, and maintains auditability through documented decision trails. Spec-first approach guarantees testable requirements and measurable outcomes.

### II. Accuracy and Content-Grounded Responses

The RAG chatbot MUST respond only with information retrieved from the book content or user-selected text. No hallucinations, external knowledge, or speculative answers are permitted. All responses must cite the source chapter/section. When information is not found in the retrieved context, the system must explicitly state "This information is not covered in the retrieved content" rather than fabricating an answer.

**Rationale**: Technical books require factual accuracy. Users depend on the chatbot to provide authoritative answers from the book itself. Content grounding prevents misinformation and maintains trust.

### III. Clear, Developer-Focused Writing

All book content must be written with clarity, practical examples, and runnable code. Target audience is developers learning AI, RAG systems, and deployment workflows. Each chapter includes:
- Conceptual explanations in plain language
- Concrete code examples with line-by-line comments
- Step-by-step tutorials that readers can follow
- Real-world use cases and troubleshooting sections

Avoid academic jargon without definitions. Every concept must be explained before being used.

**Rationale**: Learning materials succeed when readers can immediately apply what they read. Developer-focused writing with runnable examples accelerates comprehension and reduces frustration.

### IV. Reproducible and Production-Ready Setup

All code, configurations, and deployment instructions must work exactly as documented. Readers following the book must be able to:
- Clone repositories and run setup scripts without errors
- Deploy the book to GitHub Pages using provided workflows
- Stand up the RAG backend (FastAPI + Neon Postgres + Qdrant) with documented steps
- Configure environment variables via `.env` files (never hardcoded)
- Validate their setup with health-check endpoints and test scripts

Every command, API key setup, and deployment step must be reproducible.

**Rationale**: Broken examples destroy reader trust. Production-ready setups ensure learners build real, deployable systems rather than toy prototypes.

### V. Free-Tier and Open-Source Compliant

All technologies and services used in the book and chatbot must have free tiers or be fully open-source:
- **Docusaurus**: Free and open-source static site generator
- **GitHub Pages**: Free hosting for public repositories
- **Neon Serverless Postgres**: Free tier with 0.5 GB storage
- **Qdrant Cloud**: Free tier with 1 GB vectors
- **OpenAI Agents/ChatKit SDKs**: Open-source SDKs (API usage is reader's responsibility)
- **FastAPI**: Free and open-source Python web framework

No paid services, licensed software, or premium features may be required to complete the book's examples.

**Rationale**: Accessibility. Learners worldwide must be able to follow along without financial barriers. Free-tier compliance ensures the widest possible audience.

## Technical Stack & Constraints

### Book Infrastructure

**Static Site Generator**: Docusaurus (React-based, open-source)
**Deployment**: GitHub Pages via GitHub Actions
**Version Control**: Git + GitHub
**Content Format**: Markdown with MDX for interactive components
**Hosting Requirements**: Public repository for free GitHub Pages hosting

### RAG Chatbot Stack

**Backend Framework**: FastAPI (Python 3.10+)
**Vector Database**: Qdrant Cloud (Free Tier: 1 GB vectors, REST API)
**Relational Database**: Neon Serverless Postgres (Free Tier: 0.5 GB storage)
**AI SDKs**: OpenAI Agents / ChatKit SDKs (open-source)
**Embeddings**: OpenAI text-embedding-ada-002 or open-source alternatives (instructor-xl, BGE)
**Secrets Management**: `.env` files with `python-dotenv`, never committed to Git

### Constraints

- **No Paid Services**: All infrastructure must run on free tiers
- **No Hardcoded Secrets**: API keys, database URLs, and credentials in `.env` only
- **Modular Architecture**: Frontend, backend, and database must be independently deployable and testable
- **API-First Design**: Backend exposes REST APIs; frontend consumes them
- **Vector Search Requirements**: Qdrant must support semantic search with metadata filtering (chapter, section)
- **Database Requirements**: Neon Postgres stores chat history, user sessions, and metadata

## Development Workflow

### Feature Development Lifecycle

1. **Specify** (`/sp.specify`): Write feature spec with user stories, requirements, and success criteria
2. **Plan** (`/sp.plan`): Architect the technical solution, document decisions, check constitution compliance
3. **Tasks** (`/sp.tasks`): Break plan into testable, executable tasks
4. **Implement** (`/sp.implement`): Execute tasks with red-green-refactor TDD cycles (if tests required)
5. **Commit & PR** (`/sp.git.commit_pr`): Commit work, create pull request with AI-generated summary

### Prompt History Records (PHRs)

Every user interaction that results in code, specs, plans, or decisions MUST generate a PHR in `history/prompts/`:
- **Constitution changes** → `history/prompts/constitution/`
- **Feature-specific work** → `history/prompts/<feature-name>/`
- **General tasks** → `history/prompts/general/`

PHRs preserve full user input (verbatim, not truncated) and assistant output for auditability.

### Architecture Decision Records (ADRs)

When architecturally significant decisions are made (framework choice, database schema, API design), the system suggests:

> 📋 Architectural decision detected: [brief description]
> Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`

Wait for user consent; never auto-create ADRs. ADRs are stored in `history/adr/` and linked from relevant specs/plans.

### Testing & Validation

- **TDD Required**: When tests are explicitly requested in feature specs, write tests first (red), then implement (green), then refactor
- **Contract Tests**: Validate API endpoints match OpenAPI specs
- **Integration Tests**: Test end-to-end user journeys (user asks question → RAG retrieves context → LLM generates answer)
- **Manual Validation**: Deployment instructions tested on clean environments

### Git & Version Control

- **Feature Branches**: `###-feature-name` format (e.g., `001-rag-chatbot-backend`)
- **Commit Messages**: Clear, concise, with AI co-authorship attribution
- **Pull Requests**: Auto-generated summaries from Git agent, linked to specs/tasks
- **No Force Pushes**: To `main`/`master` unless explicitly approved

## Governance

### Constitution Authority

This constitution supersedes all other practices, guidelines, or conventions. All feature specs, plans, and code reviews MUST verify compliance with these principles.

### Amendment Process

Constitution amendments require:
1. Documented rationale for the change
2. Impact analysis on existing features and templates
3. User approval before changes are committed
4. Version bump (MAJOR for breaking changes, MINOR for additions, PATCH for clarifications)
5. Migration plan for dependent artifacts (templates, specs, plans)

### Compliance Reviews

Every `/sp.plan` execution includes a Constitution Check gate that validates:
- Technical stack aligns with approved free-tier services
- No hardcoded secrets or paid dependencies introduced
- Development workflow follows spec-driven process
- Tests and validation match feature requirements

Violations must be justified in the plan's "Complexity Tracking" section or the plan is rejected.

### Runtime Guidance

This constitution defines **non-negotiable rules**. For detailed execution guidance (how to write specs, structure plans, generate tasks), refer to:
- `.specify/templates/commands/sp.specify.md`
- `.specify/templates/commands/sp.plan.md`
- `.specify/templates/commands/sp.tasks.md`
- `CLAUDE.md` (agent-specific operational instructions)

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
