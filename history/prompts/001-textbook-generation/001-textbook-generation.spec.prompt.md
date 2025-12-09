---
id: 001
title: Textbook Generation Spec
stage: spec
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-generation
branch: 001-textbook-generation
user: Claude Code
command: /sp.specify
labels: ["specification", "textbook", "docusaurus", "education", "ai"]
links:
  spec: ./../../specs/001-textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-textbook-generation/spec.md
 - specs/001-textbook-generation/checklists/requirements.md
tests:
 - null
---

## Prompt

```text
actions:
 - id: generate_book_structure
 description: >
 Create full folder and file structure for a Docusaurus-based textbook
 using the syllabus of the "Physical AI & Humanoid Robotics" course.
 output:
 path: docs/
 format: mdx
 multiple: true
Copy code
09/12/2025, 00:11 ChatGPT
https://chatgpt.com 24/70
 requirements: 
 - Create a new folder for each module and chapter.
 - Include frontmatter in every file.
 - Write placeholder intro text if full content is not generated yet.
 - Must follow Docusaurus + Spec-Kit Plus conventions.
 - id: generate_sidebar
 description: Generate sidebar.json navigation tree for the entire textbook.
 output:
 path: sidebars.js
 format: javascript
 requirements:
 - Sidebar categories must match modules and chapters.
 - All IDs must match MDX filenames.
 - Everything must be Docusaurus-compliant.
 - id: generate_full_chapter
 description: >
 Expand a chosen chapter into full MDX content with headings, diagrams,
 code examples (Python, ROS 2, rclpy), Gazebo/Isaac Sim references, exercise
 and real-world scenarios.
 input:
 chapter_name: string
 output:
 path: docs/[chapter_name].mdx
 format: mdx
 requirements:
 - Include H1 title equal to the chapter name.
 - Add Mermaid diagrams or ASCII diagrams where relevant.
 - Use short paragraphs for RAG compatibility.
 - Include a summary, key terms, exercises.
 - id: generate_rag_architecture
 description: >
 Generate backend architecture for the book's RAG chatbot using
 FastAPI, Qdrant Cloud, Neon Postgres, and OpenAI Agents.
 output:
 path: backend/
 format: python
 multiple: true
09/12/2025, 00:11 ChatGPT
https://chatgpt.com 25/70
 requirements: 
 - Include embed pipeline.
 - Include search API.
 - Include chat endpoint.
 - Produce example requests/responses.
 - Use clean folder structure (api/, db/, utils/).
 - id: generate_mcp_schema
 description: >
 Create MCP-compatible schemas and example messages for Docorous frontend
 integration (personalization, Urdu translation, chat, chapters).
 output:
 path: mcp/
 format: json
 multiple: true
 requirements:
 - Must include user profile schema.
 - Must include chapter retrieval schema.
 - Must include personalization events.
 - Must include translation events.
 - id: generate_readme
 description: >
 Generate README.md for the GitHub repository describing project purpose,
 setup instructions, Spec-Kit Plus usage, deployment steps, and hackathon not
 output:
 path: README.md
 format: markdown
constraints:
 - The content must strictly follow the official syllabus details I provided ear
 - No hallucination about hardware/software not listed in syllabus.
 - Style must be clean, technical, beginner-friendly.
 - MDX must be valid for Docusaurus.
 - All generated files must be deterministic and reproducible.
style:
 tone: "University-level, clear, friendly, technical"
 formatting:
 code_blocks: true
09/12/2025, 00:11 ChatGPT
https://chatgpt.com 26/70

🎯 Bro-Style Summary
/sp.specify = Claude Code ko EXACT instructions deta hai:
✔ kya generate karna hai
✔ kis folder me
✔ kya format me
✔ kya rules follow karne hain
Ye tumhare poore hackathon book project ko automate, structure, aur standardize kar
deta hai.
Agar chaho to mai tumhare liye:
🔥 /sp.tasks
🔥 /sp.plan
🔥 Complete book outline
🔥 All chapter folder structures
bhi generate kar dunga.
Just say: "bro generate /sp.plan" or "bro give me all chapters".
 headers: true
 mermaid_diagrams: true
 tables: true
 exercises: true
```

## Response snapshot

Created comprehensive feature specification for Physical AI & Humanoid Robotics textbook generation. The specification includes:

- 6 user stories prioritized by importance (P1-P6)
- 14 functional requirements covering textbook structure, navigation, RAG architecture, content generation, and documentation
- 7 measurable success criteria focused on user experience and system performance
- 5 key entities for data modeling
- Quality checklist confirming specification completeness

User stories cover: textbook structure generation, navigation/sidebar, RAG chatbot, full chapter content, MCP schemas, and repository documentation.

## Outcome

- ✅ Impact: Complete feature specification ready for planning phase
- 🧪 Tests: N/A for specification creation
- 📁 Files: Created spec.md and requirements.md in specs/001-textbook-generation/
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: Specification addresses all aspects of textbook generation with clear user value

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation planning with /sp.plan