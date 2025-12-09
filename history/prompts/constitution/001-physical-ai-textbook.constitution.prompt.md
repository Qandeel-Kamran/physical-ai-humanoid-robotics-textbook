---
id: 001
title: Physical AI Textbook Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Code
command: /sp.constitution
labels: ["constitution", "textbook", "physical-ai", "humanoid-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - null
---

## Prompt

```text
title: Physical AI & Humanoid Robotics — AI-Native Textbook
version: 1.0.0
purpose:
 This Spec-Kit Plus project will generate a complete AI-native textbook for the
 "Physical AI & Humanoid Robotics" course using Claude Code, Gemini, and Docusau
 The book must be technically accurate, student-friendly, and aligned with the
 hackathon requirements.
goals:
 - Produce a full Docusaurus-based textbook using MDX.
 - Structure content exactly according to the course syllabus.
 - Include 20+ chapters covering:
 * Physical AI foundations
 * ROS 2
 * Gazebo
 * Unity
 * NVIDIA Isaac Sim
 * VSLAM
 * Navigation
 * VLA (Vision-Language-Action)
 * Capstone: Autonomous Humanoid Robot.
 - Generate modular chapters, clean frontmatter, and sidebar structure.
 - Prepare the book for GitHub Pages deployment.
deliverables:
 - Full folder structure for the textbook.
 - All chapters written in MDX with headings, diagrams, exercises, examples.
 - A complete sidebar.json navigation tree.
 - A clean README.md for project root.
 - Integration stubs for RAG chatbot, personalization, and Urdu translation.
constraints:
 - Content must directly follow the official course description.
 - Writing must be accessible to beginners but technically deep for developers.
 - All outputs must be compatible with Docusaurus + Spec-Kit Plus formatting.
 - Chapters must be deterministic, modular, and consistent in tone.
 - No hallucination: use only course details provided.
Copy code 09/12/2025, 00:11 ChatGPT
https://chatgpt.com 22/70
🎯 Simple Explanation (Bro-style)
/sp.constitution = Tumhare project ka brain.
Yahan tum likhti ho:
Book ka naam
Goals
Rules
Style
Structure
Kya generate karna hai
Kya allowed hai / kya nahi
Spec-Kit Plus iss constitution ko follow karke sari files bana deta hai.
style:
 - Clear, concise, university-level textbook writing.
 - Use diagrams, Python examples, ROS 2 code, simulation demos.
 - Use short sections to improve RAG retrieval.
ai_integration:
 - The generated book will be later enhanced with:
 * RAG chatbot using OpenAI Agents, FastAPI, Qdrant, Neon
 * Personalization system using better-auth
 * Content translation to Urdu
 - Keep chapters structured so they are easy to convert into embeddings.
frontends:
 - Final book will deploy on GitHub Pages.
 - A Docorous frontend may connect through MCP server.
 - Constitution must support expansion to interactive components.
reviews:
 - Claude Code or Gemini will iteratively refine files.
 - All generation must remain aligned with this constitution.
```

## Response snapshot

Updated the project constitution with the Physical AI & Humanoid Robotics textbook requirements. Created 6 core principles:
1. Technical Accuracy & Educational Clarity
2. Modular & Structured Content
3. Docusaurus Compatibility & Standards
4. AI Integration Ready
5. Student-Friendly Accessibility
6. Simulation & Practical Focus

Also updated the plan template to include constitution checks aligned with these principles.

## Outcome

- ✅ Impact: Created initial project constitution and updated dependent templates
- 🧪 Tests: N/A for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md and .specify/templates/plan-template.md
- 🔁 Next prompts: /sp.specify to create feature specifications
- 🧠 Reflection: Constitution properly captures the textbook generation requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification with /sp.specify