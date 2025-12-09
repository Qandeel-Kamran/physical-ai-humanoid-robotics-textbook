# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This quickstart guide provides a rapid introduction to the Physical AI & Humanoid Robotics textbook project structure, tools, and processes. It's designed for contributors who need to get up and running quickly.

## Project Structure
```
specs/001-textbook-generation/
├── plan.md              # Implementation plan
├── research.md          # Research summary and decisions
├── data-model.md        # Content data model
├── quickstart.md        # This file
└── spec.md              # Feature specification

content/
├── textbook/
│   ├── chapter-01/
│   │   ├── index.md     # Chapter content with frontmatter
│   │   └── assets/      # Chapter-specific assets
│   ├── chapter-02/
│   ├── ...
│   └── chapter-18/
```

## Prerequisites
- Git for version control
- Node.js (v16 or higher) for Docusaurus
- Text editor with Markdown support (VS Code recommended)
- Basic understanding of robotics and AI concepts

## Setting Up the Environment

### 1. Clone the Repository
```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Local Development Server
```bash
npm start
```
This will start a local development server at http://localhost:3000

## Creating New Content

### 1. Create a New Chapter
```bash
# Create chapter directory
mkdir content/textbook/chapter-01

# Create the main chapter file
touch content/textbook/chapter-01/index.md
```

### 2. Add Chapter Frontmatter
```yaml
---
title: Chapter Title
description: Brief description of the chapter
sidebar_position: 1
wordCount: "1200-1500"
prerequisites: "Prerequisite knowledge summary"
learningOutcomes:
  - "Learning outcome 1"
  - "Learning outcome 2"
  - "Learning outcome 3"
subtopics:
  - "Subtopic 1"
  - "Subtopic 2"
  - "Subtopic 3"
  - "Subtopic 4"
  - "Subtopic 5"
status: draft
authors:
  - "Author Name"
reviewers:
  - "Reviewer Name"
---
```

### 3. Write Content
Use standard Markdown syntax with the following enhancements:
- Use LaTeX for mathematical expressions: `$x = y + z$`
- Include diagrams using Mermaid: ```mermaid
- Add interactive elements with MDX components
- Reference other chapters with `[Chapter Title](./other-chapter)`

## Adding Assets

### Illustrations
- Place images in `content/textbook/chapter-XX/assets/`
- Use SVG for diagrams, PNG for photographs
- Name files descriptively: `ch01-bipedal-gait-pattern.svg`

### Code Examples
- Place code files in `content/textbook/chapter-XX/code-examples/`
- Use appropriate file extensions (.py, .cpp, .js, etc.)
- Include comments explaining key concepts

### Interactive Elements
- Create MDX files in `content/textbook/chapter-XX/interactive/`
- Use Docusaurus' MDX capabilities for interactive components

## Writing Guidelines

### Content Structure
1. **Introduction**: Brief overview of the chapter topic
2. **Main Content**: Organized in H2 and H3 headings
3. **Examples**: Practical examples with code when applicable
4. **Summary**: Key takeaways from the chapter
5. **Exercises**: Practice problems for readers
6. **Further Reading**: References and additional resources

### Style Guidelines
- Use active voice when possible
- Define technical terms when first introduced
- Include visual aids to support complex concepts
- Maintain consistent terminology throughout
- Write for graduate-level understanding

## Review Process

### 1. Self-Review Checklist
- [ ] Content aligns with learning outcomes
- [ ] Prerequisite knowledge is appropriate
- [ ] Mathematical notation is correct
- [ ] Code examples are functional
- [ ] All links work correctly
- [ ] Images have appropriate alt text
- [ ] Content follows accessibility guidelines

### 2. Peer Review
- Submit a pull request with your changes
- Request review from domain experts
- Address all feedback before merging
- Ensure content passes all validation checks

## Publishing Workflow

### 1. Multi-Phase Delivery
- **Core Chapters**: First 6 chapters for initial release
- **Extended Content**: Remaining chapters in subsequent releases
- **Interactive Elements**: Added in final phase

### 2. Quality Assurance
- Technical validation by domain experts
- Accessibility testing
- Cross-browser compatibility
- Performance optimization

## Tools and Commands

### Development
```bash
# Start development server
npm start

# Build for production
npm run build

# Serve built site locally
npm run serve

# Run validation checks
npm run validate
```

### Content Management
```bash
# Check for broken links
npm run check-links

# Validate frontmatter
npm run validate-frontmatter
```

## Getting Help
- For technical issues: Refer to the Docusaurus documentation
- For content questions: Consult the project specification
- For collaboration: Use the project's communication channels
- For domain expertise: Contact the review team