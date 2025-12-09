# Data Model: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data model for the Physical AI & Humanoid Robotics textbook project. Since this is primarily a content project, the "data model" focuses on the content structure and metadata rather than traditional database entities.

## Content Entities

### Chapter
- **Fields**:
  - id: string (unique identifier, e.g., "chapter-01")
  - title: string (chapter title)
  - position: integer (chapter number in sequence)
  - wordCount: range (e.g., "1200-1500")
  - prerequisiteKnowledge: string (prerequisite knowledge description)
  - learningOutcomes: array of strings (3 measurable learning outcomes)
  - subtopics: array of strings (3-5 subtopics)
  - status: enum (draft, reviewed, validated, published)
  - authors: array of strings (author names)
  - reviewers: array of strings (reviewer names)
  - createdDate: date
  - lastModified: date
  - publishedDate: date (nullable)

- **Validation Rules**:
  - title must be 10-100 characters
  - position must be between 1-18
  - wordCount must be within 500-2000 range
  - learningOutcomes must have exactly 3 items
  - subtopics must have 3-5 items
  - status must be one of the defined enum values

### Section
- **Fields**:
  - id: string (unique identifier, e.g., "ch01-s01")
  - chapterId: string (foreign key to Chapter)
  - title: string (section title)
  - content: string (section content in Markdown)
  - position: integer (position within chapter)
  - type: enum (text, code-example, illustration, interactive-element)

- **Validation Rules**:
  - chapterId must reference an existing Chapter
  - position must be positive
  - type must be one of the defined enum values

### Illustration
- **Fields**:
  - id: string (unique identifier, e.g., "ill-01-01")
  - chapterId: string (foreign key to Chapter)
  - title: string (illustration title)
  - description: string (brief description)
  - filePath: string (path to the illustration file)
  - type: enum (diagram, 3d-render, photograph, chart, animation)
  - altText: string (accessibility text)
  - caption: string (figure caption)
  - position: integer (position within chapter)

- **Validation Rules**:
  - chapterId must reference an existing Chapter
  - filePath must exist in the assets directory
  - type must be one of the defined enum values

### CodeExample
- **Fields**:
  - id: string (unique identifier, e.g., "code-01-01")
  - chapterId: string (foreign key to Chapter)
  - title: string (example title)
  - description: string (brief description of what the code does)
  - code: string (the actual code content)
  - language: string (programming language)
  - dependencies: array of strings (required libraries/frameworks)
  - testResults: string (output or expected behavior)
  - position: integer (position within chapter)

- **Validation Rules**:
  - chapterId must reference an existing Chapter
  - language must be a supported programming language
  - code must be syntactically valid for the specified language

### InteractiveElement
- **Fields**:
  - id: string (unique identifier, e.g., "ie-01-01")
  - chapterId: string (foreign key to Chapter)
  - title: string (element title)
  - description: string (brief description)
  - componentType: enum (simulation-viewer, code-playground, visualization, assessment)
  - props: object (configuration for the interactive element)
  - position: integer (position within chapter)

- **Validation Rules**:
  - chapterId must reference an existing Chapter
  - componentType must be one of the defined enum values

### PrerequisiteAssessment
- **Fields**:
  - id: string (unique identifier, e.g., "pa-01-01")
  - chapterId: string (foreign key to Chapter)
  - title: string (assessment title)
  - questions: array of objects (assessment questions)
  - passingScore: integer (minimum score to pass)
  - timeLimit: integer (time limit in minutes, nullable)

- **Validation Rules**:
  - chapterId must reference an existing Chapter
  - questions must contain at least 3 items
  - passingScore must be between 0-100

## Relationships

### Chapter contains Sections
- One Chapter has many Sections
- Each Section belongs to exactly one Chapter

### Chapter contains Illustrations
- One Chapter has many Illustrations
- Each Illustration belongs to exactly one Chapter

### Chapter contains CodeExamples
- One Chapter has many CodeExamples
- Each CodeExample belongs to exactly one Chapter

### Chapter contains InteractiveElements
- One Chapter has many InteractiveElements
- Each InteractiveElement belongs to exactly one Chapter

### Chapter has PrerequisiteAssessment
- One Chapter has zero or one PrerequisiteAssessment
- Each PrerequisiteAssessment belongs to exactly one Chapter

## State Transitions

### Chapter States
- draft → reviewed (when initial draft is complete and ready for review)
- reviewed → validated (when technical validation is complete)
- validated → published (when all reviews and validation are complete)

### Validation Requirements
- To transition from draft to reviewed: Content must be complete with all required sections
- To transition from reviewed to validated: Technical validation by domain experts must be complete
- To transition from validated to published: All quality checks and accessibility requirements must be met

## Content Metadata

### Frontmatter Schema
Each chapter file will include the following frontmatter:

```yaml
title: Chapter Title
description: Brief description of the chapter
sidebar_position: Integer position in sidebar
wordCount: "Range (e.g., 1200-1500)"
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
status: draft | reviewed | validated | published
authors:
  - "Author Name"
reviewers:
  - "Reviewer Name"
```

## File Structure Implications

The data model supports the following directory structure:

```
content/
├── textbook/
│   ├── chapter-01/
│   │   ├── index.md (contains chapter content and frontmatter)
│   │   ├── sections/
│   │   │   ├── section-01.md
│   │   │   └── section-02.md
│   │   ├── illustrations/
│   │   │   ├── diagram-01.svg
│   │   │   └── render-01.png
│   │   ├── code-examples/
│   │   │   ├── example-01.py
│   │   │   └── example-01.js
│   │   └── interactive/
│   │       └── element-01.mdx
│   └── shared/
│       ├── assessments/
│       └── common-assets/
```