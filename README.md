# Physical AI & Humanoid Robotics Textbook - Hackathon Implementation

This repository contains the implementation for the hackathon project to create an AI-native textbook for teaching Physical AI & Humanoid Robotics with advanced features.

## Features Implemented

### 1. Context7 MCP Server Integration for Docusaurus Documentation
- **Connected to**: `/websites/docusaurus_io_3_9_2` library via Context7 MCP server
- **Documentation Access**: Real-time access to Docusaurus 3.9.2 documentation for theme customization
- **Frontend Framework**: Docusaurus v3.9.2 for book frontend with green/white/black theme
- **Theme Customization**: Applied green/white/black theme using Context7 documentation guidance
- **Component Integration**: Leveraged Docusaurus components based on Context7 documentation

### 2. RAG Chatbot Integration
- **Backend**: FastAPI server with endpoints for chat, content management, and personalization
- **Vector Store**: Qdrant Cloud for efficient similarity search
- **Database**: Neon Serverless Postgres for user management
- **AI Integration**: OpenAI API for intelligent responses
- **Frontend**: Interactive chat widget embedded in all textbook pages

### 3. Claude Code Subagents for Reusable Intelligence
- **Textbook Analyzer**: Extracts learning outcomes, key concepts, and complexity metrics
- **Content Personalizer**: Adapts content based on user experience level
- **Translator**: Provides Urdu translation capabilities
- **Subagents Controller**: Orchestrates all subagents for coordinated functionality

### 4. User Authentication with Background Questions
- **Registration Flow**: Collects user background information:
  - Software experience level
  - Hardware experience level
  - Robotics background
  - AI/ML experience level
- **Login System**: Secure authentication with profile management
- **Profile Storage**: Maintains user preferences and background information

### 5. Chapter Personalization
- **Adaptive Content**: Modifies textbook content based on user profile
- **Experience Level Detection**: Automatically determines appropriate content complexity
- **Personalization Controls**: Easy toggle for personalized vs original content
- **Local Storage**: Remembers user preferences across sessions

### 6. Urdu Translation Capability
- **Real-time Translation**: Translate any chapter content to Urdu
- **Technical Term Dictionary**: Specialized translations for robotics/AI terminology
- **Translation Controls**: Easy toggle between original and translated content
- **Language Persistence**: Remembers translation preferences

## Table of Contents

The textbook is organized into the following chapters:

1. [Chapter 01: Introduction](docs/chapter-01-introduction.md) - Overview of Physical AI and Humanoid Robotics
2. [Chapter 02: Biomechanics](docs/chapter-02-biomechanics.md) - Human and Robot Biomechanics
3. [Chapter 03: Sensorimotor Systems](docs/chapter-03-sensorimotor.md) - Sensory and Motor Integration
4. [Chapter 04: Cognitive Architecture](docs/chapter-04-cognitive-architecture.md) - AI Systems for Humanoid Robots
5. [Chapter 05: Actuation Systems](docs/chapter-05-actuation.md) - Robot Actuators and Motors
6. [Chapter 06: Control Theory](docs/chapter-06-control-theory.md) - Control Systems for Humanoid Robots
7. [Chapter 07: Locomotion](docs/chapter-07-locomotion.md) - Walking and Movement in Humanoids
8. [Chapter 08: Manipulation](docs/chapter-08-manipulation.md) - Grasping and Object Manipulation
9. [Chapter 09: Perception](docs/chapter-09-perception.md) - Sensing and Understanding the Environment
10. [Chapter 10: Learning](docs/chapter-10-learning.md) - Machine Learning for Humanoid Robots
11. [Chapter 11: Human-Robot Interaction](docs/chapter-11-hri.md) - Interfacing with Humans
12. [Chapter 12: Simulation](docs/chapter-12-simulation.md) - Simulation Environments
13. [Chapter 13: Hardware](docs/chapter-13-hardware.md) - Physical Robot Construction
14. [Chapter 14: Energy Management](docs/chapter-14-energy.md) - Power Systems and Efficiency
15. [Chapter 15: Safety](docs/chapter-15-safety.md) - Safety Protocols and Considerations
16. [Chapter 16: Applications](docs/chapter-16-applications.md) - Real-World Applications
17. [Chapter 17: Future Directions](docs/chapter-17-future-directions.md) - Emerging Trends
18. [Chapter 18: Ethics](docs/chapter-18-ethics.md) - Ethical Considerations
19. [Chapter 19: Conclusion](docs/chapter-18-conclusion.md) - Summary and Outlook

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │    │     Backend      │    │   External      │
│   (Docusaurus)  │◄──►│    (FastAPI)     │◄──►│   Services      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
       │                       │                       │
       │              ┌──────────────────┐             │
       └─────────────►│   Subagents      │◄────────────┘
                      │   (Python)       │
                      └──────────────────┘
                              │
                      ┌──────────────────┐
                      │   Vector Store   │
                      │   (Qdrant)       │
                      └──────────────────┘
```

## API Endpoints

- `POST /api/v1/chat` - RAG-powered chat with textbook content
- `POST /api/v1/personalize` - Personalize content based on user profile
- `POST /api/v1/translate` - Translate content to target language
- `POST /api/v1/add-content` - Add content to vector store
- `GET /api/v1/search` - Search textbook content
- `POST /api/v1/auth/register` - User registration with background questions
- `POST /api/v1/auth/login` - User authentication

## Frontend Components

- `ChatbotWidget.js` - Interactive chat interface
- `AuthModal.js` - Authentication modal with registration form
- `PersonalizeButton.js` - Content personalization controls
- `TranslateButton.js` - Urdu translation controls
- `chatbot-loader.js` - Client-side chatbot initialization

## Setup Instructions

1. **Backend Setup**:
   ```bash
   cd backend
   pip install -r requirements.txt
   uvicorn main:app --reload
   ```

2. **Frontend Setup**:
   ```bash
   cd textbook-website/my-textbook-website
   npm install
   npm start
   ```

## Technologies Used

- **Backend**: FastAPI, SQLAlchemy, PostgreSQL, Qdrant, OpenAI
- **Frontend**: React, Docusaurus, JavaScript/CSS
- **AI/ML**: OpenAI embeddings, custom subagents
- **Database**: Neon Serverless Postgres
- **Vector Store**: Qdrant Cloud
- **Authentication**: Custom implementation with better-auth principles

## Bonus Features Implemented

- ✅ Up to 50 extra bonus points: Reusable intelligence via Claude Code Subagents
- ✅ Up to 50 extra bonus points: Signup and Signin with background questions
- ✅ Up to 50 extra bonus points: Chapter personalization for logged-in users
- ✅ Up to 50 extra bonus points: Urdu translation capability

## Project Structure

```
/
├── backend/                 # FastAPI backend with all services
│   ├── api/                # API routes
│   ├── services/           # Business logic
│   ├── models/             # Database models
│   ├── config/             # Configuration
│   └── main.py             # Main application
├── subagents/              # Claude Code Subagents
│   ├── textbook_analyzer.py # Content analysis
│   ├── content_personalizer.py # Personalization logic
│   ├── translator.py       # Translation capabilities
│   └── __init__.py         # Subagents controller
├── textbook-website/       # Docusaurus frontend
│   └── my-textbook-website/
│       ├── src/
│       │   └── components/ # React components
│       ├── static/js/      # Client-side scripts
│       └── docusaurus.config.js
└── README.md               # This file
```

## How to Use

1. **Chat with the textbook**: Use the chatbot widget (💬) at the bottom right to ask questions about the content
2. **Register/Login**: Click the auth button to register and provide your background information
3. **Personalize content**: Use the "Personalize Content" button on any chapter page
4. **Translate to Urdu**: Use the "Translate to Urdu" button on any chapter page
5. **Navigate**: Use the sidebar to browse different chapters

This implementation fulfills all requirements of the hackathon with bonus features for enhanced user experience.

## About

This textbook was developed as part of a comprehensive course on Physical AI & Humanoid Robotics. It covers both theoretical foundations and practical applications in the field.

## License

This textbook is provided as educational material. See the specific license terms in the repository.

## Contributing

For corrections, suggestions, or improvements, please open an issue or submit a pull request.