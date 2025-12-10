from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
import asyncio
import sys
import os

# Add the parent directory to the path to import from the same level
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from .database import get_db
from .models.user import User
from .services.rag_service import rag_service
from .services.vector_store import vector_store
from .config.settings import settings
from .api import router as api_router

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook RAG API",
    description="API for the RAG chatbot integrated with the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")

@app.on_event("startup")
async def startup_event():
    """Initialize the application"""
    print("Application starting up...")
    # You can add initialization logic here if needed

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Textbook RAG API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Add textbook content to vector store on startup (for demo purposes)
@app.on_event("startup")
async def load_textbook_content():
    """Load textbook content into vector store"""
    # This is a simplified example - in reality, you'd load from your actual textbook content
    print("Loading textbook content into vector store...")

    # Example content - in reality, this would come from your textbook files
    sample_chapters = [
        {
            "title": "Introduction to Physical AI",
            "content": "Physical AI represents a paradigm shift from traditional artificial intelligence approaches, emphasizing the importance of physical embodiment in creating intelligent systems. Unlike conventional AI that operates primarily in digital environments, Physical AI systems interact with the physical world through sensors and actuators, enabling them to learn and adapt through direct physical engagement with their environment.",
            "metadata": {"chapter": "1", "title": "Introduction to Physical AI", "url": "/docs/chapter-01-introduction"}
        },
        {
            "title": "Biomechanics and Human Movement",
            "content": "Biomechanics is the study of the structure, function, and motion of the mechanical aspects of biological systems. In humanoid robotics, understanding human biomechanics is crucial for creating robots that can move naturally and efficiently. This includes understanding joint mechanics, muscle activation patterns, and energy efficiency in movement.",
            "metadata": {"chapter": "2", "title": "Biomechanics and Human Movement", "url": "/docs/chapter-02-biomechanics"}
        },
        {
            "title": "ROS 2 Fundamentals",
            "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
            "metadata": {"chapter": "3", "title": "ROS 2 Fundamentals", "url": "/docs/chapter-03-ros2"}
        }
    ]

    for i, chapter in enumerate(sample_chapters):
        await vector_store.add_textbook_content(
            content=chapter["content"],
            metadata=chapter["metadata"],
            doc_id=f"chapter_{i+1}"
        )

    print(f"Loaded {len(sample_chapters)} textbook chapters into vector store")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)