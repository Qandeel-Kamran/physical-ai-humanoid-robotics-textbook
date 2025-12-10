from fastapi import APIRouter, Depends, HTTPException, status, Request
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from .services.rag_service import rag_service
from .services.vector_store import vector_store
from .api.auth import router as auth_router
from ..subagents import SubagentsController
import asyncio

router = APIRouter()

# Initialize subagents controller
subagents_controller = SubagentsController()

class QueryRequest(BaseModel):
    query: str
    user_context: Optional[Dict[str, Any]] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    confidence: float

class ContentRequest(BaseModel):
    content: str
    metadata: Dict[str, Any]
    doc_id: Optional[str] = None

class PersonalizeRequest(BaseModel):
    content: str
    user_profile: Dict[str, Any]

class PersonalizeResponse(BaseModel):
    original_content: str
    personalized_content: str
    experience_level: str
    modifications: List[str]

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "ur"

class TranslateResponse(BaseModel):
    original_content: str
    translated_content: str
    target_language: str
    status: str
    message: str

# Include auth routes
router.include_router(auth_router, prefix="/auth", tags=["auth"])

@router.post("/chat", response_model=QueryResponse)
async def chat_endpoint(request: QueryRequest):
    """Chat endpoint that uses RAG to answer questions"""
    try:
        result = await rag_service.get_answer(
            query=request.query,
            user_context=request.user_context
        )
        return QueryResponse(**result)
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing query: {str(e)}"
        )

@router.post("/add-content")
async def add_content_endpoint(request: ContentRequest):
    """Add content to the vector store"""
    try:
        doc_id = await vector_store.add_textbook_content(
            content=request.content,
            metadata=request.metadata,
            doc_id=request.doc_id
        )
        return {"doc_id": doc_id, "message": "Content added successfully"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error adding content: {str(e)}"
        )

@router.get("/search")
async def search_endpoint(query: str, limit: int = 5):
    """Search for content in the vector store"""
    try:
        results = await vector_store.search_content(query, limit=limit)
        return {"results": results}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error searching content: {str(e)}"
        )

@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """Personalize content based on user profile"""
    try:
        result = await subagents_controller.process_content(
            request.content,
            operations=['personalize'],
            user_profile=request.user_profile
        )

        return PersonalizeResponse(
            original_content=request.content,
            personalized_content=result['final_content'],
            experience_level=result.get('personalization', {}).get('experience_level', 'intermediate'),
            modifications=result.get('personalization', {}).get('modifications', [])
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error personalizing content: {str(e)}"
        )

@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """Translate content to target language"""
    try:
        result = await subagents_controller.process_content(
            request.content,
            operations=['translate'],
            target_language=request.target_language
        )

        return TranslateResponse(
            original_content=request.content,
            translated_content=result['final_content'],
            target_language=request.target_language,
            status=result.get('translation', {}).get('status', 'success'),
            message=result.get('translation', {}).get('message', 'Translation completed')
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating content: {str(e)}"
        )