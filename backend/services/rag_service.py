import asyncio
from typing import List, Dict, Any
import openai
from .vector_store import vector_store
from .config.settings import settings

class RAGService:
    def __init__(self):
        openai.api_key = settings.OPENAI_API_KEY

    async def get_answer(self, query: str, user_context: Dict[str, Any] = None) -> Dict[str, Any]:
        """Generate answer using RAG approach"""
        # Search for relevant content
        search_results = await vector_store.search_content(query, limit=5)

        if not search_results:
            return {
                "answer": "I couldn't find relevant information in the textbook to answer your question.",
                "sources": [],
                "confidence": 0.0
            }

        # Prepare context from search results
        context_parts = []
        sources = []

        for result in search_results:
            context_parts.append(result["content"])
            sources.append({
                "id": result["id"],
                "metadata": result["metadata"],
                "relevance_score": result["score"]
            })

        # Combine context
        context = "\n\n".join(context_parts)

        # Prepare the prompt with user context if available
        user_background = ""
        if user_context:
            user_background = f"\nUser Background:\n- Software Experience: {user_context.get('software_experience', 'Not specified')}\n- Hardware Experience: {user_context.get('hardware_experience', 'Not specified')}\n- Robotics Background: {user_context.get('robotics_background', 'Not specified')}\n- AI/ML Experience: {user_context.get('ai_ml_experience', 'Not specified')}\n\n"

        prompt = f"""You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer the user's question based on the provided context from the textbook. Be concise, accurate, and helpful.

{user_background}
Context from textbook:
{context}

Question: {query}

Answer:"""

        # Get response from OpenAI
        response = await openai.ChatCompletion.acreate(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context. Be accurate, concise, and helpful. If the context doesn't contain enough information to answer the question, say so."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.3
        )

        answer = response.choices[0].message.content

        # Calculate confidence based on relevance scores
        avg_score = sum([s["relevance_score"] for s in sources]) / len(sources) if sources else 0.0

        return {
            "answer": answer,
            "sources": sources,
            "confidence": avg_score
        }

# Global instance
rag_service = RAGService()