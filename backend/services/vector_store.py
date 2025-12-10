import asyncio
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from uuid import uuid4
import openai
from .config.settings import settings

class VectorStoreService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )
        self.collection_name = "textbook_content"
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Create collection if it doesn't exist"""
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Create collection with 1536 dimensions for OpenAI embeddings
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
            )

    async def add_textbook_content(self, content: str, metadata: Dict[str, Any], doc_id: str = None) -> str:
        """Add textbook content to vector store"""
        if not doc_id:
            doc_id = str(uuid4())

        # Get embedding from OpenAI
        response = openai.embeddings.create(
            input=content,
            model="text-embedding-ada-002"
        )
        embedding = response.data[0].embedding

        # Upsert to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=doc_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            ]
        )

        return doc_id

    async def search_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant content based on query"""
        # Get embedding for query
        response = openai.embeddings.create(
            input=query,
            model="text-embedding-ada-002"
        )
        query_embedding = response.data[0].embedding

        # Search in Qdrant
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "content": result.payload["content"],
                "metadata": result.payload["metadata"],
                "score": result.score
            })

        return results

# Global instance
vector_store = VectorStoreService()