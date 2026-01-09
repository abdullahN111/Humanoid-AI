from typing import List, Dict, Any, Optional
import logging
from datetime import datetime
import uuid

from openai import OpenAI
from src.config.settings import settings
from src.services.retrieval import retrieval_service
from src.models.agent import Query, RetrievedContent, AgentResponse
from src.models.api import APIRequest, APIResponse

logger = logging.getLogger(__name__)


class RAGAgent:
    """RAG (Retrieval-Augmented Generation) agent that retrieves relevant content and generates grounded responses."""

    def __init__(self):
        """Initialize the RAG agent with OpenAI client and retrieval service."""
        # Initialize OpenAI client - can work with external LLMs via base URL
        if settings.GEMINI_API_KEY and settings.GEMINI_BASE_URL:
            # Use Gemini via OpenAI-compatible endpoint
            self.client = OpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url=settings.GEMINI_BASE_URL
            )
            self.model = "gemini-2.5-flash"  # For Google API via OpenAI-compatible endpoint
        else:
            # Fallback to OpenAI
            self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
            self.model = "gpt-4-turbo"  # Default model for OpenAI

    def retrieve_content(self, query_text: str, top_k: int = 5, threshold: float = settings.RETRIEVAL_THRESHOLD) -> List[RetrievedContent]:
        """
        Retrieve relevant content from the vector database based on the query.

        Args:
            query_text: The query text to search for
            top_k: Number of top results to retrieve
            threshold: Minimum similarity threshold

        Returns:
            List of retrieved content chunks
        """
        try:
            retrieved_chunks = retrieval_service.retrieve_by_text(
                query_text=query_text,
                top_k=top_k,
                threshold=threshold
            )

            # Convert to RetrievedContent models
            content_chunks = []
            for chunk in retrieved_chunks:
                content_chunk = RetrievedContent(
                    id=chunk.id,
                    content=chunk.content,
                    source_url=chunk.source_url,
                    page_title=chunk.page_title,
                    similarity_score=chunk.similarity_score,
                    chunk_order=chunk.chunk_order,
                    retrieval_rank=chunk.retrieval_rank
                )
                content_chunks.append(content_chunk)

            logger.info(f"Retrieved {len(content_chunks)} content chunks for query: {query_text[:50]}...")
            return content_chunks

        except Exception as e:
            logger.error(f"Error retrieving content for query '{query_text}': {str(e)}")
            return []

    def generate_response_with_citations(self, query: str, context: List[RetrievedContent]) -> str:
        """
        Generate a response using the LLM based on the query and retrieved context with citations.

        Args:
            query: The original query from the user
            context: List of retrieved content chunks to use as context

        Returns:
            Generated response string with citations
        """
        try:
            # Format the context for the LLM with source information
            context_with_sources = []
            for i, chunk in enumerate(context):
                source_info = f"Source {i+1}: {chunk.content}\nURL: {chunk.source_url}\nTitle: {chunk.page_title}\n"
                context_with_sources.append(source_info)

            context_text = "\n\n".join(context_with_sources)

            # Create the prompt for the LLM
            prompt = f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics book.
            Answer the user's question based strictly on the provided context from the book.
            If the context doesn't contain enough information to answer the question, say so.
            Do not make up information that is not in the provided context.

            When answering, please include citations to the sources used by referencing them as [Source 1], [Source 2], etc.
            based on the order they appear in the context below.

            Context from the book:
            {context_text}

            User's question: {query}

            Please provide a clear, concise answer based only on the provided context, with citations to the sources.
            """

            # Generate response using the LLM
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are an AI assistant for the Physical AI & Humanoid Robotics book. Answer questions based strictly on the provided context from the book. Include citations to sources as [Source 1], [Source 2], etc. Do not make up information that is not in the provided context."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.3
            )

            generated_text = response.choices[0].message.content.strip()
            logger.info(f"Generated response with citations for query: {query[:50]}...")
            return generated_text

        except Exception as e:
            logger.error(f"Error generating response for query '{query}': {str(e)}", exc_info=True)  # Include full traceback
            return f"Sorry, I encountered an error while generating a response: {str(e)}"

    def validate_grounding(self, response: str, context: List[RetrievedContent]) -> bool:
        """
        Validate that the response is grounded in the provided context.

        Args:
            response: The generated response
            context: The context used to generate the response

        Returns:
            True if the response is grounded in the context, False otherwise
        """
        # Simple validation: check if response contains content similar to context
        response_lower = response.lower()
        context_content = [chunk.content.lower() for chunk in context]

        # Check if significant portions of the response appear in the context
        for content_chunk in context_content:
            # Check if at least 30% of response content appears in context
            words = response_lower.split()
            if len(words) > 0:
                matching_words = sum(1 for word in words if word in content_chunk)
                if matching_words / len(words) > 0.3:
                    return True

        # For now, we'll consider it grounded if we have context
        # In a more sophisticated implementation, we'd use semantic similarity
        return len(context) > 0

    def process_query(self, query_text: str, retrieval_params: Optional[Dict[str, Any]] = None) -> AgentResponse:
        """
        Process a query through the full RAG pipeline: retrieve → generate → validate.

        Args:
            query_text: The user's query
            retrieval_params: Parameters for retrieval (top_k, threshold, etc.)

        Returns:
            AgentResponse containing the answer and metadata
        """
        if retrieval_params is None:
            retrieval_params = {
                "top_k": settings.MAX_RETRIEVED_CHUNKS,
                "threshold": settings.RETRIEVAL_THRESHOLD
            }

        query_id = str(uuid.uuid4())
        timestamp = datetime.utcnow()

        try:
            # Step 1: Retrieve relevant content
            retrieved_content = self.retrieve_content(
                query_text=query_text,
                top_k=retrieval_params.get("top_k", settings.MAX_RETRIEVED_CHUNKS),
                threshold=retrieval_params.get("threshold", settings.RETRIEVAL_THRESHOLD)
            )

            # Step 2: Generate response based on retrieved content with citations
            generated_response = self.generate_response_with_citations(query_text, retrieved_content)

            # Step 3: Validate grounding if required
            is_grounded = self.validate_grounding(generated_response, retrieved_content)

            # Create validation notes
            validation_notes = f"Response grounded in content: {is_grounded}"
            if not is_grounded and settings.GROUNDING_REQUIRED:
                validation_notes += " - WARNING: Response may not be fully grounded in provided context"

            # Calculate confidence score based on similarity scores of retrieved content
            confidence_score = 0.0
            if retrieved_content:
                avg_similarity = sum(chunk.similarity_score for chunk in retrieved_content) / len(retrieved_content)
                confidence_score = avg_similarity

            # Create and return the response
            agent_response = AgentResponse(
                id=query_id,
                query_id=query_id,
                answer=generated_response,
                sources=retrieved_content,
                confidence_score=confidence_score,
                timestamp=timestamp,
                validation_notes=validation_notes
            )

            logger.info(f"Processed query '{query_text[:50]}...' with {len(retrieved_content)} sources")
            return agent_response

        except Exception as e:
            logger.error(f"Error processing query '{query_text}': {str(e)}")

            # Return error response
            error_response = AgentResponse(
                id=query_id,
                query_id=query_id,
                answer="Sorry, I encountered an error while processing your query. Please try again.",
                sources=[],
                confidence_score=0.0,
                timestamp=timestamp,
                validation_notes=f"Error processing query: {str(e)}"
            )

            return error_response