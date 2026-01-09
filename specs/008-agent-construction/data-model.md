# Data Model: Agent Construction

## Entities

### Query
- **Description**: A user's question or request for information from the book content
- **Fields**:
  - `id` (string): Unique identifier for the query
  - `text` (string): The actual query text from the user
  - `timestamp` (datetime): When the query was submitted
  - `user_id` (string, optional): Identifier for the user making the query
  - `metadata` (dict, optional): Additional metadata about the query

### RetrievedContent
- **Description**: Book sections retrieved from the vector database that are relevant to the query
- **Fields**:
  - `id` (string): Unique identifier for the content chunk
  - `content` (string): The actual text content from the book
  - `source_url` (string): URL or reference to the original source
  - `page_title` (string): Title of the page/chapter where content appears
  - `similarity_score` (float): How similar this content is to the query
  - `chunk_order` (int): Order of this chunk in the original document
  - `retrieval_rank` (int): Rank of this chunk in the retrieval results

### AgentResponse
- **Description**: The AI-generated answer based on retrieved content, including citations and confidence indicators
- **Fields**:
  - `id` (string): Unique identifier for the response
  - `query_id` (string): Reference to the original query
  - `answer` (string): The AI-generated answer to the query
  - `sources` (list of RetrievedContent): List of content chunks used to generate the answer
  - `confidence_score` (float): Confidence level in the response
  - `timestamp` (datetime): When the response was generated
  - `validation_notes` (string): Any notes about the validation of the response

### APIRequest
- **Description**: Structured request containing user query and metadata for the FastAPI endpoint
- **Fields**:
  - `query` (string): The user's query text
  - `user_context` (dict, optional): Context about the user or session
  - `retrieval_params` (dict, optional): Parameters for the retrieval process
  - `grounding_required` (boolean): Whether strict grounding is required

### APIResponse
- **Description**: Structured response containing the agent's answer, sources, and additional metadata
- **Fields**:
  - `success` (boolean): Whether the request was successful
  - `answer` (string): The agent's answer to the query
  - `sources` (list of dict): List of sources used to generate the answer
  - `timestamp` (datetime): When the response was generated
  - `query_id` (string): ID of the original query
  - `error` (string, optional): Error message if the request failed

## Relationships

- `Query` → `AgentResponse` (one-to-one): Each query generates one agent response
- `Query` → `RetrievedContent` (one-to-many): Each query may retrieve multiple content chunks
- `AgentResponse` → `RetrievedContent` (many-to-many): Each response uses multiple retrieved content chunks

## Validation Rules

1. **Query Validation**:
   - Query text must not be empty
   - Query text must be between 1 and 1000 characters

2. **RetrievedContent Validation**:
   - Content must not be empty
   - Similarity score must be between 0 and 1
   - Source URL must be a valid URL format

3. **AgentResponse Validation**:
   - Answer must not be empty
   - Confidence score must be between 0 and 1
   - Must include at least one source if grounding is required

4. **APIRequest Validation**:
   - Query field is required
   - User context must be a valid dictionary if provided

5. **APIResponse Validation**:
   - Success field is required
   - Answer field is required when success is true
   - Sources list must contain valid source objects when provided