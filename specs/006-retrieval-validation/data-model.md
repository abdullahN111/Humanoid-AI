# Data Models: Retrieval Pipeline Validation

**Feature**: 006-retrieval-validation
**Created**: 2026-01-05
**Status**: Complete

## Overview

This document defines the data models for the retrieval pipeline validation system. These models represent the core entities that will be processed, validated, and analyzed by the system.

## Core Entities

### 1. RetrievalQuery

**Description**: A semantic search query with optional metadata filters used to test the retrieval system.

**Fields**:
- `id` (string): Unique identifier for the query (UUID)
- `query_text` (string): The text of the query being tested
- `expected_results` (list): List of expected content chunks that should be retrieved
- `metadata_filters` (object): Optional filters to apply during retrieval (URL, section, module)
- `test_scenario` (string): Description of the test scenario
- `created_at` (datetime): Time when the query was created

**Validation Rules**:
- `query_text` must not be empty
- `id` must be a valid UUID
- `expected_results` must be a valid list (can be empty)

**Relationships**:
- One-to-many with ValidationResult (one query can have multiple validation results)

### 2. RetrievedChunk

**Description**: A content chunk returned by the retrieval system with similarity score and source reference.

**Fields**:
- `id` (string): Unique identifier for the chunk (UUID)
- `content` (string): The actual content text of the chunk
- `source_url` (string): URL where the content originated
- `page_title` (string): Title of the source page
- `similarity_score` (float): Similarity score returned by the search algorithm
- `chunk_order` (integer): Order of this chunk within the source document
- `retrieval_rank` (integer): Rank of this chunk in the retrieval results
- `metadata` (object): Additional metadata from the Qdrant payload

**Validation Rules**:
- `similarity_score` must be between 0 and 1
- `retrieval_rank` must be positive
- `content` must not be empty

**Relationships**:
- Many-to-one with RetrievalQuery (multiple chunks retrieved per query)
- Part of ValidationResult (used to determine validation outcome)

### 3. ValidationResult

**Description**: The outcome of validating a retrieved chunk against the original query and source content.

**Fields**:
- `id` (string): Unique identifier for the validation result (UUID)
- `query_id` (string): Reference to the associated RetrievalQuery
- `retrieved_chunks` (list): List of retrieved chunks for this query
- `is_relevant` (boolean): Whether the retrieved chunks are relevant to the query
- `relevance_score` (float): Quantitative measure of relevance (0-1)
- `accuracy_metrics` (object): Detailed accuracy metrics (precision, recall, etc.)
- `validation_notes` (string): Notes about the validation process
- `validated_at` (datetime): Time when the validation was performed

**Validation Rules**:
- `relevance_score` must be between 0 and 1
- `query_id` must reference an existing RetrievalQuery
- `retrieved_chunks` must be a valid list

**Relationships**:
- Many-to-one with RetrievalQuery (multiple validation results per query)
- Contains RetrievedChunk entities

### 4. AccuracyMetric

**Description**: A quantitative measure of how well the retrieval system performs, including precision and recall measures.

**Fields**:
- `id` (string): Unique identifier for the metric set (UUID)
- `precision` (float): Proportion of retrieved documents that are relevant
- `recall` (float): Proportion of relevant documents that are retrieved
- `f1_score` (float): Harmonic mean of precision and recall
- `mrr` (float): Mean Reciprocal Rank of relevant documents
- `hit_rate` (float): Proportion of queries that return at least one relevant result
- `mean_similarity` (float): Average similarity score of retrieved results
- `total_queries` (integer): Total number of queries tested
- `total_relevant_retrieved` (integer): Total number of relevant chunks retrieved
- `total_expected_retrieved` (integer): Total number of expected chunks retrieved
- `calculated_at` (datetime): Time when the metrics were calculated

**Validation Rules**:
- All score fields must be between 0 and 1
- `total_queries` must be non-negative
- `total_relevant_retrieved` must be non-negative
- `total_expected_retrieved` must be non-negative

**Relationships**:
- One-to-many with ValidationResult (metrics calculated from multiple validations)

### 5. ValidationReport

**Description**: A comprehensive report of the validation process and results.

**Fields**:
- `id` (string): Unique identifier for the report (UUID)
- `report_title` (string): Title of the validation report
- `validation_summary` (object): Summary of validation results
- `accuracy_metrics` (AccuracyMetric): Overall accuracy metrics
- `edge_cases_found` (list): List of edge cases discovered during validation
- `errors_encountered` (list): List of errors encountered during validation
- `test_dataset_info` (object): Information about the test dataset used
- `environment_info` (object): Information about the environment where validation was run
- `generated_at` (datetime): Time when the report was generated

**Validation Rules**:
- `report_title` must not be empty
- `accuracy_metrics` must be a valid AccuracyMetric object

**Relationships**:
- Contains AccuracyMetric for overall results
- References multiple ValidationResult entities

## Data Flow

1. **Input Data**: RetrievalQuery with test questions and expected results
2. **Processing**: Execute queries against Qdrant and retrieve chunks
3. **Validation**: Compare retrieved chunks with expected results
4. **Metrics**: Calculate accuracy metrics from validation results
5. **Reporting**: Generate comprehensive validation reports

## Constraints and Guarantees

1. **Integrity**: Validation results must accurately reflect retrieval performance
2. **Completeness**: All test queries must be processed and validated
3. **Consistency**: Metrics must be calculated using consistent methods
4. **Traceability**: Each validation result must be traceable to its source query
5. **Reproducibility**: Validation process must produce consistent results across runs