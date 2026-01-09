# Quickstart: Retrieval Pipeline Validation

**Feature**: 006-retrieval-validation
**Created**: 2026-01-05
**Status**: Complete

## Overview

This quickstart guide provides instructions for setting up and running the retrieval pipeline validation system that ensures the RAG system accurately returns relevant book content from the vector database.

## Prerequisites

- Python 3.9 or higher
- Access to the existing backend with Qdrant Cloud configuration
- The data ingestion pipeline must have already been run to populate the vector database
- Qdrant Cloud collection with stored embeddings and metadata

## Setup Instructions

### 1. Extend Existing Backend Project

The validation system will be integrated into the existing backend project created for the data ingestion feature:

```bash
# Navigate to the existing backend directory
cd backend
```

### 2. Add Validation Dependencies

Add any additional dependencies needed for validation:

```bash
# Navigate to backend directory
cd backend

# Add validation-specific dependencies
uv add pytest scikit-learn pandas numpy
```

### 3. Environment Variables

Ensure the following environment variables are set (these should already exist from the data ingestion feature):

```bash
# Check that these variables are in your .env file:
# QDRANT_URL=your_qdrant_cloud_url
# QDRANT_API_KEY=your_qdrant_api_key
# COHERE_API_KEY=your_cohere_api_key (if needed for comparison)
```

### 4. Project Structure

The validation system extends the existing backend structure:

```
backend/
├── src/
│   ├── validation/           # New validation modules
│   │   ├── __init__.py
│   │   ├── connection.py     # Qdrant connection management
│   │   ├── search.py         # Similarity search implementation
│   │   ├── metrics.py        # Validation metrics calculation
│   │   └── consistency.py    # Content consistency verification
│   └── tests/                # Test datasets and validation logic
│       ├── __init__.py
│       ├── datasets.py       # Test datasets with known queries
│       └── validators.py     # Validation logic
├── scripts/
│   ├── validate_retrieval.py # Main validation script
│   └── generate_report.py    # Report generation script
├── docs/
│   └── validation_results.md # Documentation of validation results
└── tests/                    # Validation tests
    ├── __init__.py
    ├── test_retrieval.py     # Retrieval validation tests
    └── test_consistency.py   # Consistency verification tests
```

## Running the Validation Pipeline

### 1. Execute the Validation Script

```bash
# Run the validation pipeline
python scripts/validate_retrieval.py

# Or with specific options
python scripts/validate_retrieval.py --batch-size 10 --similarity-threshold 0.7
```

### 2. Generate Validation Reports

```bash
# Generate detailed validation reports
python scripts/generate_report.py
```

## Key Components

### Validation Pipeline Flow

1. **Connection Setup**: Connect to existing Qdrant Cloud collection
2. **Query Execution**: Execute test queries against the vector database
3. **Result Retrieval**: Fetch relevant chunks with similarity scores
4. **Validation Logic**: Compare retrieved results with expected outcomes
5. **Metrics Calculation**: Compute accuracy metrics (precision, recall, F1-score)
6. **Result Reporting**: Generate validation reports and document edge cases

### Configuration

The system uses the following configuration:

- Qdrant connection: Uses existing backend configuration
- Validation batch size: 10 (configurable)
- Similarity threshold: 0.7 (configurable)
- Test dataset: Predefined queries with expected results

## Validation Metrics

The system calculates the following metrics:

- **Precision**: Proportion of retrieved documents that are relevant
- **Recall**: Proportion of relevant documents that are retrieved
- **F1-score**: Harmonic mean of precision and recall
- **Mean Reciprocal Rank (MRR)**: Quality measure of ranking
- **Hit Rate**: Proportion of queries that return at least one relevant result

## Troubleshooting

### Common Issues

1. **Qdrant Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
2. **Empty Results**: Check that the vector database has been populated by the ingestion pipeline
3. **Validation Failures**: Review the validation logs for specific error details

### Logging

The system logs validation results to `backend/logs/` directory with different levels:
- `info`: General validation progress information
- `warning`: Non-critical validation issues
- `error`: Validation errors that require attention
- `debug`: Detailed information for troubleshooting