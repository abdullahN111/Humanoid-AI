# API Contracts: Vision-Language-Action (VLA) Integration

**Feature**: [spec.md](../spec.md) | **Plan**: [plan.md](../plan.md) | **Date**: 2025-12-30

## Overview

This document defines the API contracts for the Vision-Language-Action (VLA) integration module. These contracts specify the interfaces between the voice processing, cognitive planning, and execution components of the VLA system. The contracts ensure consistent communication between services and provide clear specifications for integration with the ROS 2 framework.

## Service Architecture

The VLA system consists of three main services that communicate through well-defined APIs:

1. **Voice Processing Service**: Handles speech-to-text conversion and intent recognition
2. **Cognitive Planning Service**: Uses LLMs to generate action plans from natural language goals
3. **Plan Execution Service**: Executes action plans and monitors execution state

## Voice Processing Service API

### 1. Process Voice Command

**Endpoint**: `POST /api/v1/voice/process`

**Description**: Processes audio input through speech-to-text and intent recognition to generate a structured command intent.

**Request**:
```json
{
  "audioData": "base64 encoded audio data or file URL",
  "language": "string (default: 'en-US')",
  "userId": "string (optional user identifier)",
  "context": {
    "environmentState": "EnvironmentState object",
    "robotState": "RobotState object",
    "conversationHistory": "array of previous commands and responses"
  }
}
```

**Response (Success)**:
```json
{
  "success": true,
  "commandId": "string (UUID of processed command)",
  "transcript": "string (text from speech-to-text)",
  "intent": "CommandIntent object",
  "confidence": "number (0.0-1.0)",
  "processingTime": "number (milliseconds)",
  "timestamp": "ISO 8601 timestamp"
}
```

**Response (Error)**:
```json
{
  "success": false,
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional error details)"
  }
}
```

**HTTP Status Codes**:
- 200: Success
- 400: Bad Request (invalid input)
- 401: Unauthorized (if authentication required)
- 413: Payload Too Large (audio file too large)
- 500: Internal Server Error
- 503: Service Unavailable (STT service unavailable)

**Rate Limiting**: 10 requests per minute per user

### 2. Get Voice Command Status

**Endpoint**: `GET /api/v1/voice/command/{commandId}`

**Description**: Retrieves the status and result of a previously submitted voice command.

**Response (Success)**:
```json
{
  "commandId": "string",
  "status": "string ('processing', 'completed', 'failed')",
  "result": "VoiceCommand object (if completed)",
  "error": "object (if failed)",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Command not found
- 500: Internal Server Error

## Cognitive Planning Service API

### 1. Generate Action Plan

**Endpoint**: `POST /api/v1/planning/generate`

**Description**: Generates an action plan from a natural language goal using LLMs.

**Request**:
```json
{
  "goal": "string (natural language goal)",
  "context": {
    "robotCapabilities": "array of string (e.g., ['navigation', 'manipulation'])",
    "environmentState": "EnvironmentState object",
    "robotState": "RobotState object",
    "constraints": {
      "timeLimit": "number (seconds)",
      "safetyConstraints": "array of string",
      "resourceLimits": "object"
    }
  },
  "userId": "string (optional user identifier)",
  "planType": "string ('navigation', 'manipulation', 'complex', 'custom')"
}
```

**Response (Success)**:
```json
{
  "success": true,
  "planId": "string (UUID of generated plan)",
  "plan": "ActionPlan object",
  "confidence": "number (0.0-1.0)",
  "processingTime": "number (milliseconds)",
  "timestamp": "ISO 8601 timestamp"
}
```

**Response (Error)**:
```json
{
  "success": false,
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional error details)"
  }
}
```

**HTTP Status Codes**:
- 200: Success
- 400: Bad Request (invalid goal or context)
- 401: Unauthorized (if authentication required)
- 422: Unprocessable Entity (goal is impossible given constraints)
- 500: Internal Server Error
- 503: Service Unavailable (LLM service unavailable)

**Rate Limiting**: 5 requests per minute per user

### 2. Validate Action Plan

**Endpoint**: `POST /api/v1/planning/validate`

**Description**: Validates an action plan for feasibility and safety before execution.

**Request**:
```json
{
  "plan": "ActionPlan object",
  "context": {
    "robotState": "RobotState object",
    "environmentState": "EnvironmentState object"
  }
}
```

**Response (Success)**:
```json
{
  "isValid": "boolean",
  "issues": "array of ValidationIssue objects",
  "suggestedChanges": "array of PlanStep objects (optional suggested modifications)",
  "riskLevel": "string ('low', 'medium', 'high', 'critical')"
}
```

**HTTP Status Codes**:
- 200: Success
- 400: Bad Request (invalid plan format)
- 500: Internal Server Error

### 3. Get Plan Status

**Endpoint**: `GET /api/v1/planning/plan/{planId}`

**Description**: Retrieves the status and details of a generated plan.

**Response (Success)**:
```json
{
  "planId": "string",
  "status": "string ('generated', 'validated', 'executing', 'completed', 'failed')",
  "plan": "ActionPlan object",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Plan not found
- 500: Internal Server Error

## Plan Execution Service API

### 1. Execute Action Plan

**Endpoint**: `POST /api/v1/execution/execute`

**Description**: Starts execution of an action plan and returns an execution ID.

**Request**:
```json
{
  "planId": "string (ID of plan to execute)",
  "executionParameters": {
    "speedFactor": "number (0.1-2.0, execution speed multiplier)",
    "safetyLevel": "string ('conservative', 'normal', 'aggressive')",
    "maxRetries": "number (max retries per failed step)",
    "userInterventionMode": "string ('auto', 'confirm', 'manual')"
  },
  "context": {
    "robotState": "RobotState object",
    "environmentState": "EnvironmentState object"
  }
}
```

**Response (Success)**:
```json
{
  "success": true,
  "executionId": "string (UUID of execution instance)",
  "initialState": "ExecutionState object",
  "timestamp": "ISO 8601 timestamp"
}
```

**Response (Error)**:
```json
{
  "success": false,
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional error details)"
  }
}
```

**HTTP Status Codes**:
- 200: Success
- 400: Bad Request (invalid plan or parameters)
- 404: Plan not found
- 409: Conflict (plan already executing)
- 500: Internal Server Error

### 2. Get Execution Status

**Endpoint**: `GET /api/v1/execution/status/{executionId}`

**Description**: Retrieves the current status of a plan execution.

**Response (Success)**:
```json
{
  "executionId": "string",
  "status": "string ('initializing', 'executing', 'paused', 'completed', 'failed', 'cancelled')",
  "currentState": "ExecutionState object",
  "progress": "number (0.0-1.0)",
  "estimatedCompletionTime": "ISO 8601 timestamp",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Execution not found
- 500: Internal Server Error

### 3. Pause Execution

**Endpoint**: `POST /api/v1/execution/pause/{executionId}`

**Description**: Pauses execution of an action plan.

**Response (Success)**:
```json
{
  "success": true,
  "executionId": "string",
  "previousState": "string (state before pause)",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Execution not found
- 409: Conflict (execution not in pausable state)
- 500: Internal Server Error

### 4. Resume Execution

**Endpoint**: `POST /api/v1/execution/resume/{executionId}`

**Description**: Resumes execution of a paused action plan.

**Response (Success)**:
```json
{
  "success": true,
  "executionId": "string",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Execution not found
- 409: Conflict (execution not in paused state)
- 500: Internal Server Error

### 5. Cancel Execution

**Endpoint**: `POST /api/v1/execution/cancel/{executionId}`

**Description**: Cancels execution of an action plan.

**Response (Success)**:
```json
{
  "success": true,
  "executionId": "string",
  "finalState": "ExecutionState object",
  "timestamp": "ISO 8601 timestamp"
}
```

**HTTP Status Codes**:
- 200: Success
- 404: Execution not found
- 500: Internal Server Error

## WebSocket Endpoints for Real-time Updates

### 1. Execution Updates

**Endpoint**: `WS /api/v1/execution/updates/{executionId}`

**Description**: Provides real-time updates on plan execution progress.

**Message Format**:
```json
{
  "type": "string ('step_started', 'step_completed', 'step_failed', 'execution_completed', 'execution_failed')",
  "data": "object (execution update data)",
  "timestamp": "ISO 8601 timestamp"
}
```

### 2. Voice Command Updates

**Endpoint**: `WS /api/v1/voice/updates/{commandId}`

**Description**: Provides real-time updates on voice command processing.

**Message Format**:
```json
{
  "type": "string ('processing', 'transcribed', 'intent_recognized', 'completed', 'failed')",
  "data": "object (voice command update data)",
  "timestamp": "ISO 8601 timestamp"
}
```

## Authentication and Authorization

### 1. API Keys

All API endpoints require authentication via API keys:

**Header**: `Authorization: Bearer {api_key}`

### 2. Token-based Authentication

For long-running operations, token-based authentication is supported:

**Header**: `X-Auth-Token: {auth_token}`

## Error Handling

### Standard Error Format

All error responses follow this format:

```json
{
  "success": false,
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional error details)",
    "timestamp": "ISO 8601 timestamp"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| VLA_001 | 400 | Invalid audio data format |
| VLA_002 | 400 | Invalid natural language goal |
| VLA_003 | 400 | Invalid action plan format |
| VLA_004 | 401 | Authentication required |
| VLA_005 | 404 | Resource not found |
| VLA_006 | 409 | Resource conflict |
| VLA_007 | 413 | Audio file too large |
| VLA_008 | 422 | Unprocessable entity (impossible plan) |
| VLA_009 | 500 | Internal server error |
| VLA_010 | 503 | Service unavailable |

## Rate Limiting

### Default Limits

- Voice Processing: 10 requests/minute per user
- Cognitive Planning: 5 requests/minute per user
- Plan Execution: 20 requests/minute per user

### Burst Limits

- Voice Processing: 20 requests in 10 seconds
- Cognitive Planning: 10 requests in 10 seconds
- Plan Execution: 40 requests in 10 seconds

## Request/Response Validation

### JSON Schema Validation

All requests and responses are validated against JSON Schema definitions that match the data models specified in [data-model.md](../data-model.md).

### Content Types

- Requests: `application/json`
- Responses: `application/json`
- Audio uploads: `multipart/form-data` or base64-encoded in JSON

## Versioning

### API Versioning

APIs are versioned using the URL path: `/api/v{version}/...`

Current version: `v1`

### Backward Compatibility

- Breaking changes require a new major version
- New optional fields are added without version increment
- Deprecation period of 6 months before removing fields

## Monitoring and Observability

### Required Headers

All requests should include:

- `X-Request-ID`: Unique identifier for the request
- `X-User-ID`: User identifier (if available)
- `X-Session-ID`: Session identifier (if available)

### Response Headers

All responses include:

- `X-Request-ID`: Echo of the request ID
- `X-Processing-Time`: Time taken to process the request (in milliseconds)
- `X-RateLimit-Remaining`: Remaining requests in current window
- `X-RateLimit-Reset`: Time when rate limit resets

## Integration with ROS 2

### ROS 2 Service Definitions

The API contracts align with ROS 2 service definitions that mirror the API endpoints:

- `ProcessVoiceCommand.srv` - Matches voice processing API
- `GenerateActionPlan.srv` - Matches planning API
- `ExecutePlan.srv` - Matches execution API

### Message Types

The JSON objects in the API directly correspond to ROS 2 message types defined in [data-model.md](../data-model.md).

## Security Considerations

### Input Validation

- All string inputs are sanitized to prevent injection attacks
- Audio files are validated for proper format and size
- Natural language inputs are filtered for malicious content

### Data Encryption

- API keys are encrypted at rest
- Communication uses TLS 1.3
- Sensitive data is encrypted in transit and at rest

### Access Control

- API keys are scoped to specific services
- User permissions control access to resources
- Audit logging tracks all API calls