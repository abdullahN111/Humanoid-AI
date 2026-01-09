from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from src.config.settings import settings
from src.api.routes import agent, health
import logging

# Set up logging
from src.utils.helpers import setup_logging
setup_logging()

logger = logging.getLogger(__name__)


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title=settings.app_name if hasattr(settings, 'app_name') else "RAG Agent API",
        version=settings.app_version if hasattr(settings, 'app_version') else "1.0.0",
        debug=settings.DEBUG if hasattr(settings, 'DEBUG') else False,
        docs_url="/docs",
        redoc_url="/redoc"
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.allowed_origins if hasattr(settings, 'allowed_origins') else ["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routes
    app.include_router(agent.router, prefix=settings.API_PREFIX if hasattr(settings, 'API_PREFIX') else "/api", tags=["agent"])
    app.include_router(health.router, prefix=settings.API_PREFIX if hasattr(settings, 'API_PREFIX') else "/api", tags=["health"])

    # Add exception handlers
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request, exc):
        logger.error(f"Validation error: {exc}")
        return JSONResponse(
            status_code=422,
            content={"detail": exc.errors(), "body": exc.body}
        )

    @app.exception_handler(Exception)
    async def general_exception_handler(request, exc):
        logger.error(f"General error: {exc}")
        return JSONResponse(
            status_code=500,
            content={"detail": "Internal server error"}
        )

    return app


app = create_app()


# Validate configuration on startup
@app.on_event("startup")
async def startup_event():
    logger.info("Starting up RAG Agent API")
    try:
        from src.config.settings import settings
        errors = settings.validate()
        if errors:
            logger.error(f"Configuration validation errors: {errors}")
            raise Exception(f"Configuration validation failed: {errors}")
        else:
            logger.info("Configuration validated successfully")
    except Exception as e:
        logger.error(f"Startup validation failed: {e}")
        raise


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )