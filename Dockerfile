# CARLA V2V Research Platform - Full Container
# 
# This container runs the complete V2V platform including:
# - Web-based control panel and visualization server
# - CARLA Python client for simulation control
# - Connects to an external CARLA simulator server over the network
#
# Build:   docker build -t carla-v2v .  OR  docker compose build
# Run:     docker run -p 8000:8000 carla-v2v
# Access:  http://localhost:8000

FROM python:3.10-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app

# Set working directory
WORKDIR /app

# Install system dependencies for CARLA client and numpy
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libjpeg-dev \
    libpng-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.docker.txt requirements.txt

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ ./src/
COPY start_server.py .

# Create logs directory
RUN mkdir -p logs data

# Expose the web server port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/')" || exit 1

# Run the server
CMD ["python", "start_server.py"]
