# Base image
FROM python:3.12-slim

# Install system tools
RUN apt-get update && apt-get install -y \
    make git libxrender1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/vmod

# Copy only requirements first for caching
COPY requirements.txt ./ 

# Create venv and install dependencies
RUN pip install --upgrade pip \
    && pip install -r requirements.txt

# Copy the rest of the project
COPY . .

# Activate and test
CMD ["/bin/bash", "-c", "make test"]
