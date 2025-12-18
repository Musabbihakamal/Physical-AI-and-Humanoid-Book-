# Build stage
FROM node:18-alpine as build

WORKDIR /app

# Copy package files
COPY package*.json ./
COPY frontend/package*.json ./frontend/

# Install dependencies
RUN cd frontend && npm ci

# Copy source code
COPY frontend/ /app/frontend/

# Build the application
WORKDIR /app/frontend
RUN npm run build

# Production stage
FROM nginx:alpine

# Copy built assets from build stage
COPY --from=build /app/frontend/build /usr/share/nginx/html

# Copy custom nginx configuration
COPY --from=build /app/frontend/nginx.conf /etc/nginx/nginx.conf

EXPOSE 3000

CMD ["nginx", "-g", "daemon off;"]