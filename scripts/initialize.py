#!/usr/bin/env python3
"""
Project Initialization Script
This script sets up the complete project environment in a cross-platform way.
"""

import os
import sys
import subprocess
import platform
from pathlib import Path


def run_command(cmd, cwd=None, shell=True):
    """Run a command and return the result."""
    print(f"Running: {cmd}")
    try:
        result = subprocess.run(
            cmd,
            shell=shell,
            cwd=cwd,
            capture_output=True,
            text=True,
            check=True
        )
        print(f"✅ Success: {cmd}")
        if result.stdout:
            print(f"Output: {result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed: {cmd}")
        print(f"Error: {e.stderr}")
        return False


def check_python_version():
    """Check if Python 3.11+ is available."""
    major, minor = sys.version_info[:2]
    if major < 3 or (major == 3 and minor < 11):
        print(f"❌ Python 3.11+ required. Current version: {major}.{minor}")
        return False
    print(f"✅ Python version {major}.{minor} is compatible")
    return True


def check_node_version():
    """Check if Node.js 18+ is available."""
    try:
        result = subprocess.run(
            ["node", "--version"],
            capture_output=True,
            text=True,
            check=True
        )
        version = result.stdout.strip().replace('v', '')
        major_version = int(version.split('.')[0])
        if major_version < 18:
            print(f"❌ Node.js 18+ required. Current version: {version}")
            return False
        print(f"✅ Node.js version {version} is compatible")
        return True
    except (subprocess.CalledProcessError, ValueError):
        print("❌ Node.js is not installed or not in PATH")
        return False


def install_backend_dependencies():
    """Install backend Python dependencies."""
    print("📦 Installing backend dependencies...")
    backend_path = Path("backend")

    if not backend_path.exists():
        print("❌ Backend directory not found")
        return False

    # Create virtual environment
    venv_path = backend_path / "venv"
    if platform.system().lower() == "windows":
        python_exe = str(venv_path / "Scripts" / "python")
        pip_exe = str(venv_path / "Scripts" / "pip")
    else:
        python_exe = str(venv_path / "bin" / "python")
        pip_exe = str(venv_path / "bin" / "pip")

    # Create virtual environment
    if not venv_path.exists():
        print("Creating virtual environment...")
        if not run_command(f"python -m venv {venv_path}", cwd="backend"):
            return False

    # Upgrade pip
    if not run_command(f"{pip_exe} install --upgrade pip", cwd="backend"):
        return False

    # Install requirements
    if not run_command(f"{pip_exe} install -r requirements.txt", cwd="backend"):
        return False

    print("✅ Backend dependencies installed")
    return True


def install_frontend_dependencies():
    """Install frontend Node.js dependencies."""
    print("📦 Installing frontend dependencies...")
    frontend_path = Path("frontend")

    if not frontend_path.exists():
        print("❌ Frontend directory not found")
        return False

    if not run_command("npm install", cwd="frontend"):
        return False

    print("✅ Frontend dependencies installed")
    return True


def create_env_files():
    """Create environment files if they don't exist."""
    print("📝 Creating environment configuration...")

    # Backend .env
    backend_env = Path("backend/.env")
    if not backend_env.exists():
        backend_env_example = Path("backend/.env.example")
        if backend_env_example.exists():
            backend_env_example.rename(backend_env)
            print("✅ Created backend .env file")
        else:
            # Create a basic .env file
            backend_env.write_text("""# Backend Environment Variables
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=postgresql://localhost/book_agent_db
CLAUDE_API_KEY=your_claude_api_key
""")
            print("✅ Created basic backend .env file")

    # Frontend .env
    frontend_env = Path("frontend/.env")
    if not frontend_env.exists():
        frontend_env_example = Path("frontend/.env.example")
        if frontend_env_example.exists():
            frontend_env_example.rename(frontend_env)
            print("✅ Created frontend .env file")
        else:
            # Create a basic .env file
            frontend_env.write_text("""# Frontend Environment Variables
REACT_APP_BACKEND_URL=http://localhost:8000
REACT_APP_RAG_ENABLED=true
""")
            print("✅ Created basic frontend .env file")

    print("✅ Environment configuration complete")
    return True


def initialize_database():
    """Initialize the database if needed."""
    print("💾 Initializing database...")

    # This is a simplified database initialization
    # In a real project, you'd run your actual database initialization
    backend_path = Path("backend")
    if (backend_path / "src" / "database" / "database.py").exists():
        venv_path = backend_path / "venv"
        if platform.system().lower() == "windows":
            python_exe = str(venv_path / "Scripts" / "python")
        else:
            python_exe = str(venv_path / "bin" / "python")

        # Try to initialize the database
        init_cmd = f'{python_exe} -c "import sys; sys.path.insert(0, \'src\'); from database.database import init_db; init_db()"'
        if run_command(init_cmd, cwd="backend"):
            print("✅ Database initialized")
        else:
            print("⚠️  Database initialization may have failed - check backend setup")
    else:
        print("⚠️  Database initialization file not found - skipping")

    return True


def main():
    """Main initialization function."""
    print("🚀 Initializing Book + RAG Bot + Multi-Agent System...")
    print(f"Platform: {platform.system()} {platform.release()}")
    print(f"Current directory: {Path.cwd()}")

    # Check prerequisites
    if not check_python_version():
        return False

    if not check_node_version():
        return False

    # Create environment files
    if not create_env_files():
        return False

    # Install dependencies
    if not install_backend_dependencies():
        return False

    if not install_frontend_dependencies():
        return False

    # Initialize database
    initialize_database()

    print("\n🎉 Project initialization complete!")
    print("\n📋 Next steps:")
    print("1. Update environment variables in backend/.env and frontend/.env")
    print("2. Start backend: cd backend && python -m src.main (or use start_backend.py)")
    print("3. Start frontend: cd frontend && npm start")
    print("4. Access the application at http://localhost:3000")

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)