@echo off
REM Ingest content from Vercel deployment
echo ========================================
echo RAG Content Ingestion
echo ========================================
echo.
echo Ingesting content from: https://physical-ai-and-humanoid-book.vercel.app/
echo.

call .venv\Scripts\activate
if errorlevel 1 (
    echo ERROR: Virtual environment not found. Run setup.bat first.
    pause
    exit /b 1
)

python main.py --mode ingest --url https://physical-ai-and-humanoid-book.vercel.app/

echo.
echo ========================================
echo Ingestion Complete!
echo ========================================
pause
