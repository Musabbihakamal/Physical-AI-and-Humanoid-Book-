# Deploy RAG Bot to Production (Vercel + Backend)

Your book is deployed on Vercel at: `https://physical-ai-and-humanoid-book.vercel.app/`

To integrate the RAG bot, you need to:
1. Deploy the backend API
2. Ingest content from your live site
3. Configure the frontend to use the production backend
4. Redeploy the frontend

## Step 1: Deploy the Backend

### Option A: Railway (Recommended - Free Tier Available)

**1. Install Railway CLI:**
```bash
npm install -g @railway/cli
```

**2. Login to Railway:**
```bash
railway login
```

**3. Deploy the backend:**
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
railway init
railway up
```

**4. Set environment variables in Railway dashboard:**
- Go to https://railway.app/dashboard
- Select your project
- Go to Variables tab
- Add these variables:
  ```
  COHERE_API_KEY=your_cohere_api_key_here
  QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
  QDRANT_API_KEY=your_qdrant_api_key_here
  PORT=8000
  ```

**5. Get your backend URL:**
Railway will provide a URL like: `https://your-app.railway.app`

### Option B: Render (Free Tier Available)

**1. Go to https://render.com and sign up**

**2. Create New Web Service:**
- Connect your GitHub repository
- Select the `backend` directory
- Configure:
  - **Name:** `book-rag-backend`
  - **Environment:** Python 3
  - **Build Command:** `pip install -r requirements.txt`
  - **Start Command:** `python -m src.main`
  - **Plan:** Free

**3. Add Environment Variables:**
```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.f9vvic1mOncyz0WOS3AvLiqn7A7JAuZ15lGoyTOsfNs
PORT=8000
```

**4. Deploy and get your URL:**
Render will provide: `https://book-rag-backend.onrender.com`

### Option C: Fly.io

**1. Install flyctl:**
```bash
# Windows PowerShell
iwr https://fly.io/install.ps1 -useb | iex
```

**2. Login and deploy:**
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
fly auth login
fly launch
fly deploy
```

**3. Set secrets:**
```bash
fly secrets set COHERE_API_KEY=your_cohere_api_key_here
fly secrets set QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
fly secrets set QDRANT_API_KEY=your_qdrant_api_key_here
```

## Step 2: Ingest Content from Your Live Site

Once the backend is deployed, ingest content from your Vercel site:

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\Activate.ps1
python main.py --mode ingest --url https://physical-ai-and-humanoid-book.vercel.app/
```

This will:
- Crawl all pages from your deployed book
- Extract and chunk the content
- Generate embeddings
- Store in Qdrant (cloud instance)

**Expected output:**
```
INFO - Starting crawl from: https://physical-ai-and-humanoid-book.vercel.app/
INFO - Discovered 50+ pages
INFO - Created 200+ chunks
INFO - Stored in Qdrant
INFO - Ingestion complete!
```

## Step 3: Configure Frontend for Production Backend

**1. Create `.env.production` file in frontend directory:**

```bash
cd "D:\PROJECT\HACKATHON 1\frontend"
```

Create file `.env.production`:
```env
REACT_APP_BACKEND_URL=https://your-backend-url.railway.app
```

Replace `your-backend-url.railway.app` with your actual backend URL from Step 1.

**2. Configure in Vercel Dashboard:**

- Go to https://vercel.com/dashboard
- Select your project: `physical-ai-and-humanoid-book`
- Go to Settings → Environment Variables
- Add:
  - **Key:** `REACT_APP_BACKEND_URL`
  - **Value:** `https://your-backend-url.railway.app`
  - **Environment:** Production

## Step 4: Redeploy Frontend to Vercel

**Option A: Push to GitHub (Automatic Deployment)**

```bash
cd "D:\PROJECT\HACKATHON 1"
git add .
git commit -m "Configure RAG bot for production deployment"
git push origin main
```

Vercel will automatically redeploy with the new environment variable.

**Option B: Manual Deployment via Vercel CLI**

```bash
cd "D:\PROJECT\HACKATHON 1\frontend"
npm install -g vercel
vercel --prod
```

## Step 5: Test the Integration

**1. Check backend health:**
```bash
curl https://your-backend-url.railway.app/health
```

Expected: `{"status":"healthy","service":"book-system"}`

**2. Check RAG endpoint:**
```bash
curl https://your-backend-url.railway.app/api/rag/health
```

Expected: `{"status":"healthy","collection":"docusaurus_embeddings","ready":true}`

**3. Test on your live site:**
- Visit: https://physical-ai-and-humanoid-book.vercel.app/
- Look for the floating chat button (bottom-right)
- Click and ask: "What is ROS 2?"
- You should get an answer with source citations!

## Troubleshooting

### Issue: Chat widget shows "Backend not running"

**Solution:** Check CORS settings in backend. Add to `backend/src/api/main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://physical-ai-and-humanoid-book.vercel.app",
        "https://*.vercel.app"  # Allow all Vercel preview deployments
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Issue: "No relevant information found"

**Solution:** Re-run the ingestion pipeline to ensure content is in Qdrant.

### Issue: Backend deployment fails

**Solution:** Check that `requirements.txt` includes all dependencies:
- fastapi
- uvicorn
- cohere
- qdrant-client
- beautifulsoup4
- lxml
- tiktoken

### Issue: Environment variable not working

**Solution:** 
1. Verify it's set in Vercel dashboard
2. Redeploy after adding the variable
3. Check browser console: `console.log(process.env.REACT_APP_BACKEND_URL)`

## Architecture Overview

```
User Browser
    ↓
Vercel (Frontend)
https://physical-ai-and-humanoid-book.vercel.app/
    ↓
Railway/Render (Backend API)
https://your-backend.railway.app/api/rag/query
    ↓
Qdrant Cloud (Vector Database)
https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io
    ↓
Cohere API (Embeddings & Generation)
```

## Cost Estimate

- **Vercel:** Free (Hobby plan)
- **Railway:** Free tier (500 hours/month) or $5/month
- **Render:** Free tier (750 hours/month) or $7/month
- **Qdrant Cloud:** Free tier (1GB storage)
- **Cohere API:** Free tier (100 API calls/minute)

**Total:** $0-7/month depending on usage

## Next Steps

1. Choose a backend hosting provider (Railway recommended)
2. Deploy the backend and get the URL
3. Run ingestion pipeline with your Vercel URL
4. Add backend URL to Vercel environment variables
5. Redeploy frontend
6. Test the RAG chat widget on your live site

## Support

- Backend logs: Check Railway/Render dashboard
- Frontend logs: Browser console (F12)
- API docs: `https://your-backend-url.railway.app/docs`
