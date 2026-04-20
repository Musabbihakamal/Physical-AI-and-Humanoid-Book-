"""
Simple indexing script to populate RAG bot with sample content
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add parent directory to path for imports
sys.path.append('..')

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import cohere
import uuid

def create_sample_content():
    """Create sample robotics content for testing"""
    return [
        {
            "content": "ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
            "url": "https://docs.ros.org/en/humble/",
            "page_title": "ROS 2 Documentation",
            "section_title": "Introduction to ROS 2"
        },
        {
            "content": "Gazebo is a 3D robotics simulator that provides accurate physics simulation, high-quality graphics, and convenient programmatic and graphical interfaces. It's widely used for testing robotics algorithms and robot designs.",
            "url": "https://gazebosim.org/docs",
            "page_title": "Gazebo Documentation",
            "section_title": "What is Gazebo"
        },
        {
            "content": "Isaac Sim is NVIDIA's robotics simulation platform built on Omniverse. It provides photorealistic rendering, accurate physics simulation, and synthetic data generation for training AI models in robotics applications.",
            "url": "https://docs.omniverse.nvidia.com/isaacsim/",
            "page_title": "Isaac Sim Documentation",
            "section_title": "Isaac Sim Overview"
        },
        {
            "content": "URDF (Unified Robot Description Format) is an XML format for representing a robot model. It describes the robot's physical properties, including links, joints, sensors, and visual elements needed for simulation and control.",
            "url": "https://wiki.ros.org/urdf",
            "page_title": "URDF Documentation",
            "section_title": "Robot Description Format"
        },
        {
            "content": "Physical AI combines artificial intelligence with robotics to create systems that can interact with and manipulate the physical world. This includes perception, planning, control, and learning in real-world environments.",
            "url": "https://example.com/physical-ai",
            "page_title": "Physical AI Guide",
            "section_title": "Introduction to Physical AI"
        }
    ]

def index_content():
    """Index sample content into Qdrant"""
    print("Starting RAG bot content indexing...")

    # Initialize clients
    try:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        print("Connected to Qdrant")

        cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        print("Connected to Cohere")

    except Exception as e:
        print(f"Failed to connect to services: {e}")
        return False

    collection_name = "docusaurus_embeddings"

    # Create collection if it doesn't exist
    try:
        collections = qdrant_client.get_collections()
        collection_exists = any(c.name == collection_name for c in collections.collections)

        if not collection_exists:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
            )
            print(f"Created collection: {collection_name}")
        else:
            print(f"Collection already exists: {collection_name}")

    except Exception as e:
        print(f"Failed to create collection: {e}")
        return False

    # Get sample content
    content_items = create_sample_content()
    print(f"Indexing {len(content_items)} content items...")

    # Generate embeddings and store
    try:
        texts = [item["content"] for item in content_items]

        # Generate embeddings
        response = cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        embeddings = response.embeddings
        print("Generated embeddings")

        # Create points for Qdrant
        points = []
        for i, (item, embedding) in enumerate(zip(content_items, embeddings)):
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "content": item["content"],
                    "url": item["url"],
                    "page_title": item["page_title"],
                    "section_title": item["section_title"]
                }
            )
            points.append(point)

        # Upload to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        print(f"Uploaded {len(points)} points to Qdrant")

        # Verify indexing
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"Collection now has {collection_info.points_count} total points")

        return True

    except Exception as e:
        print(f"Failed to index content: {e}")
        return False

if __name__ == "__main__":
    success = index_content()
    if success:
        print("\nRAG bot indexing completed successfully!")
        print("You can now test the RAG chat with questions like:")
        print("   - 'What is ROS 2?'")
        print("   - 'Tell me about Gazebo'")
        print("   - 'What is Isaac Sim?'")
    else:
        print("\nRAG bot indexing failed!")