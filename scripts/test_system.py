#!/usr/bin/env python3
"""
Test script to verify the complete system is working correctly
"""

import requests
import time
import sys

def test_system():
    print("Testing Physical AI & Humanoid Robotics Book System...")
    print("="*60)

    # Test backend health
    print("1. Testing backend health...")
    try:
        response = requests.get("http://localhost:8000/health", timeout=10)
        if response.status_code == 200:
            print("   [OK] Backend is healthy")
            health_data = response.json()
            print(f"   Health details: {health_data}")
        else:
            print(f"   [ERROR] Backend health check failed with status {response.status_code}")
    except Exception as e:
        print(f"   [ERROR] Backend health check failed: {e}")

    # Test frontend accessibility
    print("\n2. Testing frontend accessibility...")
    try:
        response = requests.get("http://localhost:3000", timeout=10)
        if response.status_code == 200:
            print("   [OK] Frontend is accessible")
            # Check if the main book page is loaded
            if "Physical AI & Humanoid Robotics" in response.text or "Book" in response.text:
                print("   [OK] Main book page is loaded correctly")
            else:
                print("   [WARN] Main book page content not found")
        else:
            print(f"   [ERROR] Frontend check failed with status {response.status_code}")
    except Exception as e:
        print(f"   [ERROR] Frontend check failed: {e}")

    # Test book content pages
    print("\n3. Testing book content accessibility...")
    book_pages = [
        "/docs/physical-ai-book",
        "/docs/physical-ai/intro",
        "/docs/physical-ai/ros2-essentials",
        "/docs/physical-ai/gazebo-simulation",
        "/docs/physical-ai/unity-visualization",
        "/docs/physical-ai/isaac-sim",
        "/docs/physical-ai/isaac-ros-rl",
        "/docs/physical-ai/vla-systems",
        "/docs/physical-ai/conversational-robotics",
        "/docs/physical-ai/capstone-humanoid"
    ]

    accessible_pages = 0
    for page in book_pages:
        try:
            response = requests.get(f"http://localhost:3000{page}", timeout=10)
            if response.status_code == 200:
                print(f"   [OK] {page} - Accessible")
                accessible_pages += 1
            else:
                print(f"   [ERROR] {page} - Status {response.status_code}")
        except Exception as e:
            print(f"   [ERROR] {page} - Error: {e}")

    print(f"\n   Book pages accessible: {accessible_pages}/{len(book_pages)}")

    # Test deployment guide
    print("\n4. Testing deployment guide...")
    try:
        response = requests.get("http://localhost:3000/docs/deployment-guide", timeout=10)
        if response.status_code == 200:
            print("   [OK] Deployment guide is accessible")
        else:
            print(f"   [ERROR] Deployment guide check failed with status {response.status_code}")
    except Exception as e:
        print(f"   [ERROR] Deployment guide check failed: {e}")

    print("\n" + "="*60)
    print("System test completed!")

    if accessible_pages == len(book_pages):
        print("[SUCCESS] All book content is accessible!")
        print("The Physical AI & Humanoid Robotics Book system is working correctly.")
        print("\nAccess the system at:")
        print("- Frontend: http://localhost:3000")
        print("- Backend API: http://localhost:8000")
        print("- Book content: http://localhost:3000/docs/physical-ai-book")
        return True
    else:
        print(f"[WARN] Only {accessible_pages}/{len(book_pages)} book pages are accessible.")
        return False

if __name__ == "__main__":
    success = test_system()
    sys.exit(0 if success else 1)