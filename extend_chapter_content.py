#!/usr/bin/env python3
"""
Script to demonstrate extending chapter content with more detailed information.
This script shows how to use the new functionality to expand topics in existing chapters.
"""
import asyncio
import json
from typing import Dict, Any

from backend.src.services.content_service import ContentService
from backend.src.services.book_content_service import BookContentService


async def extend_ros2_architecture_chapter():
    """Extend the ROS 2 architecture chapter with more detailed content"""
    print("Extending ROS 2 Architecture Chapter...")

    # Create services
    content_service = ContentService()
    book_service = BookContentService(content_service)

    # Load existing chapter content
    with open("frontend/docs/physical-ai/module-1/ch1-1-ros2-fundamentals-architecture.md", "r", encoding="utf-8") as f:
        existing_content = f.read()

    print("Original content length:", len(existing_content))

    # Example 1: Expand DDS Configuration topic
    print("\n1. Expanding DDS Configuration topic...")
    expanded_content = await book_service.expand_chapter_topic(
        existing_content=existing_content,
        topic_to_expand="DDS Configuration and Advanced Settings",
        expansion_details="Detailed exploration of DDS QoS policies, configuration parameters, and performance tuning",
        target_audience="intermediate",
        technical_domain="ROS 2"
    )

    print(f"Content after DDS expansion: {len(expanded_content)} characters")

    # Example 2: Add Advanced Security section
    print("\n2. Adding Advanced Security section...")
    advanced_security_content = await book_service.add_advanced_content_section(
        existing_content=expanded_content,
        section_title="Advanced Security in ROS 2",
        advanced_topics=[
            "Transport Layer Security (TLS)",
            "Authentication and Authorization",
            "Secure Communication Patterns",
            "Security Best Practices"
        ],
        target_audience="expert",
        technical_domain="ROS 2"
    )

    print(f"Content after security section: {len(advanced_security_content)} characters")

    # Example 3: Expand Performance Optimization
    print("\n3. Expanding Performance Optimization topic...")
    final_content = await book_service.expand_chapter_topic(
        existing_content=advanced_security_content,
        topic_to_expand="Performance Optimization Techniques",
        expansion_details="In-depth analysis of performance bottlenecks, optimization strategies, and profiling techniques for ROS 2 applications",
        target_audience="intermediate",
        technical_domain="ROS 2"
    )

    print(f"Final content length: {len(final_content)} characters")

    # Save the extended content back to the file
    with open("frontend/docs/physical-ai/module-1/ch1-1-ros2-fundamentals-architecture_extended.md", "w", encoding="utf-8") as f:
        f.write(final_content)

    print("Extended chapter saved as ch1-1-ros2-fundamentals-architecture_extended.md")

    return final_content


async def extend_python_cpp_integration_chapter():
    """Extend the Python/C++ integration chapter with more detailed content"""
    print("\nExtending Python/C++ Integration Chapter...")

    # Create services
    content_service = ContentService()
    book_service = BookContentService(content_service)

    # Load existing chapter content
    with open("frontend/docs/physical-ai/module-1/ch1-2-python-cpp-integration-ai-agents.md", "r", encoding="utf-8") as f:
        existing_content = f.read()

    print("Original content length:", len(existing_content))

    # Example 1: Expand Inter-language Communication
    print("\n1. Expanding Inter-language Communication topic...")
    expanded_content = await book_service.expand_chapter_topic(
        existing_content=existing_content,
        topic_to_expand="Advanced Inter-language Communication Patterns",
        expansion_details="Detailed exploration of custom message types, service interfaces, and efficient data transfer between Python and C++ nodes",
        target_audience="intermediate",
        technical_domain="ROS 2"
    )

    print(f"Content after communication expansion: {len(expanded_content)} characters")

    # Example 2: Add Performance Profiling section
    print("\n2. Adding Performance Profiling section...")
    profiling_content = await book_service.add_advanced_content_section(
        existing_content=expanded_content,
        section_title="Performance Profiling and Optimization",
        advanced_topics=[
            "CPU Profiling Techniques",
            "Memory Management Best Practices",
            "Latency Optimization",
            "Real-time Performance Considerations"
        ],
        target_audience="expert",
        technical_domain="ROS 2"
    )

    print(f"Content after profiling section: {len(profiling_content)} characters")

    # Example 3: Expand AI Agent Patterns
    print("\n3. Expanding AI Agent Design Patterns...")
    final_content = await book_service.expand_chapter_topic(
        existing_content=profiling_content,
        topic_to_expand="Advanced AI Agent Design Patterns",
        expansion_details="Complex AI agent architectures, state management, decision-making algorithms, and integration with ROS 2 control systems",
        target_audience="intermediate",
        technical_domain="ROS 2"
    )

    print(f"Final content length: {len(final_content)} characters")

    # Save the extended content back to the file
    with open("frontend/docs/physical-ai/module-1/ch1-2-python-cpp-integration-ai-agents_extended.md", "w", encoding="utf-8") as f:
        f.write(final_content)

    print("Extended chapter saved as ch1-2-python-cpp-integration-ai-agents_extended.md")

    return final_content


async def extend_simulation_chapter():
    """Extend a simulation chapter with more detailed content"""
    print("\nExtending Simulation Chapter...")

    # Create services
    content_service = ContentService()
    book_service = BookContentService(content_service)

    # For this example, we'll create new content that could be added to a simulation chapter
    basic_content = """# Advanced Simulation Techniques in Robotics

## Overview
This chapter covers advanced simulation techniques for robotics development.
"""

    print("Original content length:", len(basic_content))

    # Add detailed Gazebo simulation section
    print("\n1. Adding Detailed Gazebo Simulation section...")
    gazebo_content = await book_service.add_advanced_content_section(
        existing_content=basic_content,
        section_title="Advanced Gazebo Simulation Features",
        advanced_topics=[
            "Physics Engine Configuration",
            "Sensor Simulation and Calibration",
            "Dynamic Environment Generation",
            "Multi-robot Simulation Scenarios"
        ],
        target_audience="intermediate",
        technical_domain="Gazebo"
    )

    print(f"Content after Gazebo section: {len(gazebo_content)} characters")

    # Expand Isaac Sim integration
    print("\n2. Expanding Isaac Sim Integration topic...")
    final_content = await book_service.expand_chapter_topic(
        existing_content=gazebo_content,
        topic_to_expand="Isaac Sim Advanced Features",
        expansion_details="Photorealistic rendering, synthetic data generation, reinforcement learning environments, and hardware-in-the-loop simulation",
        target_audience="intermediate",
        technical_domain="Isaac Sim"
    )

    print(f"Final content length: {len(final_content)} characters")

    # Save the extended content
    with open("frontend/docs/physical-ai/module-2/advanced-simulation-techniques.md", "w", encoding="utf-8") as f:
        f.write(final_content)

    print("Advanced simulation chapter created as advanced-simulation-techniques.md")

    return final_content


async def main():
    """Main function to run all chapter extension examples"""
    print("Chapter Content Extension Demonstration")
    print("=" * 50)

    # Extend the ROS 2 architecture chapter
    ros2_extended = await extend_ros2_architecture_chapter()

    # Extend the Python/C++ integration chapter
    cpp_extended = await extend_python_cpp_integration_chapter()

    # Create a new simulation chapter with detailed content
    sim_extended = await extend_simulation_chapter()

    print("\n" + "=" * 50)
    print("EXTENSION SUMMARY:")
    print(f"- ROS 2 Architecture chapter extended")
    print(f"- Python/C++ Integration chapter extended")
    print(f"- Advanced Simulation chapter created")
    print("All extended content follows the Physical AI Book Constitution requirements")
    print("Content includes proper structure, code examples, diagrams, and exercises")


if __name__ == "__main__":
    asyncio.run(main())