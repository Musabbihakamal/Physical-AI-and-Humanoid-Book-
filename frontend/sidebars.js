// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'physical-ai-book',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'physical-ai/module-1/ch1-introduction-to-physical-ai',
        'physical-ai/module-1/ch1-1-ros2-fundamentals-architecture',
        'physical-ai/module-1/ch1-2-python-cpp-integration-ai-agents',
        'physical-ai/module-1/ch1-3-robot-description-control',
        'physical-ai/module-1/ch1-4-advanced-ros2-concepts',
        'physical-ai/module-1/ch2-ros2-architecture-fundamentals',
        'physical-ai/module-1/ch3-nodes-topics-publishers-subscribers',
        'physical-ai/module-1/ch4-controlling-robots-with-rclpy',
        'physical-ai/module-1/ch5-building-humanoid-urdfs'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation (Gazebo & Unity)',
      items: [
        'physical-ai/module-2/ch1-gazebo-simulation-environment-setup',
        'physical-ai/module-2/ch2-urdf-sdf-robot-description-formats',
        'physical-ai/module-2/ch3-physics-simulation-sensor-simulation',
        'physical-ai/module-2/ch4-unity-robotics-visualization',
        'physical-ai/module-2/ch5-integrating-ros2-nodes-with-gazebo-unity'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'physical-ai/module-3/ch1-nvidia-isaac-sim-foundations',
        'physical-ai/module-3/ch2-perception-pipelines-in-isaac-ros',
        'physical-ai/module-3/ch3-vslam-navigation',
        'physical-ai/module-3/ch4-training-ai-models-in-simulation',
        'physical-ai/module-3/ch5-bipedal-path-planning-with-nav2'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) and Capstone Project',
      items: [
        'physical-ai/module-4/ch1-introduction-to-vla',
        'physical-ai/module-4/ch2-voice-to-action-whisper',
        'physical-ai/module-4/ch3-cognitive-planning-with-llms',
        'physical-ai/module-4/ch4-multi-modal-interaction',
        'physical-ai/module-4/ch5-capstone-project-humanoid-ai-companion'
      ],
    },
  ],
};

module.exports = sidebars;